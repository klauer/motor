#include "LinmotCurve.h"
#include "LinmotMotorDriver.h"
#include <epicsThread.h>
#include <string>
#include <sstream>
#include <iomanip>


bool LmCurveInfo::set_name(const char* new_name) {
    if (!new_name || strlen((char *)new_name) >= LM_CI_STRING_LENGTH)
        return false;
    strcpy((char*)name, new_name);
    return true;
}


void LmCurveInfo::dump(FILE *fp) {
    char sname[LM_CI_STRING_LENGTH + 1];
    sname[LM_CI_STRING_LENGTH] = 0;
    memcpy(sname, name, LM_CI_STRING_LENGTH);

    fprintf(fp, "--------------------\n");
    fprintf(fp, "data_offset:      %u\n", data_offset);
    fprintf(fp, "object_type:      %u\n", object_type);
    fprintf(fp, "num_setpoints:    %u\n", num_setpoints);
    fprintf(fp, "data_type_size:   %u\n", data_type_size);
    fprintf(fp, "name:             %s\n", sname);
    fprintf(fp, "curve_id:         %u\n", curve_id);
    fprintf(fp, "x_length:         %u\n", x_length);
    fprintf(fp, "x_dim_uuid:       0x%x\n", x_dim_uuid);
    fprintf(fp, "y_dim_uuid:       0x%x\n", y_dim_uuid);
    fprintf(fp, "wizard_type:      %u\n", wizard_type);

    for (int i=0; i < LM_CI_NUM_WIZARD_PARAMS; i++) {
        fprintf(fp, "wizard_params[%d]: %u\n", i, wizard_params[i]);
    }
}


bool new_position_time_curve(struct LmCurveInfo &ci, double time_sec,
                             uint16_t num_setpoints, std::string& name,
                             uint16_t curve_id)
{
    ci.data_offset = (uint16_t)LM_CI_DATA_OFFSET;
    ci.object_type = (uint16_t)LM_OT_POSITION_VS_TIME;

    if (time_sec <= 0.0 || time_sec >= LM_MAX_CURVE_TIME)
        return false;

    ci.num_setpoints = num_setpoints;
    ci.data_type_size = 4;
    ci.set_name(name);
    ci.curve_id = curve_id;
    // x-length is in units of 10us
    ci.x_length = (uint32_t)(time_sec * LM_CI_XLENGTH_SCALE);
    ci.x_dim_uuid = LM_UUID_TIME;
    ci.y_dim_uuid = LM_UUID_POSITION;
    ci.wizard_type = 0;
    for (int i=0; i < LM_CI_NUM_WIZARD_PARAMS; i++) {
        ci.wizard_params[i] = 0;
    }
    ci.__padding__ = 0;
    return true;
}

struct LmCurveInfo *LmPositionTimeCurve::get_curve_info() {
    LmCurveInfo *ci = new LmCurveInfo;
    int num_setpoints = setpoints.size();
    if (new_position_time_curve(*ci, get_total_seconds(),
                                num_setpoints, name, curve_id)) {
        return ci;
    } else {
        delete ci;
        return NULL;
    }
}


void LmPositionTimeCurve::set_curve_info(LmCurveInfo &ci) {
    char sname[LM_CI_STRING_LENGTH + 1];
    sname[LM_CI_STRING_LENGTH] = 0;
    memcpy((void*)sname, ci.name, LM_CI_STRING_LENGTH);

    name = (const char*)sname;
    curve_id = ci.curve_id;
    if (ci.x_length > 0 && ci.num_setpoints > 0) {
        dt = (double)ci.x_length / ((double)ci.num_setpoints * LM_CI_XLENGTH_SCALE);
    } else {
        dt = 0.0;
    }
}


// TODO: having LinmotController method implementations here is likely to irk
// real C++ developers
void LinmotController::curveAccessThread() {
    while (true) {
        curveAccessEvent_.wait();
        curveLock_.lock();
        curveAccess(curve_, writeCurve_);
        curveLock_.unlock();
    }
}


// #undef DEBUG
// #define DEBUG 1

void LinmotController::curveReadBack(LmPositionTimeCurve &curve) {
    LinmotAxis *axis = (LinmotAxis*)getAxis(0);
    unsigned int num_points = curve.setpoints.size();
    unsigned int j;

    double resolution = 1.0;
    double offset = 0.0;
    int direction = 0;
    int st;

    setStringParam(profileName_, curve.name.c_str());
    setIntegerParam(profileCurveId_, curve.curve_id);

    st = getDoubleParam(axis->axisNo_, profileMotorResolution_, &resolution);
    st |= getDoubleParam(axis->axisNo_, profileMotorOffset_, &offset);
    st |= getIntegerParam(axis->axisNo_, profileMotorDirection_, &direction);

    if (num_points > LM_MAX_PROFILE_POINTS)
        num_points = LM_MAX_PROFILE_POINTS;

    printf("[st=%d] Resolution %f offset %f direction %d\n", st,
            resolution, offset, direction);
    if (resolution == 0.0) {
        resolution = 1.0;
    }

    if (st) {
        resolution = 1.0;
        offset = 0.5;
    }

    if (direction != 0) {
        resolution = -resolution;
    }

    for (j=0; j < num_points; j++) {
#if DEBUG
        printf("curve.setpoints[%d] = %f\n", j, curve.setpoints[j]);
#endif
        axis->profilePositions_[j] = resolution * curve.setpoints[j] + offset;
    }
    doCallbacksFloat64Array(axis->profilePositions_, num_points, profilePositions_, 0);
    setIntegerParam(profileNumPoints_, num_points);
    setDoubleParam(profileFixedTime_, curve.dt);
#if DEBUG
    printf("read curve back:\n");
    printf("dt is %f\n", curve.dt);
    printf("curve name is %s\n", curve.name.c_str());
    printf("num points is %d\n", num_points);
#endif
    setIntegerParam(profileRead_, 0);
    callParamCallbacks();
}

void LinmotController::curveAccess(LmPositionTimeCurve &curve, bool writing) {
    int cycle_counter = 0;

    int config_mode;

    unsigned int mode_state = 0;
    int status_word;
    int next_control_word = 0;
    int value_in;

    epicsInt32 buffer[sizeof(LmCurveInfo) >> 2]; // large enough to hold LmCurveInfo
    epicsInt32 *bptr = NULL;
    double *ppos = NULL;
    std::stringstream ss;

    int buffer_len = 0;
    int curve_id = curve.curve_id;

    int toggle = 0;
    int expected_status = -1;
    int next_mode_status = -1;
    int error_code;
    bool waiting_for_status = false;
    bool buffer_response = false;
    bool changed_mode = false;
    const bool reading = !writing;

    LmCurveInfo *curve_info = curve.get_curve_info();
    epicsTime t1, t2;

    if (writing) {
        config_mode = LM_CONFIG_CURVE_WRITE;
    } else {
        config_mode = LM_CONFIG_CURVE_READ;
        curve.name = "";
        curve.curve_id = 0;
        curve.setpoints.clear();
        curve.dt = 0.0;
    }

    // Build state: busy
    setIntegerParam(profileBuildState_, PROFILE_BUILD_BUSY);
    callParamCallbacks();

    while (true) {
        cycleEvent_.wait();

        t1 = epicsTime::getCurrent();

        if (cfgStatusWord_->read(&status_word) != asynSuccess)
            continue;

        if (cfgValueIn_->read(&value_in) != asynSuccess)
            continue;

        changed_mode = false;

        if (waiting_for_status) {
            cycle_counter++;
            if (cycle_counter >= LM_CYCLE_COUNT_TIMEOUT) {
                setCurveBuildStatus("Curve access timeout", PROFILE_BUILD_DONE,
                        PROFILE_STATUS_FAILURE);
                break;
            }

            if (status_word == next_mode_status) {
                mode_state++;
                changed_mode = true;
#if DEBUG
                printf("! Reached next mode status\n");
#endif
            } else if (status_word == expected_status) {
#if DEBUG
                printf("! Got expected status\n");
#endif
            } else {
                if ((status_word & 0xFF) == expected_status) {
                    error_code = (status_word >> 8) & 0xFF;
                    if (error_code) {
#if DEBUG
                        printf("Error code is: %x\n", error_code);
#endif
                        ss.str("");
                        ss.clear();
                        ss << "Errored ";
                        ss << std::hex << error_code;
                        setCurveBuildStatus(ss.str(), PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE);
                        break;
                    }
                }
                continue;
            }
        }

        if (buffer_response) {
            if (buffer_len > 0) {
                *bptr = value_in;
                bptr++;
                buffer_len--;
            } else {
                setCurveBuildStatus("Unexpected response size",
                        PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE);
                goto cleanup;
            }
        }

#if DEBUG
        if (buffer_response) {
            printf("Read value: %x\n", value_in);
        }
        printf("-- %d mode_state %x status_word %x value_in %x\n",
                cycle_counter, mode_state, status_word, value_in);
#endif

        switch (mode_state) {
        case LM_MODE_INIT:
            next_control_word = LM_CONFIG_INIT;
            expected_status = next_mode_status = 0x0F;
            cfgIndexOut_->write(0x0);
            cfgValueOut_->write(0x0);

            if (writing) {
                setStringParam(profileBuildMessage_, "Deleting old curve");
                deleteCurve(curve_id);
            }
            break;

        case LM_MODE_SEND_COMMAND:
            expected_status = next_mode_status = 0x01;

            cfgIndexOut_->write(curve_id);
            if (writing) {
                cfgValueOut_->write(curve_info->packed_block_size());
            } else {
                cfgValueOut_->write(0x0);
                ss.str("");
                ss.clear();
                ss << "Reading curve " << curve_id;
                setCurveBuildStatus(ss.str(), PROFILE_BUILD_BUSY);
            }

            toggle = 1;
            bptr = buffer;
            buffer_len = sizeof(buffer);
            break;

        case LM_MODE_CURVE_INFO:
            if (changed_mode) {
                if (writing) {
                    setStringParam(profileBuildMessage_, "Writing curve info");
                    bptr = (epicsInt32*)curve_info;
                    buffer_len = sizeof(LmCurveInfo) >> 2;
#if DEBUG
                    printf("Sending curve_info:\n");
                    curve_info->dump();
#endif
                } else {
                    setStringParam(profileBuildMessage_, "Reading curve info");
                }
            }

            toggle = 1 - toggle;
            expected_status = 0x402 + toggle;
            next_mode_status = 0x002 + toggle;
            buffer_response = reading;
            break;

        case LM_MODE_SETPOINTS:
            if (changed_mode) {
                toggle = 0;
                if (writing) {
                    setStringParam(profileBuildMessage_, "Writing positions");
                    ppos = &curve.setpoints[0];
                    buffer_len = curve.setpoints.size();
                } else {
                    LmCurveInfo ci;
                    memcpy(&ci, buffer, sizeof(LmCurveInfo));
                    ci.dump();
                    curve.set_curve_info(ci);
                    setStringParam(profileBuildMessage_, "Reading positions");
                }
                buffer_response = false;
            } else {
                toggle = 1 - toggle;
                curve.setpoints.push_back((double)(value_in) / LM_POSITION_SCALE);
            }

            expected_status = 0x404 + toggle;
            next_mode_status = 0x004 + toggle;
            break;

        case LM_MODE_FINISHED:
            ss.str("");
            ss.clear();
            if (writing) {
                ss << "Wrote curve " << curve_id << " " << curve_info->name;
            } else {
                ss << "Read curve " << curve_id;
                curve.setpoints.push_back((double)(value_in) / LM_POSITION_SCALE);
                curveReadBack(curve);
            }

            setCurveBuildStatus(ss.str(), PROFILE_BUILD_DONE, PROFILE_STATUS_SUCCESS);
            break;

        default:
            setCurveBuildStatus("Reached unknown state", PROFILE_BUILD_DONE,
                    PROFILE_STATUS_FAILURE);
            goto cleanup;
        }

        if (mode_state == LM_MODE_FINISHED) {
            // finally done
            break;
        }

        if (writing) {
            if (mode_state == LM_MODE_CURVE_INFO) {
                cfgValueOut_->write(*bptr);
                buffer_len--;
                if (buffer_len >= 0) {
                    bptr++;
                }
            } else if (mode_state == LM_MODE_SETPOINTS) {
                cfgValueOut_->write((epicsInt32)(*ppos * LM_POSITION_SCALE));
                if (buffer_len >= 0) {
                    ppos++;
                }
            }
        }

        waiting_for_status = true;
        cycle_counter = 0;

        if (mode_state > 0) {
            next_control_word = ((config_mode + mode_state - 1) << 8) |
                                 ((mode_state - 1) << 1) | toggle;
        }

        cfgControlWord_->write(next_control_word);

#if DEBUG
        printf("  toggle %x\n", toggle);
        printf("  buffer_response %x\n", buffer_response);
        printf("  expected_status %x\n", expected_status);
        printf("  next_mode_status %x\n", next_mode_status);
        printf("  wrote control_word %x\n", next_control_word);
        epicsInt32 value_out;
        cfgValueOut_->read(&value_out);
        printf("  wrote value_out %x\n", value_out);
        printf("  dt %f\n", 1000.0 * (epicsTime::getCurrent() - t1));
#endif
        callParamCallbacks();
    }

cleanup:
    if (curve_info) {
        delete curve_info;
        curve_info = NULL;
    }

    callParamCallbacks();
}

asynStatus LinmotController::readCurve() {
    int st = 0;
    int curveId;
    int buildState;

    st |= getIntegerParam(profileBuildState_, &buildState);

    if (buildState == PROFILE_BUILD_BUSY) {
        setStringParam(profileBuildMessage_, "Curve access busy");
        return asynError;
    }

    st |= getIntegerParam(profileCurveId_, &curveId);
    if (st) {
        setStringParam(profileBuildMessage_, "Failed to get curve id");
        return asynError;
    }

    writeCurve_ = false;
    curve_.curve_id = curveId;
    curveAccessEvent_.signal();
    return asynSuccess;
}

asynStatus LinmotController::buildProfile() {
    int st = 0;
    int timeMode;
    double time;
    int numPoints;
    int writeCurveId;
    int buildState;
    char name[LM_CI_STRING_LENGTH + 1];

    LinmotAxis *axis = (LinmotAxis*)getAxis(0);

    st |= getIntegerParam(profileBuildState_, &buildState);

    if (buildState == PROFILE_BUILD_BUSY) {
        setStringParam(profileBuildMessage_, "Curve access busy");
        return asynError;
    }

    if (asynMotorController::buildProfile() != asynSuccess)
        return asynError;

    st |= getIntegerParam(profileTimeMode_, &timeMode);
    st |= getIntegerParam(profileNumPoints_, &numPoints);
    st |= getDoubleParam(profileFixedTime_, &time);
    st |= getIntegerParam(profileCurveId_, &writeCurveId);
    if (st) {
        setStringParam(profileBuildMessage_, "Failed to get params");
        return asynError;
    }

    if (timeMode != PROFILE_TIME_MODE_FIXED) {
        setStringParam(profileBuildMessage_, "Fixed time mode only");
        goto fail;
    }

    if (writeCurveId <= 0 || writeCurveId > LM_MAX_CURVE_ID) {
        setStringParam(profileBuildMessage_, "Invalid curve ID");
        goto fail;
    }

    if (getStringParam(profileName_, LM_CI_STRING_LENGTH, (char *)name) || !name[0]) {
        strcpy(name, "IocCurve");
    }

    curve_.name = name;
    curve_.curve_id = writeCurveId;
    curve_.setpoints.clear();
    curve_.dt = time;
    for (int i=0; i < numPoints; i++) {
        curve_.setpoints.push_back(axis->profilePositions_[i]);
        printf("Setpoint[%d] = %f\n", i, axis->profilePositions_[i]);
    }

    writeCurve_ = true;
    curveAccessEvent_.signal();
    return asynSuccess;

fail:
    setIntegerParam(profileBuildStatus_, PROFILE_STATUS_FAILURE);
    return asynError;

}

asynStatus LinmotController::setCurveBuildStatus(const char *message,
  ProfileBuildState build_state,
  ProfileStatus build_status)
{
    int st = 0;
    st |= setStringParam(profileBuildMessage_, message);
    st |= setIntegerParam(profileBuildState_, build_state);
    st |= setIntegerParam(profileBuildStatus_, build_status);

    if (st)
        return asynError;
    else
        return asynSuccess;
}


asynStatus LinmotController::runCurveTotal(epicsUInt16 curve_id,
    double time_sec,
    double amplitude_scale,
    double offset
    )
{
    int st = 0;
    int buildState;

    st |= getIntegerParam(profileBuildState_, &buildState);

    if (st || buildState == PROFILE_BUILD_BUSY) {
        setStringParam(profileExecuteMessage_, "Curve access busy");
        return asynError;
    }

    if (curve_id <= 0 || curve_id > LM_MAX_CURVE_ID) {
        setStringParam(profileExecuteMessage_, "Invalid curve ID");
        return asynError;
    }

    if (time_sec <= 0.0 || time_sec >= LM_MAX_CURVE_TIME) {
        setStringParam(profileExecuteMessage_, "Bad total time");
        return asynError;
    }

    // Create records, set LOPR/HOPR
    if (amplitude_scale < -20.0 || amplitude_scale > 20.0) {
        setStringParam(profileExecuteMessage_, "Bad amplitude scale [-20, 20]");
        return asynError;
    }

    if (offset < LM_MIN_POSITION || offset > LM_MAX_POSITION) {
        setStringParam(profileExecuteMessage_, "Bad position offset");
        return asynError;
    }


    LmTimeCurveTotal tc;
    tc.curve_id = curve_id;
    tc.curve_offset = (int32_t)(offset * LM_POSITION_SCALE);
    tc.time = (int32_t)(time_sec * LM_CI_XLENGTH_SCALE);
    tc.amplitude_scale = (int16_t)(amplitude_scale * 1.0e3);

    epicsInt32 *buf = (epicsInt32 *)&tc;
    return sendCmd(LM_HDR_TIME_CURVE_TOTAL,
            *buf,
            *(buf + 1),
            *(buf + 2)
            );
}


asynStatus LinmotController::executeProfile()
{
    int st = 0;
    int curveId = 0;
    int runMode = 0;
    double ampScale = 0.0;
    double offset = 0.0;

    st = getIntegerParam(profileCurveId_, &curveId);
    st |= getIntegerParam(profileRunMode_, &runMode);
    st |= getDoubleParam(profileAmplitudeScale_, &ampScale);
    st |= getDoubleParam(profileOffset_, &offset);

    if (runMode == LM_PROFILE_MODE_SCALED) {
        double timeScale = 0.0;
        st |= getDoubleParam(profileTimeScale_, &timeScale);
        if (st)
            return asynError;
        // return runCurveScaled(curve_id, timeTotal, ampScale, offset);
        return asynSuccess;
    }

    double timeTotal = 0.0;
    st |= getDoubleParam(profileTimeTotal_, &timeTotal);

    if (st)
        return asynError;

    return runCurveTotal(curveId, timeTotal, ampScale, offset);
}


int __curve_test(void)
{
    // assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);
    // assert(__LITTLE_ENDIAN__);
    assert((offsetof(LmCurveInfo, data_offset) == 0));
    assert((offsetof(LmCurveInfo, wizard_params[6]) == 66));
    assert(sizeof(LmCurveInfo) == 72);

    assert(sizeof(LmTimeCurveTotal) == 12);

    LmCurveInfo ci;

    uint32_t info_array[] = {
        196678, 262645, 1131898190, 1702261365, 0, 0, 0, 65536, 100000, 327706,
        -2036333311, 666894337, -1591738359, 7, 0, 0, 0, 666894336
    };

    printf("%lu %lu\n", sizeof(info_array), sizeof(ci));
    assert(sizeof(info_array) >= sizeof(ci));
    memcpy(&ci, info_array, sizeof(ci));

    ci.dump();

    LmCurveInfo ci2;
    std::string curve_name("NewCurve");
    new_position_time_curve(ci2, 1.0f, 501, curve_name, 1);
    ci2.dump();

    LmPositionTimeCurve c(curve_name, 1, 0.01);
    c.setpoints.push_back(0.0);
    c.setpoints.push_back(1.0);
    c.setpoints.push_back(2.0);
    LmCurveInfo *ci3 = c.get_curve_info();
    ci3->dump();

    LmPositionTimeCurve c2;
    c2.set_curve_info(*ci3);

    delete ci3;
    return 0;
}
