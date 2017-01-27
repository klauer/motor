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


void LinmotController::cycleThreadLoop() {
    int cycle_counter = 0;

    bool writing = true;
    int config_mode;

    unsigned int mode_state = 0;
    int status_word;
    int next_control_word = 0;
    int value_in;

    epicsInt32 buffer[4096]; // TODO
    epicsInt32 *bptr = NULL;
    std::stringstream ss;

    int buffer_idx = -1;
    int buffer_len = 0;
    int i;

    int curve_id = 12;
    int toggle = 0;
    int expected_status = -1;
    int next_mode_status = -1;
    int error_code;
    bool waiting_for_status = false;
    bool buffer_response = false;
    bool changed_mode = false;

    if (writing)
        config_mode = LM_CONFIG_CURVE_WRITE;
    else
        config_mode = LM_CONFIG_CURVE_READ;

    std::string curve_name("write_test");
    LmPositionTimeCurve curve(curve_name, curve_id, 0.01);
    curve.setpoints.push_back(0.0);
    curve.setpoints.push_back(1.0);
    curve.setpoints.push_back(2.0);
    LmCurveInfo *curve_info = curve.get_curve_info();

    epicsTime t1, t2;

    epicsThreadSleep(10.0);

    if (strcmp(portName, "MOTOR0")) {
        printf("Ignoring port %s\n", portName);
        return;
    }

    while (true) {
        cycleEvent_.wait();
        // new cycle
        cycle_counter++;
        if ((cycle_counter % 3000) == 0) {
            printf("New cycle callback: %d\n", cycle_counter);
        }

        t1 = epicsTime::getCurrent();

        if (cfgStatusWord_->read(&status_word) != asynSuccess)
            continue;

        if (cfgValueIn_->read(&value_in) != asynSuccess)
            continue;

        // Build state: busy
        setIntegerParam(profileBuildState_, PROFILE_BUILD_BUSY);

        changed_mode = false;

        if (waiting_for_status) {
            if (status_word == next_mode_status) {
                mode_state++;
                changed_mode = true;
                printf("! Reached next mode status\n");
            } else if (status_word == expected_status) {
                printf("! Got expected status\n");
            } else {
                if ((status_word & 0xFF) == expected_status) {
                    error_code = (status_word >> 8) & 0xFF;
                    if (error_code) {
                        printf("Error code is: %x\n", error_code);

                        ss.str("");
                        ss.clear();
                        ss << "Errored ";
                        ss << std::hex << error_code;
                        const std::string error_message = ss.str();
                        setStringParam(profileBuildMessage_, error_message.c_str());
                        setIntegerParam(profileBuildStatus_, PROFILE_STATUS_FAILURE);
                        setIntegerParam(profileBuildState_, PROFILE_BUILD_DONE);
                        goto cleanup;
                    }
                }
                continue;
            }
        }

        if (buffer_response) {
            buffer[buffer_idx++] = value_in;
            printf("Read value: %x\n", value_in);
        }

        printf("-- %d mode_state %x status_word %x value_in %x\n",
                cycle_counter, mode_state, status_word, value_in);

        if (changed_mode && mode_state >= LM_MODE_SETPOINTS) {
            if (!writing) {
                buffer_idx--;

                if (buffer_response) {
                    printf("Buffer contents: \n");
                    for (int i=0; i <= buffer_idx; i++) {
                        printf("%x ", buffer[i]);
                    }
                    printf("\n");
                }

                if (mode_state == LM_MODE_SETPOINTS) {
                    memcpy(curve_info, buffer, sizeof(LmCurveInfo));
                    curve_info->dump();
                }
            }

            if (mode_state == LM_MODE_SETPOINTS + 1) {
                setIntegerParam(profileBuildStatus_, PROFILE_STATUS_SUCCESS);
                setIntegerParam(profileBuildState_, PROFILE_BUILD_DONE);

                ss.str("");
                ss.clear();
                if (writing) {
                    ss << "Wrote curve " << curve_id;
                } else {
                    ss << "Read curve " << curve_id;
                }

                const std::string status_message = ss.str();
                setStringParam(profileBuildMessage_, status_message.c_str());
                goto cleanup;
            }
        }

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
            }

            toggle = 1;
            buffer_idx = 0;
            break;

        case LM_MODE_CURVE_INFO:
            if (changed_mode) {
                if (writing) {
                    setStringParam(profileBuildMessage_, "Writing curve info");
                    bptr = (epicsInt32*)curve_info;
                    buffer_len = sizeof(LmCurveInfo) >> 2;
                    printf("Sending curve_info:\n");
                    curve_info->dump();
                } else {
                    setStringParam(profileBuildMessage_, "Reading curve info");
                }
            }

            toggle = 1 - toggle;
            expected_status = 0x402 + toggle;
            next_mode_status = 0x002 + toggle;
            buffer_response = !writing;
            break;

        case LM_MODE_SETPOINTS:
            if (changed_mode) {
                toggle = 0;
                buffer_idx = 0;

                if (writing) {
                    setStringParam(profileBuildMessage_, "Writing positions");
                    bptr = &buffer[0];
                    buffer_len = curve.setpoints.size();
                    for (i=0; i < buffer_len; i++) {
                        buffer[i] = (epicsInt32)(curve.setpoints[i] * LM_POSITION_SCALE);
                    }
                } else {
                    setStringParam(profileBuildMessage_, "Reading positions");
                }
            } else {
                toggle = 1 - toggle;
            }

            expected_status = 0x404 + toggle;
            next_mode_status = 0x004 + toggle;
            buffer_response = !writing;
            break;

        default:
            waiting_for_status = false;
            buffer_response = false;
            continue;
        }

        if (writing && (mode_state == LM_MODE_CURVE_INFO || mode_state == LM_MODE_SETPOINTS)) {
            cfgValueOut_->write(*bptr);
            buffer_len--;
            if (buffer_len < 0) {
                printf("  buffer empty (%d)?\n", buffer_len);
            } else {
                bptr++;
            }
        }

        waiting_for_status = true;

        if (mode_state > 0) {
            next_control_word = ((config_mode + mode_state - 1) << 8) |
                                 ((mode_state - 1) << 1) | toggle;
        }

        cfgControlWord_->write(next_control_word);

#if 1 // DEBUG
        printf("  toggle %x\n", toggle);
        printf("  buffer_response %x\n", buffer_response);
        if (buffer_response) {
            printf("  buffer_idx %x\n", buffer_idx);
        }
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
    }
    callParamCallbacks();
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
