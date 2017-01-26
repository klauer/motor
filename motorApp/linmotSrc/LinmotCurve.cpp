#include "LinmotCurve.h"


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
