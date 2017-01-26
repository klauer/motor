#ifndef _H_LINMOT_CURVE
#define _H_LINMOT_CURVE


#include <cstdio>
#include <cstddef>
#include <cassert>
#include <cstring>
#include <string>
#include <vector>
#include <stdint.h>


const double LM_MAX_CURVE_TIME = 42949.0; // 2**32/1e5;


enum LmObjectType {
    LM_OT_POSITION_VS_TIME=0x0003,
    LM_OT_ENCODER_VS_POSITION=0x0103,
};


enum LmUUID {
    LM_UUID_POSITION=0x0005,
    LM_UUID_TIME=0x001A,
    LM_UUID_ENCODER=0x001B,
};


enum LmConfigModuleCmd {
    LM_CMD_NOOP=0x0000,
    LM_CMD_PREPARE_CONFIG_MODULE=0x000F,
    LM_CMD_SAVE_TO_FLASH=0x4001,
    LM_CMD_DELETE_ALL_CURVES=0x4101,
    LM_CMD_RESTART_DRIVE=0x3000,
    LM_CMD_STOP_MC=0x3500,
    LM_CMD_START_MC=0x3600
};


enum LmCommandHeader {
    LM_HDR_TIME_CURVE_SCALED=0x0440,
    LM_HDR_TIME_CURVE_TOTAL=0x0450,
    LM_HDR_INVALIDATE_CURVE=0x0500
};


typedef struct __attribute__((packed)) {
    uint16_t curve_id;
    int32_t curve_offset;
    int32_t time;
    int16_t amplitude_scale;
} LmTimeCurveTotal;


const uint16_t LM_CI_DATA_OFFSET = 70;
const uint16_t LM_CI_STRING_LENGTH = 22;
const int LM_CI_NUM_WIZARD_PARAMS = 7;
const double LM_CI_XLENGTH_SCALE = 1.0e5;
const double LM_POSITION_SCALE = 1.0e4;


const uint32_t LM_CONFIG_INIT = 0x0F;
const uint32_t LM_CONFIG_CURVE_WRITE = 0x50;
const uint32_t LM_CONFIG_CURVE_MODIFY = 0x53;
const uint32_t LM_CONFIG_CURVE_READ = 0x60;

const uint32_t LM_MODE_INIT = 0;
const uint32_t LM_MODE_SEND_COMMAND = 1;
const uint32_t LM_MODE_CURVE_INFO = 2;
const uint32_t LM_MODE_SETPOINTS = 3;


struct __attribute__((packed)) LmCurveInfo {
    uint16_t data_offset;
    uint16_t object_type;
    uint16_t num_setpoints;
    uint16_t data_type_size;
    unsigned char name[LM_CI_STRING_LENGTH];
    uint16_t curve_id;
    uint32_t x_length;
    uint16_t x_dim_uuid;
    uint16_t y_dim_uuid;
    uint16_t wizard_type;
    uint32_t wizard_params[LM_CI_NUM_WIZARD_PARAMS];

    // pad to 72 bytes for easy sending over the wire
    uint16_t __padding__;

    LmCurveInfo() : data_offset(LM_CI_DATA_OFFSET), data_type_size(4),
                    wizard_type(0), __padding__(0) {
        for (int i=0; i < LM_CI_NUM_WIZARD_PARAMS; i++) {
            wizard_params[i] = 0;
        }
    }

    uint32_t packed_block_size() {
        return (((num_setpoints << 18) |
                 (sizeof(LmCurveInfo) - sizeof(__padding__))) & 0xffffffff);
    }

    bool set_name(const char* new_name);
    bool set_name(std::string& new_name) {
        return set_name(new_name.c_str());
    }

    void dump(FILE *fp=stdout);
};

typedef struct LmCurveInfo LmCurveInfo;

bool new_position_time_curve(struct LmCurveInfo &ci, double time_sec,
                             uint16_t num_setpoints, std::string& name,
                             uint16_t curve_id);

class LmCurve {
public:
    std::string name;
    uint16_t curve_id;
    std::vector<double> setpoints;

    LmCurve()
    {
    }

    LmCurve(std::string& _name, uint16_t _curve_id) :
        name(_name), curve_id(_curve_id)
    {
    }

};

class LmPositionTimeCurve : public LmCurve {
public:
    double dt;

    LmPositionTimeCurve() : LmCurve()
    {
    }

    LmPositionTimeCurve(std::string& _name, uint16_t _curve_id, double _dt) :
        LmCurve(_name, _curve_id), dt(_dt)
    {
    }

    struct LmCurveInfo *get_curve_info();
    void set_curve_info(LmCurveInfo &ci);

    const double get_total_seconds() {
        int num_setpoints = setpoints.size();
        return num_setpoints * dt;
    }

};



#endif
