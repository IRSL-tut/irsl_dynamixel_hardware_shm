#pragma once

#include "irsl/shm_controller.h"
#include "irsl/simple_yaml_parser.hpp"

namespace isc = irsl_shm_controller;

typedef std::vector<isc::irsl_float_type> floatvec;
typedef std::vector<int32_t>              int32vec;

namespace irsl_dynamixel {
///
class DynamixelShm
{
public:
    DynamixelShm();

public:
    void initialize(const std::string &yaml_file, int hash, int shm_key);
    void finalize();
    void initializeCommand ();
    void readDx ();
    void writeDx ();

    double getPeriod() { return period; }
    void incrementFrame() { sm->incrementFrame(); }

    std::shared_ptr<isc::ShmManager> shm_manager() { return sm; }
protected:
    std::shared_ptr<isc::ShmManager> sm;
    isc::ShmSettings ss;

    double period;

    size_t joint_num;
    int32vec dyn_pos_cur;
    int32vec dyn_vel_cur;
    int32vec dyn_eff_cur;
    floatvec flt_pos_cur;
    floatvec flt_vel_cur;
    floatvec flt_eff_cur;

    floatvec flt_pos_cmd;
    int32vec dyn_pos_cmd;
    floatvec flt_vel_cmd;
    int32vec dyn_vel_cmd;

private:
    class Impl;
    Impl *impl;
};

typedef std::shared_ptr<DynamixelShm> DynamixelShmPtr;

};
