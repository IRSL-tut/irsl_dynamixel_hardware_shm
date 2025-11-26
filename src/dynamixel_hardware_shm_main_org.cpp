#include <iostream>

#include "irsl/shm_controller.h"
#include "irsl/realtime_task.h"
#include "irsl/simple_yaml_parser.hpp"
#include "irsl/signal_handler.h"
#include "irsl/thirdparty/CLI11.hpp"

namespace icu = irsl_common_utils;
namespace isc = irsl_shm_controller;
namespace irt = irsl_realtime_task;

//// TODO: move to common space
#include <unordered_map>
static const std::unordered_map<std::string, int> jointTypeMap = {
    {"PositionCommand",  isc::ShmSettings::JointType::PositionCommand},
    {"PositionGains",    isc::ShmSettings::JointType::PositionGains},
    {"VelocityCommand",  isc::ShmSettings::JointType::VelocityCommand},
    {"VelocityGains",    isc::ShmSettings::JointType::VelocityGains},
    {"TorqueCommand",    isc::ShmSettings::JointType::TorqueCommand},
    {"TorqueGains",      isc::ShmSettings::JointType::TorqueGains},
    {"MotorTemperature", isc::ShmSettings::JointType::MotorTemperature},
    {"MotorCurrent",     isc::ShmSettings::JointType::MotorCurrent},
};

typedef std::vector<isc::irsl_float_type> floatvec;
typedef std::vector<int32_t>              int32vec;

#include <stdlib.h>
class OptParse : public CLI::App
{
private:
    uint64_t _hash;
    uint32_t _key;
public:
    OptParse(const std::string &name) : CLI::App(name)
    {
        add_option("--hash", _hash, "")->default_val("8888");
        add_option("--shm_key", _key, "")->default_val("8888");
    }
    uint64_t getHash() { return _hash; }
    uint32_t getShmKey() { return _key; }
    void opt_parse(int argc, char **argv)
    {
        try {
            this->parse(argc, argv);
        } catch(const CLI::ParseError &e) {
            ::exit( this->exit(e) );
        }
    }
};
////

#include "irsl_dynamixel_hardware_shm/DynamixelInterface.h"
#include "irsl_dynamixel_hardware_shm/common.h"

void status_print(
    const floatvec& cur_pos_flt,
    const floatvec& cur_vel_flt)
{
    size_t n = std::min(cur_pos_flt.size(), cur_vel_flt.size());

    for (size_t i = 0; i < n; i++)
    {
        std::cerr << i << " "
                  << cur_pos_flt[i] << " "
                  << cur_vel_flt[i] << std::endl;
    }
}

bool _running_ = true;
void _sighandle(int sig)
{
    _running_ = false;
}
int main(int argc, char **argv)
{
    setSignalHandler(SIGINT, _sighandle);

    std::string fname;
    std::vector<std::string> joint_types = {"PositionCommand", "VelocityCommand" }; // "PositionGains", "VelocityGains"
    bool verbose = false;

    OptParse op("Dynamixel controller");
    op.add_option("--config", fname, "name of input file(.yaml)")->default_val("config.yaml");
    op.add_flag("-v,--verbose", verbose, "verbose message");
    op.opt_parse(argc, argv);


    YAML::Node n;
    try
    {
        // check fname
        n = YAML::LoadFile(fname);
    }
    catch (const std::exception &)
    {
        std::cerr << "parameter file [" << fname << "] can not open" << std::endl;
        return false;
    }
    YAML::Node hardware_settings = n[_DX_HW_CONFIG_];

    irsl_dynamixel::DynamixelInterface di;
    bool ret;
    ret = di.initialize(hardware_settings);
    if (!ret)
    {
        return -1;
    }

    isc::ShmSettings ss;
    ss.hash    = op.getHash();
    ss.shm_key = op.getShmKey();

    ss.numJoints = di.getNumberOfDynamixels();
    ss.numForceSensors = 0;
    ss.numImuSensors = 0;

    //// TODO shm_libs
    ss.jointType = 0;
    for (const auto &jtype : joint_types)
    {
        auto it = jointTypeMap.find(jtype);
        if (it != jointTypeMap.end())
        {
            ss.jointType |= it->second;
        }
    }
    std::cerr << "jointType : " << ss.jointType << std::endl;

    isc::ShmManager sm(ss);
    bool res;
    res = sm.openSharedMemory(true);
    std::cerr << "open: " << res << std::endl;
    if (!res)
    {
        return -1;
    }

    res = sm.isOpen();
    std::cerr << "isOpen: " << res << std::endl;
    if (!res)
    {
        return -1;
    }

    sm.resetFrame();
    double period_sec = hardware_settings["period"].as<double>(); // TODO shm_libs
    unsigned long interval_ns = (unsigned long)(period_sec * 1000000000);

    size_t joint_num = di.getNumberOfDynamixels();
    int32vec dyn_pos_cur(joint_num);
    int32vec dyn_vel_cur(joint_num);
    int32vec dyn_eff_cur(joint_num);
    floatvec flt_pos_cur(joint_num);
    floatvec flt_vel_cur(joint_num);
    floatvec flt_eff_cur(joint_num);

    floatvec flt_pos_cmd(joint_num);
    int32vec dyn_pos_cmd(joint_num);

    floatvec flt_vel_cmd(joint_num);
    int32vec dyn_vel_cmd(joint_num);

    di.getDynamixelCurrentStatus(dyn_pos_cur, dyn_vel_cur, dyn_eff_cur);

    di.convertDyn2FltPosition(dyn_pos_cur, flt_pos_cur);
    di.convertDyn2FltVelocity(dyn_vel_cur, flt_vel_cur);
    di.convertDyn2FltTorque(dyn_eff_cur, flt_eff_cur);

    sm.writePositionCurrent(flt_pos_cur);
    sm.writeVelocityCurrent(flt_vel_cur);
    sm.writeTorqueCurrent(flt_eff_cur);

    if (ss.jointType & isc::ShmSettings::JointType::PositionCommand)
    {
        sm.writePositionCommand(flt_pos_cur);
    }
    else if (ss.jointType & isc::ShmSettings::JointType::VelocityCommand)
    {
        sm.writeVelocityCommand(flt_vel_cur);
    }

    // sm.writeTorqueCommand(flt_eff_cur);
    if (verbose)
    {
        status_print(flt_pos_cur, flt_vel_cur);
    }

    irt::RealtimeContext rt(interval_ns);
    int cntr = 0;
    rt.start();
    while (_running_)
    {
        // read current value from Dynamixel
        di.getDynamixelCurrentStatus(dyn_pos_cur, dyn_vel_cur, dyn_eff_cur);

        // convert to floating value
        di.convertDyn2FltPosition(dyn_pos_cur, flt_pos_cur);
        di.convertDyn2FltVelocity(dyn_vel_cur, flt_vel_cur);
        di.convertDyn2FltTorque(dyn_eff_cur, flt_eff_cur);

        // write to sheread memory
        sm.writePositionCurrent(flt_pos_cur);
        sm.writeVelocityCurrent(flt_vel_cur);
        sm.writeTorqueCurrent(flt_eff_cur);

        if (ss.jointType & isc::ShmSettings::JointType::PositionCommand)
        {
            // read command value from shered memory
            sm.readPositionCommand(flt_pos_cmd);
            // write comand value to Dynamixel
            di.convertFlt2DynPosition(flt_pos_cmd, dyn_pos_cmd);
            di.writePosition(dyn_pos_cmd);
        }
        else if (ss.jointType & isc::ShmSettings::JointType::VelocityCommand)
        {
            // read command value from shered memory
            sm.readVelocityCommand(flt_vel_cmd);
            // write comand value to Dynamixel
            di.convertFlt2DynVelocity(flt_vel_cmd, dyn_vel_cmd);
            di.writeVelocity(dyn_vel_cmd);
        }

        if (verbose)
        {
            status_print(flt_pos_cur, flt_vel_cur);
            std::cerr << "--------------------" << std::endl;
        }

        cntr++;

        sm.incrementFrame();
        if (cntr > 100)
        {
            std::cerr << "max: " << rt.getMaxInterval() << std::endl;
            rt.reset();
            cntr = 0;
        }
        rt.waitNextFrame();
    }
    // polling
    sm.closeSharedMemory();
    return 0;
}
