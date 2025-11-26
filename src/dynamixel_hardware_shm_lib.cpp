#include "irsl_dynamixel_hardware_shm/DynamixelInterface.h"
#include "irsl_dynamixel_hardware_shm/common.h"

#include "irsl_dynamixel_hardware_shm/DynamixelShmLib.h"

#include <sstream>

namespace irsl_dynamixel {

class DynamixelShm::Impl
{
public:
    Impl () {
        di = std::shared_ptr<DynamixelInterface>(new DynamixelInterface());
    }
    std::shared_ptr<DynamixelInterface> di;
};

DynamixelShm::DynamixelShm () : period(0.01) {
    impl = new Impl();
}

void DynamixelShm::initialize (const std::string &yaml_file, int hash, int shm_key) {
    sm = std::shared_ptr<isc::ShmManager>(new isc::ShmManager());

    YAML::Node ynode;
    try {
        ynode = YAML::LoadFile(yaml_file);
    } catch (const std::exception &e) {
        std::ostringstream ss;
        ss << "parameter file [" << yaml_file << "] can not open / ";
        ss << e.what();
        throw std::runtime_error(ss.str());
    }

    if ( !ynode[_DX_HW_CONFIG_] ) {
        std::ostringstream ss;
        ss << "parameter file [" << yaml_file << "] does not contain keyword : " << _DX_HW_CONFIG_;
        throw std::runtime_error(ss.str());
    }
    YAML::Node hw_settings = ynode[_DX_HW_CONFIG_];

    irsl_common_utils::readValue(hw_settings, "period", period);

    bool ret = impl->di->initialize(hw_settings);
    if (!ret)  {
        throw std::runtime_error("Fail: di->initialize");
    }
    ss.hash    = hash;
    ss.shm_key = shm_key;
    ss.numJoints = impl->di->getNumberOfDynamixels();
    ss.numForceSensors = 0;
    ss.numImuSensors = 0;
    ss.jointType = isc::ShmSettings::JointType::PositionCommand | isc::ShmSettings::JointType::VelocityCommand;

    sm->setSettings(ss);

    bool res_op = sm->openSharedMemory(true);
    if (!res_op) {
        throw std::runtime_error("Fail: sm->openSharedMemory");
    }
    bool res_is = sm->isOpen();
    if (!res_is) {
        throw std::runtime_error("Fail: sm->isOpen");
    }
    //
    sm->resetFrame();
    //
    joint_num = impl->di->getNumberOfDynamixels();
    dyn_pos_cur.resize(joint_num);
    dyn_vel_cur.resize(joint_num);
    dyn_eff_cur.resize(joint_num);
    flt_pos_cur.resize(joint_num);
    flt_vel_cur.resize(joint_num);
    flt_eff_cur.resize(joint_num);

    flt_pos_cmd.resize(joint_num);
    dyn_pos_cmd.resize(joint_num);
    flt_vel_cmd.resize(joint_num);
    dyn_vel_cmd.resize(joint_num);
}

void DynamixelShm::finalize () {
    sm->closeSharedMemory();
}

void DynamixelShm::initializeCommand () {
    if (ss.jointType & isc::ShmSettings::JointType::PositionCommand) {
        sm->writePositionCommand(flt_pos_cur);
    }
    else if (ss.jointType & isc::ShmSettings::JointType::VelocityCommand) {
        sm->writeVelocityCommand(flt_vel_cur);
    }
}

void DynamixelShm::readDx () {
    // read current value from Dynamixel
    impl->di->getDynamixelCurrentStatus(dyn_pos_cur, dyn_vel_cur, dyn_eff_cur);

    // convert to floating value
    impl->di->convertDyn2FltPosition(dyn_pos_cur, flt_pos_cur);
    impl->di->convertDyn2FltVelocity(dyn_vel_cur, flt_vel_cur);
    impl->di->convertDyn2FltTorque  (dyn_eff_cur, flt_eff_cur);

    // write to sheread memory
    sm->writePositionCurrent(flt_pos_cur);
    sm->writeVelocityCurrent(flt_vel_cur);
    sm->writeTorqueCurrent  (flt_eff_cur);
}

void DynamixelShm::writeDx () {
    if (ss.jointType & isc::ShmSettings::JointType::PositionCommand) {
        // read command value from shered memory
        sm->readPositionCommand(flt_pos_cmd);
        // write comand value to Dynamixel
        impl->di->convertFlt2DynPosition(flt_pos_cmd, dyn_pos_cmd);
        impl->di->writePosition(dyn_pos_cmd);
    } else if (ss.jointType & isc::ShmSettings::JointType::VelocityCommand) {
        // read command value from shered memory
        sm->readVelocityCommand(flt_vel_cmd);
        // write comand value to Dynamixel
        impl->di->convertFlt2DynVelocity(flt_vel_cmd, dyn_vel_cmd);
        impl->di->writeVelocity(dyn_vel_cmd);
    }
}

} // namespace
