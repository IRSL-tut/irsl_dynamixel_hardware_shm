#include "irsl_dynamixel_hardware_shm/DynamixelInterface.h"

DynamixelInterface::DynamixelInterface()
    : dxl_wb_(std::make_unique<DynamixelWorkbench>())
{
}

DynamixelInterface::~DynamixelInterface()
{
}

bool DynamixelInterface::initialize(YAML::Node &settings)
{
    // Initialize the interface by setting parameters from settings
    bool result = parseParamsFromYAML(settings);
    if (!result)
    {
        std::cerr << "Error: unable to set parameters" << std::endl;
        return false; // Return immediately on failure
    }

    // Discover connected Dynamixels
    result = discoverConnectedDynamixels();
    if (!result)
    {
        std::cerr << "Error: unable to discover connected Dynamixels" << std::endl;
        return false; // Return immediately on failure
    }

    // Initialize settings for the Dynamixels
    result = writeInitialSettings();
    if (!result)
    {
        std::cerr << "Error: unable to initialize Dynamixel settings" << std::endl;
        return false; // Return immediately on failure
    }

    // Initialize control items (e.g. motor controllers, sensors)
    result = initializeControlItems();
    if (!result)
    {
        std::cerr << "Error: unable to initialize control items" << std::endl;
        return false; // Return immediately on failure
    }

    // Initialize SDK handlers for Dynamixel communication
    result = initSDKHandlers();
    if (!result)
    {
        std::cerr << "Error: unable to initialize SDK handlers" << std::endl;
        return false; // Return immediately on failure
    }
    return true;
}

bool DynamixelInterface::parseParamsFromYAML(YAML::Node &settings)
{

    auto const port_name = settings["port_name"].as<std::string>();
    auto const baud_rate = settings["baud_rate"].as<int32_t>();

    dx_info.clear();
    size_t index = 0;

    for (const auto &joint : settings["joint"])
    {
        DynamixelInfo info;
        info.comm_group_name = "default";
        for (auto joint_data = joint.begin(); joint_data != joint.end(); ++joint_data)
        {
            std::string key = joint_data->first.as<std::string>();
            if (key == "ID")
            {
                info.id = (int8_t)(joint_data->second.as<int32_t>());
            }
            else if (key == "CommunicationGroupName")
            {
                info.comm_group_name = joint_data->second.as<std::string>();
            }
            else if (key == "DynamixelSettings")
            {
                auto const dx_settings = joint_data->second;
                for (const auto &dx_setting : dx_settings)
                {
                    ItemValue item_value{dx_setting.first.as<std::string>(), dx_setting.second.as<int32_t>()};
                    info.dxl_setting.push_back(item_value);
#ifdef DEBUG
                    std::cout << "key: " << item_value.item_name << ", value: " << item_value.value << std::endl;
#endif
                }
            }
        }
        comm_group_names.insert(info.comm_group_name);
        dx_info.push_back(info);
        dx_info_index_map[info.id] = index++;
    }

    comm_group_id_map.clear();
    for (const auto &dx_i : dx_info)
    {
        comm_group_id_map[dx_i.comm_group_name].push_back(dx_i.id);
    }

    return initializeDynamixelWorkbench(port_name, baud_rate);
}

bool DynamixelInterface::initializeDynamixelWorkbench(const std::string &port_name, int32_t baud_rate)
{
    // Initialize a flag to track the result of the operation
    bool result = false;

    // Get a pointer to the log message in case initialization fails
    const char *log;

    // Attempt to initialize Dynamixel Workbench using the provided port name and baud rate
    result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);

    // If initialization fails, print an error message with the log details
    if (result == false)
    {
        std::cerr << "Error initializing Dynamixel Workbench: " << log << std::endl;
    }

    return result; // Return true on success, false on failure
}

bool DynamixelInterface::writeInitialSettings()
{
    const char *log;

    for (const auto &info : dx_info)
    {
        // Get the current ID
        uint8_t id = info.id;
        // torque off
        dxl_wb_->torqueOff(id, &log);
        for (const auto &setting : info.dxl_setting)
        {
            bool result = dxl_wb_->itemWrite(id, setting.item_name.c_str(), setting.value, &log);
            if (result == false)
            {
                std::cerr << log << std::endl;
                std::cerr << "Failed to write value[" << setting.value << "] on items[" << setting.item_name << "] to Dynamixel[ ID : " << id << "]" << std::endl;
                return false; // Return immediately if any setting fails
            }
        }
        // trque on
        dxl_wb_->torqueOn(id, &log);
    }

    return true;
}

bool DynamixelInterface::discoverConnectedDynamixels(void)
{
    bool result = false;
    const char *log;

    for (auto const &dxl : dx_info)
    {
        uint16_t model_number = 0;
        uint8_t id = (uint8_t)dxl.id;
        result = dxl_wb_->ping(id, &model_number, &log);
        if (result == false)
        {
            std::cerr << log << std::endl;
            std::cerr << "Can't find Dynamixel ID " << (int32_t)id << std::endl;
            return result;
        }
        else
        {
            std::cout << "ID : " << (int32_t)dxl.id << ", Model Number : " << model_number << std::endl;
        }
    }

    return result;
}

bool DynamixelInterface::initializeControlItems(void)
{
    if (dx_info.empty())
    {
        std::cerr << "No Dynamixel info available." << std::endl;
        return false;
    }

    const char *log = nullptr;

    uint8_t sample_id = dx_info.front().id;

    std::vector<std::string> keys = {
        "Goal_Position", "Goal_Velocity",
        "Present_Position", "Present_Velocity", "Present_Current"};

    for (const auto &key : keys)
    {
        const ControlItem *item = dxl_wb_->getItemInfo(sample_id, key.c_str());

        if (item == nullptr)
        {
            if (key == "Goal_Velocity")
                item = dxl_wb_->getItemInfo(sample_id, "Moving_Speed");
            else if (key == "Present_Velocity")
                item = dxl_wb_->getItemInfo(sample_id, "Present_Speed");
            else if (key == "Present_Current")
                item = dxl_wb_->getItemInfo(sample_id, "Present_Load");
        }

        if (item == nullptr)
        {
            std::cerr << "Failed to get ControlItem: " << key << std::endl;
            return false;
        }

        control_items_[key] = item;
    }

    return true;
}

bool DynamixelInterface::initSDKHandlers(void)
{
    bool result = false;
    const char *log = NULL;

    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
    if (result == false)
    {
        std::cerr << log << std::endl;
        return result;
    }
    else
    {
        std::cout << log << std::endl;
    }

    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
    if (result == false)
    {
        std::cerr << log << std::endl;
        return result;
    }
    else
    {
        std::cout << log << std::endl;
    }

    if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
        uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

        /*
          As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
        */
        // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
        uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length + 2;

        result = dxl_wb_->addSyncReadHandler(start_address,
                                             read_length,
                                             &log);
        if (result == false)
        {
            std::cerr << log << std::endl;
            return result;
        }
    }

    return result;
}

size_t DynamixelInterface::getNumberOfDynamixels()
{
    return dx_info.size();
}

void DynamixelInterface::convertDyn2FltPosition(
    const std::vector<int32_t> &pos_vec,
    std::vector<irsl_shm_controller::irsl_float_type> &pos_float_vec)
{
    if (pos_vec.size() != pos_float_vec.size())
    {
        pos_float_vec.resize(pos_vec.size());
    }
    for (size_t i = 0; i < dx_info.size(); i++)
    {
        irsl_shm_controller::irsl_float_type angle = dxl_wb_->convertValue2Radian(dx_info[i].id, pos_vec[i]);
        pos_float_vec[i] = angle;
    }
}

void DynamixelInterface::convertDyn2FltVelocity(
    const std::vector<int32_t> &vel_vec,
    std::vector<irsl_shm_controller::irsl_float_type> &vel_float_vec)
{
    if (vel_vec.size() != vel_float_vec.size())
    {
        vel_float_vec.resize(vel_vec.size());
    }
    for (size_t i = 0; i < dx_info.size(); i++)
    {
        irsl_shm_controller::irsl_float_type vel = dxl_wb_->convertValue2Velocity(dx_info[i].id, vel_vec[i]);
        vel_float_vec[i] = vel;
    }
}

void DynamixelInterface::convertDyn2FltCurrent(
    const std::vector<int32_t> &cur_vec,
    std::vector<irsl_shm_controller::irsl_float_type> &cur_float_vec)
{
    if (cur_vec.size() != cur_float_vec.size())
    {
        cur_float_vec.resize(cur_vec.size());
    }
    for (size_t i = 0; i < dx_info.size(); i++)
    {
        irsl_shm_controller::irsl_float_type cur = dxl_wb_->convertValue2Current(dx_info[i].id, cur_vec[i]);
        cur_float_vec[i] = cur;
    }
}

void DynamixelInterface::convertDyn2FltTorque(
    const std::vector<int32_t> &cur_vec,
    std::vector<irsl_shm_controller::irsl_float_type> &torque_float_vec)
{
    if (cur_vec.size() != torque_float_vec.size())
    {
        torque_float_vec.resize(cur_vec.size());
    }
    for (size_t i = 0; i < dx_info.size(); i++)
    {
        irsl_shm_controller::irsl_float_type tor = dxl_wb_->convertValue2Current(dx_info[i].id, cur_vec[i]);
        torque_float_vec[i] = tor;
    }
}

void DynamixelInterface::convertFlt2DynPosition(
    const std::vector<irsl_shm_controller::irsl_float_type> &pos_float_vec,
    std::vector<int32_t> &dynamixel_position)
{
    size_t id_vec_size = dx_info.size();
    dynamixel_position.resize(id_vec_size);
    for (size_t i = 0; i < id_vec_size; i++)
    {
        dynamixel_position[i] = dxl_wb_->convertRadian2Value(dx_info[i].id, pos_float_vec[i]);
    }
}

bool DynamixelInterface::writeBySyncHandler(
    uint8_t handler_index,
    const std::vector<int32_t> &value_vector)
{
    bool result = false;
    const char *log = nullptr;

    std::vector<int32_t> value_tmp;

    for (const auto &group_pair : comm_group_id_map)
    {
        const std::vector<uint8_t> &comm_group_id = group_pair.second;

        value_tmp.clear();
        for (uint8_t id : comm_group_id)
        {
            auto it = dx_info_index_map.find(id);
            if (it != dx_info_index_map.end())
            {
                value_tmp.push_back(value_vector[it->second]);
            }
            else
            {
                std::cerr << "ID not found in dx_info_index_map: " << static_cast<int>(id) << std::endl;
                return false;
            }
        }

        result = dxl_wb_->syncWrite(
            handler_index,
            const_cast<uint8_t *>(comm_group_id.data()), comm_group_id.size(),
            value_tmp.data(), 1, &log);
        if (!result)
        {
            std::cerr << log << std::endl;
            return false;
        }
    }

    return true;
}


bool DynamixelInterface::writePosition(const std::vector<int32_t> &dynamixel_position)
{
    return writeBySyncHandler(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, dynamixel_position);
}

bool DynamixelInterface::writeVelocity(const std::vector<int32_t> &dynamixel_velocity)
{
    return writeBySyncHandler(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, dynamixel_velocity);
}


// bool DynamixelInterface::writePosition(
//     const std::vector<int32_t> &dynamixel_position)
// {
//     bool result = false;
//     const char *log = NULL;

//     std::vector<int32_t> dynamixel_position_tmp;
//     for (const auto &group_pair : comm_group_id_map)
//     {
//         const std::string &group_name = group_pair.first;
//         const std::vector<uint8_t> &comm_group_id = group_pair.second;

//         dynamixel_position_tmp.clear();
//         for (uint8_t id : comm_group_id)
//         {
//             auto it = dx_info_index_map.find(id);
//             if (it != dx_info_index_map.end())
//             {
//                 dynamixel_position_tmp.push_back(dynamixel_position[it->second]);
//             }
//             else
//             {
//                 std::cerr << "ID not found in dx_info_index_map: " << static_cast<int>(id) << std::endl;
//                 return false;
//             }
//         }

//         result = dxl_wb_->syncWrite(
//             SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
//             const_cast<uint8_t *>(comm_group_id.data()), comm_group_id.size(),
//             dynamixel_position_tmp.data(), 1, &log);
//         if (result == false)
//         {
//             std::cerr << log << std::endl;
//             return result;
//         }
//     }
//     return result;
// }

// bool DynamixelInterface::writeVelocity(
//     const std::vector<int32_t> &dynamixel_velocity)
// {
//     bool result = false;
//     const char *log = NULL;

//     std::vector<int32_t> dynamixel_velocity_tmp;
//     for (const auto &group_pair : comm_group_id_map)
//     {
//         const std::string &group_name = group_pair.first;
//         const std::vector<uint8_t> &comm_group_id = group_pair.second;

//         dynamixel_velocity_tmp.clear();
//         for (uint8_t id : comm_group_id)
//         {
//             auto it = dx_info_index_map.find(id);
//             if (it != dx_info_index_map.end())
//             {
//                 dynamixel_velocity_tmp.push_back(dynamixel_velocity[it->second]);
//             }
//             else
//             {
//                 std::cerr << "ID not found in dx_info_index_map: " << static_cast<int>(id) << std::endl;
//                 return false;
//             }
//         }

//         result = dxl_wb_->syncWrite(
//             SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY,
//             const_cast<uint8_t *>(comm_group_id.data()), comm_group_id.size(),
//             dynamixel_velocity_tmp.data(), 1, &log);
//         if (result == false)
//         {
//             std::cerr << log << std::endl;
//             return result;
//         }
//     }
//     return result;
// }


void DynamixelInterface::convertFlt2DynVelocity(
    const std::vector<irsl_shm_controller::irsl_float_type> &vel_float_vec,
    std::vector<int32_t> &dynamixel_velocity)
{
    size_t id_vec_size = dx_info.size();
    dynamixel_velocity.resize(id_vec_size);
    for (size_t i = 0; i < id_vec_size; i++)
    {
        dynamixel_velocity[i] = dxl_wb_->convertVelocity2Value(dx_info[i].id, vel_float_vec[i]);
    }
}


void DynamixelInterface::getDynamixelCurrentStatus(
    std::vector<int32_t> &pos_vec,
    std::vector<int32_t> &vel_vec,
    std::vector<int32_t> &cur_vec)
{
    bool result = false;
    const char *log = NULL;

    size_t id_vec_size = dx_info.size();

    if (pos_vec.size() != id_vec_size)
    {
        pos_vec.resize(id_vec_size);
    }
    if (vel_vec.size() != id_vec_size)
    {
        vel_vec.resize(id_vec_size);
    }
    if (cur_vec.size() != id_vec_size)
    {
        cur_vec.resize(id_vec_size);
    }

    for (const auto &group_pair : comm_group_id_map)
    {
        const std::string &group_name = group_pair.first;
        const std::vector<uint8_t> &comm_group_id = group_pair.second;

        size_t comm_group_id_size = comm_group_id.size();
        std::vector<int32_t> pos_vec_tmp(comm_group_id.size());
        std::vector<int32_t> vel_vec_tmp(comm_group_id.size());
        std::vector<int32_t> cur_vec_tmp(comm_group_id.size());

        result = dxl_wb_->syncRead(
            SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
            const_cast<uint8_t *>(comm_group_id.data()), comm_group_id_size,
            &log);
        if (!result)
        {
            std::cerr << "syncRead failed " << log << std::endl;
            return;
        }

        result = dxl_wb_->getSyncReadData(
            SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
            const_cast<uint8_t *>(comm_group_id.data()), comm_group_id_size,
            control_items_["Present_Position"]->address,
            control_items_["Present_Position"]->data_length,
            pos_vec_tmp.data(),
            &log);
        if (!result)
        {
            std::cerr << "getSyncReadData position failed " << log << std::endl;
        }

        result = dxl_wb_->getSyncReadData(
            SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
            const_cast<uint8_t *>(comm_group_id.data()), comm_group_id_size,
            control_items_["Present_Velocity"]->address,
            control_items_["Present_Velocity"]->data_length,
            vel_vec_tmp.data(),
            &log);
        if (result == false)
        {
            std::cerr << "getSyncReadData velocity failed " << log << std::endl;
        }

        result = dxl_wb_->getSyncReadData(
            SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
            const_cast<uint8_t *>(comm_group_id.data()), comm_group_id_size,
            control_items_["Present_Current"]->address,
            control_items_["Present_Current"]->data_length,
            cur_vec_tmp.data(),
            &log);
        if (result == false)
        {
            std::cerr << "getSyncReadData current failed " << log << std::endl;
        }

        for (size_t j = 0; j < comm_group_id_size; ++j)
        {
            uint8_t id = comm_group_id[j];
            auto it = dx_info_index_map.find(id);
            if (it != dx_info_index_map.end())
            {
                size_t idx = it->second;
                pos_vec[idx] = pos_vec_tmp[j];
                vel_vec[idx] = vel_vec_tmp[j];
                cur_vec[idx] = cur_vec_tmp[j];
            }
            else
            {
                std::cerr << "Unknown ID in dx_info_index_map: " << static_cast<int>(id) << std::endl;
            }
        }
    }
}
