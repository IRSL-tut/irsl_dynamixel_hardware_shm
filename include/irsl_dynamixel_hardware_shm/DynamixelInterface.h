#pragma once

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <iostream>
#include "irsl/shm_controller.h"

#include <yaml-cpp/yaml.h>
#include <memory>
#include <unordered_map>

// SYNC_WRITE_HANDLER
static constexpr uint8_t SYNC_WRITE_HANDLER_FOR_GOAL_POSITION = 0;
static constexpr uint8_t SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY = 1;

// SYNC_READ_HANDLER(Only for Protocol 2.0)
static constexpr uint8_t SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT = 0;

/**
 * @brief Dynamixel item struct to hold control items and their values.
 *
 * This struct represents a single dynamixel item, with a name and a value.
 */
struct ItemValue
{
    std::string item_name; ///< Name of the dynamixel item.
    int32_t value;         ///< Value of the dynamixel item.
};

/**
 * @brief A struct to hold information about individual Dynamixels.
 *
 * This struct represents a single Dynamixel, with attributes for its ID,
 * baud rate, protocol version, and control modes.
 */
struct DynamixelInfo
{
    // std::string name;
    uint8_t id;                         ///< id
    std::string comm_group_name;        ///< communication group name
    std::vector<ItemValue> dxl_setting; ///< Dynamixel settings
};

class DynamixelInterface
{
public:
    /**
     * @brief Constructor for the interface class.
     *
     * This constructor initializes the dynamixel interface with default values.
     */
    DynamixelInterface();

    /**
     * @brief Destructor for the interface class.
     *
     * This destructor frees any dynamically allocated resources held by the
     * dynamixel interface.
     */
    ~DynamixelInterface();

    /**
     * @brief Initializes the specified Dynamixels with their initial settings.
     *
     * Writes the initial settings to the Dynamixels.
     *
     * @return true All settings were written successfully.
     * @return false Some settings failed to write.
     */
    bool writeInitialSettings();

    /**
     * @brief Reads YAML data and initializes Dynamixel settings and initialization.
     *
     * @param n YAML node data
     * @return true Successful
     * @return false Failed (file read or parsing error)
     */
    bool initialize(YAML::Node &settings);

    /**
     * @brief Parses parameters from a YAML node and initializes the Dynamixel interface.
     *
     * Reads the specified YAML node to extract initialization settings for the dynamixel interface.
     *
     * @param settings YAML node containing initialization settings
     * @return true Successful
     * @return false Failed (parse error or invalid data)
     */
    bool parseParamsFromYAML(YAML::Node &settings);

    /**
     * @brief Initializes the Dynamixel Workbench.
     *
     * Initializes the Dynamixel Workbench by setting up a connection to the
     * specified port and baud rate.
     *
     * @param port_name The name of the device (e.g. "/dev/ttyUSB0")
     * @param baud_rate The communication speed (e.g. 1000000)
     * @return true Initialization was successful
     * @return false Initialization failed
     */
    bool initializeDynamixelWorkbench(const std::string &port_name, int32_t baud_rate);

    /**
     * @brief Retrieves information about the connected Dynamixels and outputs their model details.
     *
     * @return true All Dynamixels were successfully queried.
     * @return false Some queries failed
     */
    bool discoverConnectedDynamixels();

    /**
     * @brief Retrieves necessary control items for operation and records them.
     *
     * This function retrieves the required control items, such as position and velocity,
     * from the Dynamixel settings and records them in the interface's internal state.
     *
     * @return true Successful
     * @return false Unable to retrieve necessary control items
     */
    bool initializeControlItems();

    /**
     * @brief Transfers SDK read/write handlers into the controller.
     *
     * This function registers the required SDK SyncRead/SyncWrite handlers for operation.
     *
     * @return true Successful
     * @return false Failed to register handlers
     */
    bool initSDKHandlers();

    /**
     * @brief Retrieves the number of Dynamixels registered in the interface.
     *
     * This function returns the total count of registered Dynamixels.
     *
     * @return size_t The total count of registered Dynamixels.
     */
    size_t getNumberOfDynamixels();

    /**
     * @brief Current status of the Dynamixel is retrieved.
     *
     * Retrieves the current position, velocity, and current data from the
     * Dynamixel.
     *
     * @param pos_vec Output: Angle data (raw value)
     * @param vel_vec Output: Velocity data (raw value)
     * @param cur_vec Output: Current data (raw value)
     */
    void getDynamixelCurrentStatus(
        std::vector<int32_t> &pos_vec,
        std::vector<int32_t> &vel_vec,
        std::vector<int32_t> &cur_vec);

    /**
     * @brief Convert angle data (raw value) to radians.
     *
     * @param pos_vec Input: Angle data (Dynamixel raw value)
     * @param pos_float_vec Output: Angle in radians
     */
    void convertPosition(
        const std::vector<int32_t> &pos_vec,
        std::vector<irsl_shm_controller::irsl_float_type> &pos_float_vec);
    /**
     * @brief Speed data (raw value) conversion to user-defined unit.
     *
     * @param vel_vec Input speed values (Dynamixel raw value)
     * @param vel_float_vec Output speed values in user-defined units
     */
    void convertVelocity(
        const std::vector<int32_t> &vel_vec,
        std::vector<irsl_shm_controller::irsl_float_type> &vel_float_vec);

    /**
     * @brief Current data (raw value) conversion to user-defined unit.
     *
     * @param cur_vec Input current values (Dynamixel raw value)
     * @param cur_float_vec Output current values in user-defined units
     */
    void convertCurrent(
        const std::vector<int32_t> &cur_vec,
        std::vector<irsl_shm_controller::irsl_float_type> &cur_float_vec);

    /**
     * @brief Torque (current) data is converted.
     *
     * @note Currently performs the same conversion as convertCurrent
     *
     * @param cur_vec Input current values (Dynamixel raw value)
     * @param torque_float_vec Output torque values (unit undefined)
     */
    void convertTorque(
        const std::vector<int32_t> &cur_vec,
        std::vector<irsl_shm_controller::irsl_float_type> &torque_float_vec);

    /**
     * @brief Convert position command from radians to raw value.
     *
     * This function converts the position command (in radians) into the raw value
     * expected by the Dynamixel.
     *
     * @param pos_float_vec Input: Position command in radians
     * @param dynamixel_position Output: Raw value for the Dynamixel
     */
    void convertPositionCmd(
        const std::vector<irsl_shm_controller::irsl_float_type> &pos_float_vec,
        std::vector<int32_t> &dynamixel_position);

    /**
     * @brief Send position command to Dynamixel.
     *
     * @param dynamixel_position Input: Position command (raw value)
     * @return true Successful
     * @return false Failed to send command
     */
    bool writePosition(
        const std::vector<int32_t> &dynamixel_position);

    /**
     * @brief Convert velocity command from rad per sec. to raw value.
     *
     * This function converts the velocity command into
     * the raw value expected by the Dynamixel.
     *
     * @param vel_float_vec Input: Velocity command (rad per sec.)
     * @param dynamixel_velocity Output: Raw value for the Dynamixel
     */
    void convertVelocityCmd(
        const std::vector<irsl_shm_controller::irsl_float_type> &vel_float_vec,
        std::vector<int32_t> &dynamixel_velocity);

    /**
     * @brief Transfers velocity command to Dynamixel.
     *
     * Sends the velocity command to the Dynamixel.
     *
     * @param dynamixel_velocity Input: Velocity command (raw value)
     * @return true Successful
     * @return false Failed to send command
     */
    bool writeVelocity(
        const std::vector<int32_t> &dynamixel_velocity);

private:
    /**
     * @brief Writes a set of values using the specified SyncWrite handler.
     *
     * This function uses the provided SyncWrite handler index and value vector to
     * update the Dynamixel's settings. Currently, only two handlers are implemented,
     * one for goal position and one for goal velocity.
     *
     * @param handler_index Index of the SyncWrite handler to use (0 or 1)
     * @param value_vector Vector containing the values to write
     * @return true Successful
     * @return false Failed to update Dynamixel settings
     */
    bool writeBySyncHandler(
        uint8_t handler_index,
        const std::vector<int32_t> &value_vector);

private:
    // Dynamixel SDK
    std::unique_ptr<DynamixelWorkbench> dxl_wb_;

    // Device configuration
    std::vector<DynamixelInfo> dx_info;
    std::unordered_map<uint8_t, size_t> dx_info_index_map;

    // Control information
    std::map<std::string, const ControlItem *> control_items_;

    // Communication groups
    std::set<std::string> comm_group_names;
    std::map<std::string, std::vector<uint8_t>> comm_group_id_map;
};
