#pragma once

#include "irsl/simple_yaml_parser.hpp"
using namespace irsl_common_utils;

static const std::string hardware_setings_name = "dynamixel_hardware_shm";

/**
 * @brief ポート・周期などのインタフェース設定
 */
struct DynamixelHardwareShm
{
    double period;         ///< 制御周期（秒）
    std::string port_name; ///< デバイス名（例: "/dev/ttyUSB0"）
    int32_t baud_rate;     ///< 通信速度（例: 1000000）
};

namespace YAML
{
    //// YAML auto conversion
    template <>
    struct convert<DynamixelHardwareShm>
    {
        static inline bool decode(const Node &node, DynamixelHardwareShm &cType)
        {
            cType.period = node["period"].as<double>();
            cType.port_name = node["port_name"].as<std::string>();
            cType.baud_rate = node["baud_rate"].as<int32_t>();
            return true;
        }
    };

}
