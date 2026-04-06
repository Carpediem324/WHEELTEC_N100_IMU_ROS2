#ifndef WHEELTEC_N100_IMU__IMU_NODE_HPP_
#define WHEELTEC_N100_IMU__IMU_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "serial/serial.h"
#include "wheeltec_n100_imu/fdilink_data_struct.h"

using namespace std::chrono_literals;

constexpr uint8_t FRAME_HEAD = 0xfc;
constexpr uint8_t FRAME_END = 0xfd;
constexpr uint8_t TYPE_IMU = 0x40;
constexpr uint8_t TYPE_AHRS = 0x41;
constexpr uint8_t TYPE_INSGPS = 0x42;
constexpr uint8_t TYPE_GROUND = 0xf0;
constexpr uint8_t IMU_LEN = 0x38;
constexpr uint8_t AHRS_LEN = 0x30;
constexpr uint8_t INSGPS_LEN = 0x54;

constexpr double PI = 3.141592653589793;
constexpr double MAG_SCALE_MILLI_GAUSS_TO_TESLA = 1.0e-7;

const std::vector<double> IMU_MAG_COV = {0.01, 0.01, 0.01};
const std::vector<double> IMU_GYRO_COV = {0.01, 0.01, 0.01};
const std::vector<double> IMU_ACCEL_COV = {0.05, 0.05, 0.05};

#endif  // WHEELTEC_N100_IMU__IMU_NODE_HPP_
