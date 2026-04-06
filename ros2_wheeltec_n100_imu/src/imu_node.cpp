#include "wheeltec_n100_imu/imu_node.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <cmath>
#include <utility>

#include "wheeltec_n100_imu/crc_table.hpp"

class ImuNode : public rclcpp::Node
{
public:
  ImuNode()
  : Node("imu_node")
  {
    declare_parameters();
    load_parameters();

    q_rot_.setRPY(0.0, 0.0, yaw_offset_);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    imu_true_east_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_true_east_topic_, 10);
    mag_pose_pub_ = create_publisher<geometry_msgs::msg::Pose2D>(mag_pose_2d_topic_, 10);
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_, 10);

    setup_serial();
    timer_ = create_wall_timer(2ms, std::bind(&ImuNode::read_imu, this));
  }

private:
  void declare_parameters()
  {
    declare_parameter("debug", false);
    declare_parameter("serial_port", "/dev/ttyACM0");
    declare_parameter("serial_baud", 921600);
    declare_parameter("serial_timeout", 20);
    declare_parameter("device_type", 1);
    declare_parameter("frist_sn", false);
    declare_parameter("imu_topic", "imu");
    declare_parameter("imu_frame", "imu");
    declare_parameter("mag_pose_2d_topic", "magnetic_pose_2d");
    declare_parameter("imu_trueEast_topic", "imu_trueEast");
    declare_parameter("mag_topic", "magnetic_field");
    declare_parameter("yaw_offset", -2.094);
    declare_parameter("mag_offset_x", 0.0);
    declare_parameter("mag_offset_y", 0.0);
    declare_parameter("mag_offset_z", 0.0);
    declare_parameter("imu_mag_covVec", IMU_MAG_COV);
    declare_parameter("imu_gyro_covVec", IMU_GYRO_COV);
    declare_parameter("imu_accel_covVec", IMU_ACCEL_COV);
  }

  void load_parameters()
  {
    if_debug_ = get_parameter("debug").as_bool();
    serial_port_ = get_parameter("serial_port").as_string();
    serial_baud_ = static_cast<uint32_t>(get_parameter("serial_baud").as_int());
    serial_timeout_ = static_cast<uint32_t>(get_parameter("serial_timeout").as_int());
    device_type_ = static_cast<int>(get_parameter("device_type").as_int());
    first_sn_ = get_parameter("frist_sn").as_bool();

    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_true_east_topic_ = get_parameter("imu_trueEast_topic").as_string();
    mag_pose_2d_topic_ = get_parameter("mag_pose_2d_topic").as_string();
    mag_topic_ = get_parameter("mag_topic").as_string();
    imu_frame_id_ = get_parameter("imu_frame").as_string();

    yaw_offset_ = get_parameter("yaw_offset").as_double();
    mag_offset_x_ = get_parameter("mag_offset_x").as_double();
    mag_offset_y_ = get_parameter("mag_offset_y").as_double();
    mag_offset_z_ = get_parameter("mag_offset_z").as_double();

    imu_mag_cov_ = get_parameter("imu_mag_covVec").as_double_array();
    imu_gyro_cov_ = get_parameter("imu_gyro_covVec").as_double_array();
    imu_accel_cov_ = get_parameter("imu_accel_covVec").as_double_array();

    mag_covariance_ = imu_mag_cov_.empty() ? 0.0 : imu_mag_cov_[0];
  }

  void setup_serial()
  {
    try {
      serial_.setPort(serial_port_);
      serial_.setBaudrate(serial_baud_);
      serial_.setFlowcontrol(serial::flowcontrol_none);
      serial_.setParity(serial::parity_none);
      serial_.setStopbits(serial::stopbits_one);
      serial_.setBytesize(serial::eightbits);
      serial_.setTimeout(serial::Timeout::simpleTimeout(serial_timeout_));
      serial_.open();
    } catch (const serial::IOException & e) {
      RCLCPP_FATAL(get_logger(), "Unable to open serial port %s (%s)", serial_port_.c_str(), e.what());
      throw;
    }

    if (!serial_.isOpen()) {
      throw std::runtime_error("Serial port failed to open: " + serial_port_);
    }

    RCLCPP_INFO(get_logger(), "Initialized serial port: %s @ %u", serial_port_.c_str(), serial_baud_);
  }

  bool read_exact(uint8_t * buffer, size_t bytes)
  {
    return serial_.read(buffer, bytes) == bytes;
  }

  void read_imu()
  {
    if (!serial_.isOpen()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial is not open");
      return;
    }

    uint8_t check_head = 0xff;
    if (!read_exact(&check_head, 1) || check_head != FRAME_HEAD) {
      return;
    }

    uint8_t head_type = 0xff;
    if (!read_exact(&head_type, 1)) {
      return;
    }

    if (head_type != TYPE_IMU && head_type != TYPE_AHRS && head_type != TYPE_INSGPS &&
      head_type != TYPE_GROUND && head_type != 0x50)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid frame type: 0x%02x", head_type);
      return;
    }

    uint8_t frame_len = 0xff;
    if (!read_exact(&frame_len, 1)) {
      return;
    }

    if ((head_type == TYPE_IMU && frame_len != IMU_LEN) ||
      (head_type == TYPE_AHRS && frame_len != AHRS_LEN) ||
      (head_type == TYPE_INSGPS && frame_len != INSGPS_LEN))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Length mismatch for frame type 0x%02x", head_type);
      return;
    }

    if (head_type == TYPE_GROUND || head_type == 0x50) {
      uint8_t ground_sn = 0;
      if (!read_exact(&ground_sn, 1)) {
        return;
      }
      update_sn(ground_sn);

      std::array<uint8_t, 512> ignore{};
      const size_t remain = static_cast<size_t>(frame_len) + 4;
      read_exact(ignore.data(), remain);
      return;
    }

    std::array<uint8_t, 4> header_tail{};
    if (!read_exact(header_tail.data(), header_tail.size())) {
      return;
    }

    const uint8_t serial_num = header_tail[0];
    const uint8_t header_crc8 = header_tail[1];
    const uint16_t header_crc16 = static_cast<uint16_t>(header_tail[3]) |
      (static_cast<uint16_t>(header_tail[2]) << 8);

    std::array<uint8_t, 4> header_for_crc = {check_head, head_type, frame_len, serial_num};
    if (CRC8_Table(header_for_crc.data(), header_for_crc.size()) != header_crc8) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "header crc8 error");
      return;
    }

    if (!first_sn_) {
      read_sn_ = static_cast<uint8_t>(serial_num - 1);
      first_sn_ = true;
    }
    update_sn(serial_num);

    if (head_type == TYPE_IMU) {
      imu_frame_.frame.header = {check_head, head_type, frame_len, serial_num, header_crc8,
        header_tail[2], header_tail[3]};
      if (!read_exact(imu_frame_.read_buf.read_msg, IMU_LEN + 1)) {
        return;
      }
      validate_and_publish_imu(header_crc16, imu_frame_.frame.data.data_buff, IMU_LEN,
        imu_frame_.frame.frame_end, head_type);
    } else if (head_type == TYPE_AHRS) {
      ahrs_frame_.frame.header = {check_head, head_type, frame_len, serial_num, header_crc8,
        header_tail[2], header_tail[3]};
      if (!read_exact(ahrs_frame_.read_buf.read_msg, AHRS_LEN + 1)) {
        return;
      }
      if (!validate_and_publish_imu(header_crc16, ahrs_frame_.frame.data.data_buff, AHRS_LEN,
        ahrs_frame_.frame.frame_end, head_type))
      {
        return;
      }
      publish_messages();
    } else if (head_type == TYPE_INSGPS) {
      insgps_frame_.frame.header = {check_head, head_type, frame_len, serial_num, header_crc8,
        header_tail[2], header_tail[3]};
      if (!read_exact(insgps_frame_.read_buf.read_msg, INSGPS_LEN + 1)) {
        return;
      }
      validate_and_publish_imu(header_crc16, insgps_frame_.frame.data.data_buff, INSGPS_LEN,
        insgps_frame_.frame.frame_end, head_type);
    }
  }

  bool validate_and_publish_imu(
    uint16_t header_crc16,
    const uint8_t * payload,
    uint8_t expected_len,
    uint8_t frame_end,
    uint8_t frame_type)
  {
    const uint16_t crc16 = CRC16_Table(payload, expected_len);
    if (crc16 != header_crc16) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "crc16 failed type=0x%02x", frame_type);
      ++crc_error_;
      return false;
    }
    if (frame_end != FRAME_END) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "frame end invalid type=0x%02x", frame_type);
      return false;
    }
    return true;
  }

  void publish_messages()
  {
    sensor_msgs::msg::Imu imu_data;
    imu_data.header.stamp = get_clock()->now();
    imu_data.header.frame_id = imu_frame_id_;

    const Eigen::Quaterniond q_rr =
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX());
    const Eigen::Quaterniond q_z =
      Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

    if (device_type_ == 0) {
      imu_data.orientation.w = ahrs_frame_.frame.data.data_pack.Qw;
      imu_data.orientation.x = ahrs_frame_.frame.data.data_pack.Qx;
      imu_data.orientation.y = ahrs_frame_.frame.data.data_pack.Qy;
      imu_data.orientation.z = ahrs_frame_.frame.data.data_pack.Qz;
    } else {
      const Eigen::Quaterniond q_ahrs(
        ahrs_frame_.frame.data.data_pack.Qw,
        ahrs_frame_.frame.data.data_pack.Qx,
        ahrs_frame_.frame.data.data_pack.Qy,
        ahrs_frame_.frame.data.data_pack.Qz);
      const Eigen::Quaterniond q_out = q_z * q_rr * q_ahrs;
      imu_data.orientation.w = q_out.w();
      imu_data.orientation.x = q_out.x();
      imu_data.orientation.y = q_out.y();
      imu_data.orientation.z = q_out.z();
    }

    imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
    imu_data.angular_velocity.y = ahrs_frame_.frame.data.data_pack.PitchSpeed;
    imu_data.angular_velocity.z = ahrs_frame_.frame.data.data_pack.HeadingSpeed;
    imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
    imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
    imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;

    set_diag_covariance(imu_data.orientation_covariance, imu_mag_cov_);
    set_diag_covariance(imu_data.angular_velocity_covariance, imu_gyro_cov_);
    set_diag_covariance(imu_data.linear_acceleration_covariance, imu_accel_cov_);

    imu_pub_->publish(imu_data);

    sensor_msgs::msg::Imu imu_true_east = imu_data;
    tf2::Quaternion q_orig(
      imu_data.orientation.x,
      imu_data.orientation.y,
      imu_data.orientation.z,
      imu_data.orientation.w);
    tf2::Quaternion q_new = q_rot_ * q_orig;
    q_new.normalize();
    imu_true_east.orientation.x = q_new.x();
    imu_true_east.orientation.y = q_new.y();
    imu_true_east.orientation.z = q_new.z();
    imu_true_east.orientation.w = q_new.w();
    imu_true_east_pub_->publish(imu_true_east);

    const auto [roll, pitch] = compute_roll_pitch(imu_data);
    double magx = imu_frame_.frame.data.data_pack.magnetometer_x * MAG_SCALE_MILLI_GAUSS_TO_TESLA - mag_offset_x_;
    double magy = imu_frame_.frame.data.data_pack.magnetometer_y * MAG_SCALE_MILLI_GAUSS_TO_TESLA - mag_offset_y_;
    double magz = imu_frame_.frame.data.data_pack.magnetometer_z * MAG_SCALE_MILLI_GAUSS_TO_TESLA - mag_offset_z_;

    geometry_msgs::msg::Pose2D pose_2d;
    pose_2d.theta = calculate_mag_yaw(roll, pitch, magx, magy, magz);
    mag_pose_pub_->publish(pose_2d);

    sensor_msgs::msg::MagneticField mag;
    mag.header = imu_data.header;
    mag.magnetic_field.x = magx;
    mag.magnetic_field.y = magy;
    mag.magnetic_field.z = magz;
    std::fill(mag.magnetic_field_covariance.begin(), mag.magnetic_field_covariance.end(), mag_covariance_);
    mag_pub_->publish(mag);
  }

  void set_diag_covariance(std::array<double, 9> & cov, const std::vector<double> & diagonal)
  {
    cov.fill(0.0);
    if (diagonal.size() >= 3U) {
      cov[0] = diagonal[0];
      cov[4] = diagonal[1];
      cov[8] = diagonal[2];
    }
  }

  std::pair<double, double> compute_roll_pitch(const sensor_msgs::msg::Imu & imu_data) const
  {
    if (device_type_ == 0) {
      return {ahrs_frame_.frame.data.data_pack.Roll, ahrs_frame_.frame.data.data_pack.Pitch};
    }

    Eigen::Quaterniond q(
      imu_data.orientation.w,
      imu_data.orientation.x,
      imu_data.orientation.y,
      imu_data.orientation.z);
    const Eigen::Vector3d euler = q.matrix().eulerAngles(2, 1, 0);
    return {euler[2], euler[1]};
  }

  double calculate_mag_yaw(double roll, double pitch, double magx, double magy, double magz) const
  {
    const double temp1 = magy * std::cos(roll) + magz * std::sin(roll);
    const double temp2 = magx * std::cos(pitch) +
      magy * std::sin(pitch) * std::sin(roll) -
      magz * std::sin(pitch) * std::cos(roll);
    double magyaw = std::atan2(-temp1, temp2);
    if (magyaw < 0.0) {
      magyaw += 2.0 * PI;
    }
    return magyaw;
  }

  void update_sn(uint8_t current_sn)
  {
    if (++read_sn_ != current_sn) {
      if (current_sn < read_sn_) {
        sn_lost_ += 256 - static_cast<int>(read_sn_ - current_sn);
      } else {
        sn_lost_ += static_cast<int>(current_sn - read_sn_);
      }
      if (if_debug_) {
        RCLCPP_WARN(get_logger(), "Detected serial number loss (lost total=%d)", sn_lost_);
      }
    }
    read_sn_ = current_sn;
  }

private:
  serial::Serial serial_;
  bool if_debug_{false};

  std::string serial_port_;
  uint32_t serial_baud_{921600};
  uint32_t serial_timeout_{20};

  int sn_lost_{0};
  int crc_error_{0};
  uint8_t read_sn_{0};
  bool first_sn_{false};
  int device_type_{1};

  std::vector<double> imu_mag_cov_;
  std::vector<double> imu_gyro_cov_;
  std::vector<double> imu_accel_cov_;

  std::string imu_topic_;
  std::string mag_pose_2d_topic_;
  std::string imu_true_east_topic_;
  std::string mag_topic_;
  std::string imu_frame_id_;

  double yaw_offset_{-2.094};
  tf2::Quaternion q_rot_;
  double mag_offset_x_{0.0};
  double mag_offset_y_{0.0};
  double mag_offset_z_{0.0};
  double mag_covariance_{0.01};

  FDILink::imu_frame_read imu_frame_{};
  FDILink::ahrs_frame_read ahrs_frame_{};
  FDILink::insgps_frame_read insgps_frame_{};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_true_east_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr mag_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
