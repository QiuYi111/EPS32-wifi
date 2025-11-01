#pragma once

#include <Arduino.h>
#include <array>
#include <cstdint>
#include <functional>
#include <vector>

#include <mavlink/SysMavlink/mavlink.h>

namespace unitree::mav {

constexpr size_t kReflectCount = 120;
constexpr size_t kDistanceBytes = 240;

struct ImuSample {
  uint16_t packet_id = 0;
  std::array<float, 4> quaternion{};
  std::array<float, 3> angular_velocity{};
  std::array<float, 3> linear_acceleration{};
};

struct LidarAuxPacket {
  uint16_t packet_id = 0;
  uint16_t payload_size = 0;
  uint8_t lidar_work_status = 0;
  uint32_t lidar_sync_delay_time = 0;
  uint32_t time_stamp_s_step = 0;
  uint32_t time_stamp_us_step = 0;
  uint32_t sys_rotation_period = 0;
  uint32_t com_rotation_period = 0;
  float com_horizontal_angle_start = 0.f;
  float com_horizontal_angle_step = 0.f;
  float sys_vertical_angle_start = 0.f;
  float sys_vertical_angle_span = 0.f;
  float apd_temperature = 0.f;
  float dirty_index = 0.f;
  float imu_temperature = 0.f;
  float up_optical_q = 0.f;
  float down_optical_q = 0.f;
  float apd_voltage = 0.f;
  float imu_angle_x_offset = 0.f;
  float imu_angle_y_offset = 0.f;
  float imu_angle_z_offset = 0.f;
  float b_axis_dist = 0.f;
  float theta_angle = 0.f;
  float ksi_angle = 0.f;
  std::array<uint8_t, kReflectCount> reflect_data{};
};

struct LidarDistancePacket {
  uint16_t packet_id = 0;
  uint16_t packet_cnt = 0;
  uint16_t payload_size = 0;
  std::array<uint8_t, kDistanceBytes> point_data{};
};

struct PointXYZI {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  float intensity = 0.f;
};

struct Calibration {
  float rotate_yaw_bias_deg = 0.f;
  float range_scale = 0.001f;
  float z_bias = 0.0445f;
  bool skip_zero_distance = true;
};

class PointCloudAssembler {
 public:
  explicit PointCloudAssembler(const Calibration &calibration = Calibration{});

  bool assemble(const LidarAuxPacket &auxiliary,
                const LidarDistancePacket &distance,
                std::vector<PointXYZI> &cloud) const;

 private:
  Calibration calibration_;
};

class LidarPipeline {
 public:
  using CloudCallback =
      std::function<void(uint16_t packet_id, const std::vector<PointXYZI> &cloud)>;

  explicit LidarPipeline(const Calibration &calibration = Calibration{},
                         size_t reserve_points = kReflectCount);

  void set_cloud_callback(CloudCallback cb);
  void handle_auxiliary(const LidarAuxPacket &packet);
  void handle_distance(const LidarDistancePacket &packet);
  void reset();

 private:
  PointCloudAssembler assembler_;
  CloudCallback cloud_cb_;
  LidarAuxPacket last_aux_{};
  std::vector<PointXYZI> cloud_buffer_;
  bool has_aux_ = false;
};

class UnitreeMavlinkParser {
 public:
  using ImuCallback = std::function<void(const ImuSample &)>;
  using LidarAuxCallback = std::function<void(const LidarAuxPacket &)>;
  using LidarDistanceCallback = std::function<void(const LidarDistancePacket &)>;

  explicit UnitreeMavlinkParser(uint8_t channel = MAVLINK_COMM_0);

  void set_imu_callback(ImuCallback cb);
  void set_lidar_aux_callback(LidarAuxCallback cb);
  void set_lidar_distance_callback(LidarDistanceCallback cb);

  void feed(uint8_t byte);
  void poll(Stream &stream, size_t max_bytes = 0);
  void reset();

 private:
  void dispatch_message(const mavlink_message_t &msg);

  uint8_t channel_;
  mavlink_status_t status_{};
  ImuCallback imu_cb_;
  LidarAuxCallback aux_cb_;
  LidarDistanceCallback distance_cb_;
};

}  // namespace unitree::mav
