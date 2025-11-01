#include "UnitreeMavlink.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace unitree::mav {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

template <size_t N>
constexpr void copy_float_array(std::array<float, N> &dst, const float *src) {
  for (size_t i = 0; i < N; ++i) {
    dst[i] = src[i];
  }
}

template <size_t N>
constexpr void copy_uint8_array(std::array<uint8_t, N> &dst, const uint8_t *src) {
  for (size_t i = 0; i < N; ++i) {
    dst[i] = src[i];
  }
}

}  // namespace

PointCloudAssembler::PointCloudAssembler(const Calibration &calibration)
    : calibration_(calibration) {}

bool PointCloudAssembler::assemble(const LidarAuxPacket &auxiliary,
                                   const LidarDistancePacket &distance,
                                   std::vector<PointXYZI> &cloud) const {
  if (auxiliary.packet_id != distance.packet_id) {
    return false;
  }
  if (distance.payload_size == 0 || auxiliary.payload_size == 0) {
    return false;
  }

  const size_t point_count =
      std::min<size_t>(kReflectCount, static_cast<size_t>(distance.payload_size / 2));
  const size_t intensity_count =
      std::min<size_t>(kReflectCount, static_cast<size_t>(auxiliary.payload_size));
  if (point_count == 0) {
    return false;
  }

  cloud.clear();
  cloud.reserve(point_count);

  float theta = auxiliary.theta_angle * kDegToRad;
  float ksi = auxiliary.ksi_angle * kDegToRad;
  float sin_theta = std::sin(theta);
  float cos_theta = std::cos(theta);
  float sin_ksi = std::sin(ksi);
  float cos_ksi = std::cos(ksi);
  float yaw_bias =
      (auxiliary.com_horizontal_angle_start + calibration_.rotate_yaw_bias_deg) * kDegToRad;
  float yaw_step =
      (point_count > 0 ? auxiliary.com_horizontal_angle_step / static_cast<float>(point_count)
                       : 0.f) *
      kDegToRad;
  float pitch_start = auxiliary.sys_vertical_angle_start * kDegToRad;
  float pitch_step = auxiliary.sys_vertical_angle_span * kDegToRad;
  float bias_laser_beam = auxiliary.b_axis_dist * 0.001f;

  float yaw = yaw_bias;
  float pitch = pitch_start;

  for (size_t point = 0, i = 0; point < point_count; ++point, i += 2) {
    uint16_t range_raw = static_cast<uint16_t>(distance.point_data[i]) |
                         (static_cast<uint16_t>(distance.point_data[i + 1]) << 8);
    if (range_raw == 0 && calibration_.skip_zero_distance) {
      yaw += yaw_step;
      pitch += pitch_step;
      continue;
    }

    float range = calibration_.range_scale * static_cast<float>(range_raw);
    float sin_alpha = std::sin(pitch);
    float cos_alpha = std::cos(pitch);
    float sin_beta = std::sin(yaw);
    float cos_beta = std::cos(yaw);

    float A = (-cos_theta * sin_ksi + sin_theta * sin_alpha * cos_ksi) * range + bias_laser_beam;
    float B = cos_alpha * cos_ksi * range;

    PointXYZI point_xyz;
    point_xyz.x = cos_beta * A - sin_beta * B;
    point_xyz.y = sin_beta * A + cos_beta * B;
    point_xyz.z = (sin_theta * sin_ksi + cos_theta * sin_alpha * cos_ksi) * range + calibration_.z_bias;
    uint8_t intensity = 0;
    if (point < intensity_count) {
      intensity = auxiliary.reflect_data[point];
    }
    point_xyz.intensity = static_cast<float>(intensity);
    cloud.push_back(point_xyz);

    yaw += yaw_step;
    pitch += pitch_step;
  }

  return !cloud.empty();
}

LidarPipeline::LidarPipeline(const Calibration &calibration, size_t reserve_points)
    : assembler_(calibration) {
  cloud_buffer_.reserve(reserve_points);
}

void LidarPipeline::set_cloud_callback(CloudCallback cb) {
  cloud_cb_ = std::move(cb);
}

void LidarPipeline::handle_auxiliary(const LidarAuxPacket &packet) {
  last_aux_ = packet;
  has_aux_ = true;
}

void LidarPipeline::handle_distance(const LidarDistancePacket &packet) {
  if (!has_aux_ || last_aux_.packet_id != packet.packet_id) {
    return;
  }
  if (!assembler_.assemble(last_aux_, packet, cloud_buffer_)) {
    return;
  }
  if (cloud_cb_) {
    cloud_cb_(packet.packet_id, cloud_buffer_);
  }
}

void LidarPipeline::reset() {
  has_aux_ = false;
  cloud_buffer_.clear();
}

UnitreeMavlinkParser::UnitreeMavlinkParser(uint8_t channel) : channel_(channel) {
  reset();
}

void UnitreeMavlinkParser::set_imu_callback(ImuCallback cb) {
  imu_cb_ = std::move(cb);
}

void UnitreeMavlinkParser::set_lidar_aux_callback(LidarAuxCallback cb) {
  aux_cb_ = std::move(cb);
}

void UnitreeMavlinkParser::set_lidar_distance_callback(LidarDistanceCallback cb) {
  distance_cb_ = std::move(cb);
}

void UnitreeMavlinkParser::feed(uint8_t byte) {
  mavlink_message_t message;
  if (mavlink_parse_char(channel_, byte, &message, &status_)) {
    dispatch_message(message);
  }
}

void UnitreeMavlinkParser::poll(Stream &stream, size_t max_bytes) {
  size_t processed = 0;
  while (stream.available() > 0) {
    if (max_bytes && processed >= max_bytes) {
      return;
    }
    int value = stream.read();
    if (value < 0) {
      break;
    }
    feed(static_cast<uint8_t>(value));
    ++processed;
  }
}

void UnitreeMavlinkParser::reset() {
  status_ = mavlink_status_t{};
}

void UnitreeMavlinkParser::dispatch_message(const mavlink_message_t &msg) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_RET_IMU_ATTITUDE_DATA_PACKET: {
      if (!imu_cb_) {
        break;
      }
      mavlink_ret_imu_attitude_data_packet_t raw;
      mavlink_msg_ret_imu_attitude_data_packet_decode(&msg, &raw);
      ImuSample sample;
      sample.packet_id = raw.packet_id;
      copy_float_array(sample.quaternion, raw.quaternion);
      copy_float_array(sample.angular_velocity, raw.angular_velocity);
      copy_float_array(sample.linear_acceleration, raw.linear_acceleration);
      imu_cb_(sample);
      break;
    }
    case MAVLINK_MSG_ID_RET_LIDAR_AUXILIARY_DATA_PACKET: {
      if (!aux_cb_) {
        break;
      }
      mavlink_ret_lidar_auxiliary_data_packet_t raw;
      mavlink_msg_ret_lidar_auxiliary_data_packet_decode(&msg, &raw);
      LidarAuxPacket packet;
      packet.packet_id = raw.packet_id;
      packet.payload_size = raw.payload_size;
      packet.lidar_work_status = raw.lidar_work_status;
      packet.lidar_sync_delay_time = raw.lidar_sync_delay_time;
      packet.time_stamp_s_step = raw.time_stamp_s_step;
      packet.time_stamp_us_step = raw.time_stamp_us_step;
      packet.sys_rotation_period = raw.sys_rotation_period;
      packet.com_rotation_period = raw.com_rotation_period;
      packet.com_horizontal_angle_start = raw.com_horizontal_angle_start;
      packet.com_horizontal_angle_step = raw.com_horizontal_angle_step;
      packet.sys_vertical_angle_start = raw.sys_vertical_angle_start;
      packet.sys_vertical_angle_span = raw.sys_vertical_angle_span;
      packet.apd_temperature = raw.apd_temperature;
      packet.dirty_index = raw.dirty_index;
      packet.imu_temperature = raw.imu_temperature;
      packet.up_optical_q = raw.up_optical_q;
      packet.down_optical_q = raw.down_optical_q;
      packet.apd_voltage = raw.apd_voltage;
      packet.imu_angle_x_offset = raw.imu_angle_x_offset;
      packet.imu_angle_y_offset = raw.imu_angle_y_offset;
      packet.imu_angle_z_offset = raw.imu_angle_z_offset;
      packet.b_axis_dist = raw.b_axis_dist;
      packet.theta_angle = raw.theta_angle;
      packet.ksi_angle = raw.ksi_angle;
      copy_uint8_array(packet.reflect_data, raw.reflect_data);
      aux_cb_(packet);
      break;
    }
    case MAVLINK_MSG_ID_RET_LIDAR_DISTANCE_DATA_PACKET: {
      if (!distance_cb_) {
        break;
      }
      mavlink_ret_lidar_distance_data_packet_t raw;
      mavlink_msg_ret_lidar_distance_data_packet_decode(&msg, &raw);
      LidarDistancePacket packet;
      packet.packet_id = raw.packet_id;
      packet.packet_cnt = raw.packet_cnt;
      packet.payload_size = raw.payload_size;
      copy_uint8_array(packet.point_data, raw.point_data);
      distance_cb_(packet);
      break;
    }
    default:
      break;
  }
}

}  // namespace unitree::mav
