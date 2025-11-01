#include "UnitreeMavlinkControl.h"

namespace unitree::mav {

UnitreeMavlinkController::UnitreeMavlinkController(Stream &stream,
                                                   uint8_t system_id,
                                                   uint8_t component_id)
    : stream_(&stream),
      system_id_(system_id),
      component_id_(component_id),
      packet_id_counter_(0) {}

void UnitreeMavlinkController::bind_stream(Stream &stream) {
  stream_ = &stream;
}

bool UnitreeMavlinkController::set_work_mode(WorkMode mode) {
  mavlink_message_t msg;
  mavlink_msg_config_lidar_working_mode_pack(system_id_,
                                             component_id_,
                                             &msg,
                                             static_cast<uint8_t>(mode));
  return send_message(msg);
}

bool UnitreeMavlinkController::set_led_pattern(LedPattern pattern,
                                               uint32_t rotation_period_us,
                                               uint32_t zero_point_offset_us) {
  LedTable table{};
  return set_led_table(table,
                       rotation_period_us,
                       zero_point_offset_us,
                       pattern);
}

bool UnitreeMavlinkController::set_led_table(const LedTable &table,
                                             uint32_t rotation_period_us,
                                             uint32_t zero_point_offset_us,
                                             LedPattern display_mode) {
  mavlink_message_t msg;
  mavlink_msg_config_led_ring_table_packet_pack(system_id_,
                                                component_id_,
                                                &msg,
                                                next_packet_id(),
                                                rotation_period_us,
                                                zero_point_offset_us,
                                                static_cast<uint8_t>(display_mode),
                                                table.data());
  return send_message(msg);
}

bool UnitreeMavlinkController::save_configuration() {
  return send_device_command(CMD_LIDAR_SAVE_FLASH);
}

bool UnitreeMavlinkController::reboot_device() {
  return send_device_command(CMD_LIDAR_REBOOT);
}

bool UnitreeMavlinkController::send_device_command(uint8_t command_type) {
  mavlink_message_t msg;
  mavlink_msg_device_command_pack(system_id_,
                                  component_id_,
                                  &msg,
                                  command_type);
  return send_message(msg);
}

void UnitreeMavlinkController::set_ids(uint8_t system_id, uint8_t component_id) {
  system_id_ = system_id;
  component_id_ = component_id;
}

bool UnitreeMavlinkController::send_message(mavlink_message_t &msg) {
  if (!stream_) {
    return false;
  }
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);
  if (length == 0) {
    return false;
  }
  size_t written = stream_->write(buffer, length);
  if (written != length) {
    return false;
  }
  stream_->flush();
  return true;
}

uint16_t UnitreeMavlinkController::next_packet_id() {
  return ++packet_id_counter_;
}

}  // namespace unitree::mav
