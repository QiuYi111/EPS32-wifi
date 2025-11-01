#pragma once

#include <Arduino.h>
#include <array>
#include <cstdint>

#include <mavlink/SysMavlink/mavlink.h>

namespace unitree::mav {

class UnitreeMavlinkController {
 public:
  enum class WorkMode : uint8_t {
    kNormal = NORMAL_MODE,
    kStandby = STANDBY_MODE,
    kRaw = RAW_MODE,
  };

  enum class LedPattern : uint8_t {
    kCommandTable = LED_RING_COMMAND_MODE,
    kForwardSlow = LED_RING_FUN_FORWARD_SLOW_MODE,
    kForwardFast = LED_RING_FUN_FORWARD_FAST_MODE,
    kReverseSlow = LED_RING_FUN_REVERSE_SLOW_MODE,
    kReverseFast = LED_RING_FUN_REVERSE_FAST_MODE,
    kTripleFlip = LED_RING_FUN_TRIPLE_FLIP_MODE,
    kTripleBreathing = LED_RING_FUN_TRIPLE_BREATHING_MODE,
    kSixStageBreathing = LED_RING_FUN_SIXSTAGE_BREATHING_MODE,
  };

  using LedTable = std::array<uint8_t, MAVLINK_MSG_CONFIG_LED_RING_TABLE_PACKET_FIELD_LED_TABLE_LEN>;

  explicit UnitreeMavlinkController(Stream &stream,
                                    uint8_t system_id = 1,
                                    uint8_t component_id = 200);

  void bind_stream(Stream &stream);

  bool set_work_mode(WorkMode mode);
  bool set_led_pattern(LedPattern pattern,
                       uint32_t rotation_period_us = kDefaultLedRotationPeriodUs,
                       uint32_t zero_point_offset_us = 0);
  bool set_led_table(const LedTable &table,
                     uint32_t rotation_period_us = kDefaultLedRotationPeriodUs,
                     uint32_t zero_point_offset_us = 0,
                     LedPattern display_mode = LedPattern::kCommandTable);
  bool save_configuration();
  bool reboot_device();
  bool send_device_command(uint8_t command_type);

  void set_ids(uint8_t system_id, uint8_t component_id);

  static constexpr uint32_t kDefaultLedRotationPeriodUs = 200000;

 private:
  bool send_message(mavlink_message_t &msg);
  uint16_t next_packet_id();

  Stream *stream_;
  uint8_t system_id_;
  uint8_t component_id_;
  uint16_t packet_id_counter_;
};

}  // namespace unitree::mav

