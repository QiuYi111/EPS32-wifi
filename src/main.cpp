#include <Arduino.h>

#include "UnitreeMavlink.h"
#include "UnitreeMavlinkControl.h"

using namespace unitree::mav;

#ifndef UNITREE_RX_PIN
#define UNITREE_RX_PIN 18  // TODO: update to the UART RX pin wired to Unitree
#endif

#ifndef UNITREE_TX_PIN
#define UNITREE_TX_PIN 17  // TODO: update to the UART TX pin wired to Unitree
#endif

#ifndef UNITREE_BAUD_RATE
#define UNITREE_BAUD_RATE 2000000  // Replace if your Unitree device uses another rate
#endif

HardwareSerial &unitree_serial = Serial1;

UnitreeMavlinkParser parser;
LidarPipeline lidar_pipeline;
UnitreeMavlinkController controller(unitree_serial);

static void handle_imu(const ImuSample &imu) {
  Serial.printf("[IMU] packet=%u quat=(%.3f, %.3f, %.3f, %.3f) ang_vel=(%.3f, %.3f, %.3f) lin_acc=(%.3f, %.3f, %.3f)\n",
                imu.packet_id,
                imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3],
                imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2],
                imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2]);
}

static void handle_aux(const LidarAuxPacket &aux) {
  lidar_pipeline.handle_auxiliary(aux);
}

static void handle_distance(const LidarDistancePacket &dist) {
  lidar_pipeline.handle_distance(dist);
}

static void handle_cloud(uint16_t packet_id, const std::vector<PointXYZI> &cloud) {
  Serial.printf("[LIDAR] packet=%u points=%u\n", packet_id, static_cast<unsigned>(cloud.size()));
  for (size_t i = 0; i < cloud.size() && i < 5; ++i) {
    const auto &pt = cloud[i];
    Serial.printf("  #%u xyz=(%.3f, %.3f, %.3f) intensity=%.1f\n",
                  static_cast<unsigned>(i),
                  pt.x, pt.y, pt.z, pt.intensity);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1);
  Serial.println("[Unitree Demo] booting");

  unitree_serial.begin(UNITREE_BAUD_RATE, SERIAL_8N1, UNITREE_RX_PIN, UNITREE_TX_PIN);

  parser.set_imu_callback(handle_imu);
  parser.set_lidar_aux_callback(handle_aux);
  parser.set_lidar_distance_callback(handle_distance);

  lidar_pipeline.set_cloud_callback(handle_cloud);

  Serial.printf("[Unitree Demo] UART ready (baud=%d, RX=%d, TX=%d)\n",
                UNITREE_BAUD_RATE, UNITREE_RX_PIN, UNITREE_TX_PIN);

  if (!controller.set_work_mode(UnitreeMavlinkController::WorkMode::kNormal)) {
    Serial.println("[Unitree Demo] failed to set NORMAL mode");
  }
}

void loop() {
  parser.poll(unitree_serial);
  delay(1);
}
