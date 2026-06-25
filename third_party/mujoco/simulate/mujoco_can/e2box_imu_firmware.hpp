#pragma once

#include "imu_firmware_base.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace mjcan {

struct E2BoxImuFirmwareConfig {
  uint32_t request_id = 0x221;
  uint32_t quat_id = 0x2A1;
  uint32_t gyro_id = 0x321;

  uint8_t cmd_get_quat = 0x01;
  uint8_t cmd_get_gyro = 0x02;
  uint8_t cmd_get_all = 0x03;

  // Python decoder 기준:
  //   raw / 10000.0
  double quat_scale = 10000.0;

  // Python decoder 기준:
  //   raw / 100.0 deg/s
  double gyro_scale = 100.0;

  bool normalize_quat = true;
};

class E2BoxImuFirmware final : public ImuFirmwareBase {
public:
  explicit E2BoxImuFirmware(
      E2BoxImuFirmwareConfig config = E2BoxImuFirmwareConfig());

  ~E2BoxImuFirmware() override = default;

  std::string name() const override;

  bool accepts(const CanFrame& frame) const override;

  void reset(double sim_time) override;

  std::vector<CanFrame> on_can_frame(
      const CanFrame& frame,
      const ImuSample& sample,
      double sim_time) override;

private:
  CanFrame make_quat_frame(const ImuSample& sample) const;
  CanFrame make_gyro_frame(const ImuSample& sample) const;

  static void normalize_quat_xyzw(double* qx, double* qy, double* qz, double* qw);

  static int16_t saturate_to_i16(double value);
  static void write_i16_le(CanFrame* frame, int offset, int16_t value);

private:
  E2BoxImuFirmwareConfig config_;
};

}  // namespace mjcan