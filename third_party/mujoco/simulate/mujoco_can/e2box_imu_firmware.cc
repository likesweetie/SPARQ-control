#include "e2box_imu_firmware.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace mjcan {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

template <typename T>
T clamp_value(T value, T lo, T hi) {
  return std::max(lo, std::min(value, hi));
}

}  // namespace

E2BoxImuFirmware::E2BoxImuFirmware(E2BoxImuFirmwareConfig config)
    : config_(config) {}

std::string E2BoxImuFirmware::name() const {
  return "E2BoxImuFirmware";
}

bool E2BoxImuFirmware::accepts(const CanFrame& frame) const {
  return frame.can_id == config_.request_id;
}

void E2BoxImuFirmware::reset(double /*sim_time*/) {
  // 현재 E2Box IMU firmware는 stateless request-response emulator입니다.
  // bias, latency, sample-hold, dropout 등을 넣고 싶으면 여기 상태를 추가하면 됩니다.
}

std::vector<CanFrame> E2BoxImuFirmware::on_can_frame(
    const CanFrame& frame,
    const ImuSample& sample,
    double /*sim_time*/) {
  std::vector<CanFrame> out;

  if (!accepts(frame) || frame.dlc < 1) {
    return out;
  }

  const uint8_t command = frame.data[0];

  if (command == config_.cmd_get_quat) {
    out.push_back(make_quat_frame(sample));
    return out;
  }

  if (command == config_.cmd_get_gyro) {
    out.push_back(make_gyro_frame(sample));
    return out;
  }

  if (command == config_.cmd_get_all) {
    out.push_back(make_quat_frame(sample));
    out.push_back(make_gyro_frame(sample));
    return out;
  }

  return out;
}

CanFrame E2BoxImuFirmware::make_quat_frame(const ImuSample& sample) const {
  double qx = sample.quat_xyzw[0];
  double qy = sample.quat_xyzw[1];
  double qz = sample.quat_xyzw[2];
  double qw = sample.quat_xyzw[3];

  if (config_.normalize_quat) {
    normalize_quat_xyzw(&qx, &qy, &qz, &qw);
  }

  CanFrame frame;
  frame.can_id = config_.quat_id;
  frame.dlc = 8;
  frame.data.fill(0);

  // Python decoder:
  //
  //   qz_raw, qy_raw, qx_raw, qw_raw = struct.unpack("<hhhh", data)
  //
  //   qz = qz_raw / 10000.0
  //   qy = qy_raw / 10000.0
  //   qx = qx_raw / 10000.0
  //   qw = qw_raw / 10000.0
  //
  //   qx = -qx
  //
  // 따라서 host에서 최종 qx가 원래 qx가 되려면:
  //
  //   qx_raw = -qx * 10000
  //
  const int16_t qz_raw = saturate_to_i16(qz * config_.quat_scale);
  const int16_t qy_raw = saturate_to_i16(qy * config_.quat_scale);
  const int16_t qx_raw = saturate_to_i16((-qx) * config_.quat_scale);
  const int16_t qw_raw = saturate_to_i16(qw * config_.quat_scale);

  write_i16_le(&frame, 0, qz_raw);
  write_i16_le(&frame, 2, qy_raw);
  write_i16_le(&frame, 4, qx_raw);
  write_i16_le(&frame, 6, qw_raw);

  return frame;
}

CanFrame E2BoxImuFirmware::make_gyro_frame(const ImuSample& sample) const {
  const double gx = sample.angular_velocity_rad_s[0];
  const double gy = sample.angular_velocity_rad_s[1];
  const double gz = sample.angular_velocity_rad_s[2];

  CanFrame frame;
  frame.can_id = config_.gyro_id;
  frame.dlc = 8;
  frame.data.fill(0);

  // Python decoder:
  //
  //   gx_raw, gy_raw, gz_raw, _ = struct.unpack("<hhhh", data)
  //
  //   gx = (gx_raw / 100.0) * pi / 180
  //   gy = (gy_raw / 100.0) * pi / 180
  //   gz = (gz_raw / 100.0) * pi / 180
  //
  //   gx, gy = gy, gx
  //
  // 즉, host에서 최종 decoded angular velocity가 (gx, gy, gz)가 되려면
  // firmware raw payload에는 x/y를 반대로 넣어야 합니다.
  //
  //   raw slot x <- desired gy
  //   raw slot y <- desired gx
  //   raw slot z <- desired gz
  //
  const int16_t gx_raw_slot = saturate_to_i16(gy * kRadToDeg * config_.gyro_scale);
  const int16_t gy_raw_slot = saturate_to_i16(gx * kRadToDeg * config_.gyro_scale);
  const int16_t gz_raw_slot = saturate_to_i16(gz * kRadToDeg * config_.gyro_scale);

  write_i16_le(&frame, 0, gx_raw_slot);
  write_i16_le(&frame, 2, gy_raw_slot);
  write_i16_le(&frame, 4, gz_raw_slot);
  write_i16_le(&frame, 6, 0);

  return frame;
}

void E2BoxImuFirmware::normalize_quat_xyzw(
    double* qx,
    double* qy,
    double* qz,
    double* qw) {
  if (qx == nullptr || qy == nullptr || qz == nullptr || qw == nullptr) {
    return;
  }

  const double n = std::sqrt(
      (*qx) * (*qx) +
      (*qy) * (*qy) +
      (*qz) * (*qz) +
      (*qw) * (*qw));

  if (!std::isfinite(n) || n < 1e-12) {
    *qx = 0.0;
    *qy = 0.0;
    *qz = 0.0;
    *qw = 1.0;
    return;
  }

  *qx /= n;
  *qy /= n;
  *qz /= n;
  *qw /= n;
}

int16_t E2BoxImuFirmware::saturate_to_i16(double value) {
  if (!std::isfinite(value)) {
    return 0;
  }

  const double rounded = std::round(value);

  const double clamped = clamp_value(
      rounded,
      static_cast<double>(std::numeric_limits<int16_t>::min()),
      static_cast<double>(std::numeric_limits<int16_t>::max()));

  return static_cast<int16_t>(clamped);
}

void E2BoxImuFirmware::write_i16_le(
    CanFrame* frame,
    int offset,
    int16_t value) {
  if (frame == nullptr || offset < 0 || offset + 1 >= 8) {
    return;
  }

  const uint16_t u = static_cast<uint16_t>(value);

  frame->data[offset + 0] = static_cast<uint8_t>(u & 0xFF);
  frame->data[offset + 1] = static_cast<uint8_t>((u >> 8) & 0xFF);
}

}  // namespace mjcan