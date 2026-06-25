#include "spg_firmware.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <iomanip>

namespace mjcan {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

template <typename T>
T clamp_value(T value, T lo, T hi) {
  return std::max(lo, std::min(value, hi));
}

}  // namespace

SPGFirmware::SPGFirmware(
    uint32_t can_id,
    SPGMITConfig config)
    : can_id_(can_id),
      config_(config) {}

std::string SPGFirmware::name() const {
  return "SPGFirmware";
}

bool SPGFirmware::accepts(const CanFrame& frame) const {
  return frame.can_id == can_id_;
}

void SPGFirmware::reset(double sim_time) {
  enabled_ = false;
  fault_ = false;
  fault_code_ = 0;

  has_pending_command_ = false;
  pending_command_ = ActuatorCommand{};

  while (!pending_tx_frames_.empty()) {
    pending_tx_frames_.pop();
  }

  mit_status_response_pending_ = false;

  mit_zero_reference_rad_ = 0.0;
  zero_capture_pending_ = false;
  requested_zero_offset_rad_ = 0.0;

  ignore_control_until_time_ = sim_time;
}

void SPGFirmware::on_can_frame(
    const CanFrame& frame,
    double sim_time) {
  if (!accepts(frame) || !has_dlc_at_least(frame, 1)) {
    return;
  }

  const uint8_t opcode = frame.data[0];

  switch (opcode) {
    case kCmdMITControl:
      handle_mit_control(frame, sim_time);
      break;

    case kCmdMITEnter:
      handle_enter_motor_mode(sim_time);
      break;

    case kCmdMITExit:
      handle_exit_motor_mode(sim_time);
      break;

    case kCmdMITSetZero:
      handle_set_zero_position(frame, sim_time);
      break;

    case kCmdClearError:
      handle_clear_error(sim_time);
      break;

    default:
      // MIT 전용 firmware이므로 다른 RMD/SPG opcode는 무시합니다.
      break;
  }
}

bool SPGFirmware::has_pending_command() const {
  return has_pending_command_;
}

std::optional<ActuatorCommand> SPGFirmware::consume_pending_command() {
  if (!has_pending_command_) {
    return std::nullopt;
  }

  has_pending_command_ = false;
  return pending_command_;
}

std::vector<CanFrame> SPGFirmware::make_feedback_frames(
    const ActuatorFeedbackSample& sample,
    double sim_time) {
  std::vector<CanFrame> frames;

  update_zero_reference_if_needed(sample, sim_time);

  while (!pending_tx_frames_.empty()) {
    frames.push_back(pending_tx_frames_.front());
    pending_tx_frames_.pop();
  }

  if (mit_status_response_pending_) {
    frames.push_back(make_mit_status_frame(sample));
    mit_status_response_pending_ = false;
  }

  return frames;
}

void SPGFirmware::handle_mit_control(
    const CanFrame& frame,
    double sim_time) {
  if (!has_dlc_at_least(frame, 8)) {
    return;
  }

  // Set Zero 직후 잔류 command 무시.
  if (ignore_control_until_time_ >= 0.0 &&
      sim_time < ignore_control_until_time_) {
    return;
  }

  // 실제 펌웨어는 MIT mode 진입 전 control을 무시하는 쪽이 안전합니다.
  if (!enabled_ || fault_) {
    return;
  }

  const DecodedMITCommand decoded = decode_mit_control_frame(frame);

  pending_command_ = ActuatorCommand{};
  pending_command_.mode = ActuatorControlMode::kImpedance;
  pending_command_.position_rad =
      position_mit_to_physical_rad(decoded.p_des_rad);
  pending_command_.velocity_rad_s = decoded.v_des_rad_s;
  pending_command_.torque_nm = decoded.tau_ff_nm;
  pending_command_.kp = decoded.kp;
  pending_command_.kd = decoded.kd;
  pending_command_.last_update_time = sim_time;
  pending_command_.valid = true;
  pending_command_.enabled = true;

  // std::cout << std::fixed << std::setprecision(6);
  // std::cout << "MIT control received" << std::endl;
  // std::cout << "velocity_rad_s: " << decoded.v_des_rad_s << std::endl;
  // std::cout << "torque_nm: " << decoded.tau_ff_nm<< std::endl;
  // std::cout << "KP: " << decoded.kp << std::endl;
  // std::cout << "KD: " << decoded.kd << std::endl;

  // MIT control RX는 driver의 latched command를 교체합니다.
  // bridge는 이 command를 timeout으로 지우지 않고 exit/reset 전까지 적용합니다.
  has_pending_command_ = true;

  // 0xC0은 status response를 반환합니다.
  mit_status_response_pending_ = true;
}

void SPGFirmware::handle_enter_motor_mode(double sim_time) {
  enabled_ = true;

  // MIT enter는 zero reference를 바꾸지 않습니다.
  // 초기 feedback angle은 MuJoCo qpos에서 YAML sign/offset을 적용한 logical
  // joint angle과 같아야 합니다. Zero reference 변경은 0xC3 Set Zero만 담당합니다.

  pending_command_ = ActuatorCommand{};
  pending_command_.mode = ActuatorControlMode::kZeroTorque;
  pending_command_.position_rad = 0.0;
  pending_command_.velocity_rad_s = 0.0;
  pending_command_.torque_nm = 0.0;
  pending_command_.kp = 0.0;
  pending_command_.kd = 0.0;
  pending_command_.last_update_time = sim_time;
  pending_command_.valid = true;
  pending_command_.enabled = true;

  // 최초 MIT enable 직후에는 실제 MIT control RX가 오기 전까지 zero torque를
  // latched command로 유지합니다.
  has_pending_command_ = true;

  push_pending_tx(make_ack_frame(kCmdMITEnter));
}

void SPGFirmware::handle_exit_motor_mode(double sim_time) {
  enabled_ = false;

  pending_command_ = ActuatorCommand{};
  pending_command_.mode = ActuatorControlMode::kDisabled;
  pending_command_.position_rad = 0.0;
  pending_command_.velocity_rad_s = 0.0;
  pending_command_.torque_nm = 0.0;
  pending_command_.kp = 0.0;
  pending_command_.kd = 0.0;
  pending_command_.last_update_time = sim_time;
  pending_command_.valid = true;
  pending_command_.enabled = false;

  has_pending_command_ = true;

  push_pending_tx(make_ack_frame(kCmdMITExit));
}

void SPGFirmware::handle_set_zero_position(
    const CanFrame& frame,
    double sim_time) {
  if (!has_dlc_at_least(frame, 8)) {
    return;
  }

  const int16_t offset_count = read_i16_le(frame, 6);

  // 문서 기준:
  //   DATA[6:7] offset int16 LE
  //   0.01 deg / LSB
  //
  // offset=0이면 현재위치가 0도.
  // offset=3000이면 현재위치가 30도.
  requested_zero_offset_rad_ =
      static_cast<double>(offset_count) * 0.01 * kDegToRad;

  // 현재 위치 sample은 이 함수 시점에 없으므로 다음 sample 수신 시점에 적용합니다.
  zero_capture_pending_ = true;

  ignore_control_until_time_ = sim_time + config_.set_zero_hold_s;

  push_pending_tx(make_set_zero_ack_frame(offset_count));
}

void SPGFirmware::handle_clear_error(double sim_time) {
  fault_ = false;
  fault_code_ = 0;

  enabled_ = false;

  pending_command_ = ActuatorCommand{};
  pending_command_.mode = ActuatorControlMode::kDisabled;
  pending_command_.last_update_time = sim_time;
  pending_command_.valid = true;
  pending_command_.enabled = false;

  has_pending_command_ = true;

  push_pending_tx(make_clear_error_ack_frame());
}

SPGFirmware::DecodedMITCommand SPGFirmware::decode_mit_control_frame(
    const CanFrame& frame) const {
  DecodedMITCommand out;

  // MIT Parameter Encoding:
  //   data[1] = p_hi
  //   data[2] = p_lo
  //   data[3] = v_hi
  //   data[4] = v_lo4 | kp_hi4
  //   data[5] = kp_lo
  //   data[6] = kd
  //   data[7] = tff
  //
  // 문서의 bit layout:
  //   [p_hi(8)] [p_lo(8)] [v_hi(8)] [v_lo(4)|kp_hi(4)]
  //   [kp_lo(8)] [kd(8)] [tff(8)]
  const uint16_t p_uint =
      static_cast<uint16_t>(
          (static_cast<uint16_t>(frame.data[1]) << 8) |
          static_cast<uint16_t>(frame.data[2]));

  const uint16_t v_uint =
      static_cast<uint16_t>(
          (static_cast<uint16_t>(frame.data[3]) << 4) |
          static_cast<uint16_t>((frame.data[4] >> 4) & 0x0F));

  const uint16_t kp_uint =
      static_cast<uint16_t>(
          (static_cast<uint16_t>(frame.data[4] & 0x0F) << 8) |
          static_cast<uint16_t>(frame.data[5]));

  const uint8_t kd_uint = frame.data[6];
  const uint8_t tau_uint = frame.data[7];

  out.p_des_rad = uint_to_float(
      p_uint,
      -config_.p_max_rad,
      config_.p_max_rad,
      16);

  out.v_des_rad_s = uint_to_float(
      v_uint,
      -config_.v_max_rad_s,
      config_.v_max_rad_s,
      12);

  out.kp = uint_to_float(
      kp_uint,
      0.0,
      config_.kp_max,
      12);

  out.kd = uint_to_float(
      kd_uint,
      0.0,
      config_.kd_max,
      8);

  out.tau_ff_nm = uint_to_float(
      tau_uint,
      -config_.tau_max_nm,
      config_.tau_max_nm,
      8);

  return out;
}

double SPGFirmware::uint_to_float(
    uint32_t x,
    double x_min,
    double x_max,
    uint32_t bits) {
  const uint32_t max_int = (1u << bits) - 1u;
  const double span = x_max - x_min;
  return static_cast<double>(x) * span / static_cast<double>(max_int) + x_min;
}

uint32_t SPGFirmware::float_to_uint(
    double x,
    double x_min,
    double x_max,
    uint32_t bits) {
  const uint32_t max_int = (1u << bits) - 1u;
  const double clamped = clamp_value(x, x_min, x_max);
  const double span = x_max - x_min;
  const double normalized = (clamped - x_min) / span;
  return static_cast<uint32_t>(std::round(normalized * max_int));
}

CanFrame SPGFirmware::make_ack_frame(uint8_t opcode) const {
  CanFrame frame = make_empty_frame(can_id_, 8);
  frame.data[0] = opcode;
  return frame;
}

CanFrame SPGFirmware::make_set_zero_ack_frame(int16_t offset_count) const {
  CanFrame frame = make_empty_frame(can_id_, 8);
  frame.data[0] = kCmdMITSetZero;
  write_i16_le(&frame, 6, offset_count);
  return frame;
}

CanFrame SPGFirmware::make_clear_error_ack_frame() const {
  CanFrame frame = make_empty_frame(can_id_, 8);
  frame.data[0] = kCmdClearError;
  frame.data[1] = static_cast<uint8_t>(fault_code_);
  return frame;
}

CanFrame SPGFirmware::make_mit_status_frame(
    const ActuatorFeedbackSample& sample) const {
  CanFrame frame = make_empty_frame(can_id_, 8);

  const double p_mit_rad = position_physical_to_mit_rad(sample.position_rad);

  // RX 0xC0:
  //   DATA[0]   = 0xC0
  //   DATA[1]   = motor temperature int8
  //   DATA[2:3] = torque current Iq int16
  //   DATA[4:5] = speed out dps int16
  //   DATA[6:7] = signed int16 MIT feedback position, output-side MIT zero
  //               기준 ±12.56 rad
  //
  // 0xC0 is a v14 exception: DATA[6:7] is signed int16 MIT feedback
  // position, not 14-bit raw encoder.
  frame.data[0] = kCmdMITControl;

  const int temperature_i8 = clamp_value(
      static_cast<int>(std::round(sample.temperature_c)),
      -128,
      127);
  frame.data[1] = static_cast<uint8_t>(
      static_cast<int8_t>(temperature_i8));

  // 실제 current model이 없으면 torque를 tau_max 기준으로 Iq count에 근사 매핑합니다.
  const int16_t iq_count = saturate_to_i16(
      sample.torque_nm / config_.tau_max_nm * config_.iq_full_scale_count);
  write_i16_le(&frame, 2, iq_count);

  const int16_t speed_dps = saturate_to_i16(
      sample.velocity_rad_s * kRadToDeg);
  write_i16_le(&frame, 4, speed_dps);

  // v14 문서 기준: ±12.56 rad -> int16.
  const double p_clamped = clamp_value(
      p_mit_rad,
      -config_.feedback_position_max_rad,
      config_.feedback_position_max_rad);

  const double p_norm =
      p_clamped / config_.feedback_position_max_rad;

  const int16_t p_i16 = saturate_to_i16(
      p_norm * static_cast<double>(std::numeric_limits<int16_t>::max()));

  write_i16_le(&frame, 6, p_i16);

  return frame;
}

void SPGFirmware::push_pending_tx(const CanFrame& frame) {
  constexpr std::size_t kMaxPendingTxFrames = 32;

  if (pending_tx_frames_.size() >= kMaxPendingTxFrames) {
    pending_tx_frames_.pop();
  }

  pending_tx_frames_.push(frame);
}
void SPGFirmware::update_zero_reference_if_needed(
    const ActuatorFeedbackSample& sample,
    double /*sim_time*/) {
  if (!zero_capture_pending_) {
    return;
  }

  // 현재 physical position이 requested_zero_offset_rad_로 보이도록 reference 설정.
  //
  // p_mit = p_physical - zero_reference
  // p_mit = requested_zero_offset
  // therefore:
  // zero_reference = p_physical - requested_zero_offset
  mit_zero_reference_rad_ =
      sample.position_rad - requested_zero_offset_rad_;

  zero_capture_pending_ = false;
}

double SPGFirmware::position_physical_to_mit_rad(
    double physical_position_rad) const {
  return physical_position_rad - mit_zero_reference_rad_;
}

double SPGFirmware::position_mit_to_physical_rad(
    double mit_position_rad) const {
  return mit_position_rad + mit_zero_reference_rad_;
}

}  // namespace mjcan
