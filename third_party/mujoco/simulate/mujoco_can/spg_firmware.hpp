#pragma once

#include "actuator_firmware_base.hpp"

#include <cstdint>
#include <optional>
#include <queue>
#include <string>
#include <vector>

namespace mjcan {

struct SPGMITConfig {
  // MIT command normalization range.
  // 문서 기준 기본값:
  //   p_des: ±12.5 rad
  //   v_des: ±45 rad/s
  //   Kp:    0~500 Nm/rad
  //   Kd:    0~5 Nm*s/rad
  //   tau:   ±33 Nm
  double p_max_rad = 12.5;
  double v_max_rad_s = 45.0;
  double kp_max = 500.0;
  double kd_max = 5.0;
  double tau_max_nm = 33.0;

  // MIT feedback position range.
  // v14 문서 기준 0xC0 리턴 위치는 출력단 MIT zero 기준 ±12.56 rad를 int16에 매핑.
  double feedback_position_max_rad = 12.56;

  // Feedback current field:
  // 문서상 Torque Current(Iq)는 -2048~2048 = -33A~33A.
  // simulation에서는 실제 phase/current model이 없을 수 있으므로 torque_nm을 tau_max 기준으로
  // 정규화해서 iq_counts로 근사합니다.
  double iq_full_scale_count = 2048.0;
  double iq_full_scale_current_a = 33.0;

  // 0xC3 직후 잔류 명령 무시 시간.
  // 문서에는 Set Zero 직후 20 ms hold counter가 언급되어 있습니다.
  double set_zero_hold_s = 0.020;
};

class SPGFirmware final : public ActuatorFirmwareBase {
public:
  explicit SPGFirmware(
      uint32_t can_id,
      SPGMITConfig config = SPGMITConfig());

  ~SPGFirmware() override = default;

  std::string name() const override;

  bool accepts(const CanFrame& frame) const override;

  void reset(double sim_time) override;

  void on_can_frame(const CanFrame& frame, double sim_time) override;

  bool has_pending_command() const override;

  std::optional<ActuatorCommand> consume_pending_command() override;

  std::vector<CanFrame> make_feedback_frames(
      const ActuatorFeedbackSample& sample,
      double sim_time) override;

  bool enabled() const { return enabled_; }
  bool fault() const { return fault_; }
  int fault_code() const { return fault_code_; }

  double mit_zero_reference_rad() const { return mit_zero_reference_rad_; }

private:
  enum : uint8_t {
    kCmdMITControl = 0xC0,
    kCmdMITEnter = 0xC1,
    kCmdMITExit = 0xC2,
    kCmdMITSetZero = 0xC3,
    kCmdClearError = 0x9B,
  };

  struct DecodedMITCommand {
    double p_des_rad = 0.0;
    double v_des_rad_s = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double tau_ff_nm = 0.0;
  };

private:
  void handle_mit_control(const CanFrame& frame, double sim_time);
  void handle_enter_motor_mode(double sim_time);
  void handle_exit_motor_mode(double sim_time);
  void handle_set_zero_position(const CanFrame& frame, double sim_time);
  void handle_clear_error(double sim_time);

  DecodedMITCommand decode_mit_control_frame(const CanFrame& frame) const;

  static double uint_to_float(
      uint32_t x,
      double x_min,
      double x_max,
      uint32_t bits);

  static uint32_t float_to_uint(
      double x,
      double x_min,
      double x_max,
      uint32_t bits);

  CanFrame make_ack_frame(uint8_t opcode) const;
  CanFrame make_set_zero_ack_frame(int16_t offset_count) const;
  CanFrame make_clear_error_ack_frame() const;

  CanFrame make_mit_status_frame(
      const ActuatorFeedbackSample& sample) const;

  void push_pending_tx(const CanFrame& frame);

  void update_zero_reference_if_needed(
      const ActuatorFeedbackSample& sample,
      double sim_time);

  double position_physical_to_mit_rad(double physical_position_rad) const;
  double position_mit_to_physical_rad(double mit_position_rad) const;

private:
  uint32_t can_id_ = 0;
  SPGMITConfig config_;

  bool enabled_ = false;
  bool fault_ = false;
  int fault_code_ = 0;

  bool has_pending_command_ = false;
  ActuatorCommand pending_command_;

  std::queue<CanFrame> pending_tx_frames_;

  // 0xC0 수신 시 상태 응답을 다음 make_feedback_frames()에서 생성합니다.
  bool mit_status_response_pending_ = false;

  // MIT zero coordinate:
  //   p_mit = p_physical - mit_zero_reference_rad_
  //
  // MIT enter does not change this reference. Initial p_mit follows the
  // MuJoCo logical joint angle from ActuatorBinding sign/offset.
  // 0xC3 offset=0이면 현재 위치가 0이 되도록 reference를 잡습니다.
  // 0xC3 offset=3000이면 현재 위치가 +30deg가 되도록 reference를 잡습니다.
  double mit_zero_reference_rad_ = 0.0;

  bool zero_capture_pending_ = false;
  double requested_zero_offset_rad_ = 0.0;

  double ignore_control_until_time_ = -1.0;

};

}  // namespace mjcan
