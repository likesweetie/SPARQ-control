#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mjcan {

// -----------------------------------------------------------------------------
// Classical CAN frame
// -----------------------------------------------------------------------------
//
// 현재는 Classical CAN 기준 8-byte payload만 지원합니다.
// CAN FD가 필요해지면 data size와 dlc handling을 확장하면 됩니다.
//
struct CanFrame {
  uint32_t can_id = 0;
  uint8_t dlc = 0;
  std::array<uint8_t, 8> data{};
};

// -----------------------------------------------------------------------------
// Product-independent actuator command
// -----------------------------------------------------------------------------
//
// 펌웨어가 CAN frame을 해석한 뒤 bridge에 넘기는 공통 명령 표현입니다.
// 실제 MuJoCo data->ctrl 적용은 MujocoCanBridge 또는 actuator plant layer에서 합니다.
//
enum class ActuatorControlMode {
  kDisabled = 0,
  kZeroTorque,
  kTorque,
  kDamping,
  kImpedance,
};

struct ActuatorCommand {
  ActuatorControlMode mode = ActuatorControlMode::kDisabled;

  double position_rad = 0.0;
  double velocity_rad_s = 0.0;
  double torque_nm = 0.0;

  double kp = 0.0;
  double kd = 0.0;

  double last_update_time = -1.0;

  bool valid = false;
  bool enabled = false;
};

// -----------------------------------------------------------------------------
// Product-independent actuator feedback sample
// -----------------------------------------------------------------------------
//
// MuJoCo 상태에서 읽은 값을 제품 독립 단위로 정리한 샘플입니다.
// VirtualActuatorDevice가 StateBuffer + ActuatorBinding으로부터 이 샘플을 만들고,
// 펌웨어는 이 샘플을 자기 제품의 CAN feedback payload로 encode합니다.
//
struct ActuatorFeedbackSample {
  double position_rad = 0.0;
  double velocity_rad_s = 0.0;
  double torque_nm = 0.0;

  // 선택 필드입니다. 실제 모델에서 계산하지 않으면 0 또는 NaN 정책을 따로 정하면 됩니다.
  double current_a = 0.0;
  double temperature_c = 0.0;
  double voltage_v = 0.0;

  double sim_time = 0.0;

  bool enabled = false;
  bool fault = false;
  int fault_code = 0;
};

// -----------------------------------------------------------------------------
// Actuator firmware base class
// -----------------------------------------------------------------------------
//
// 이 클래스의 의도:
//   - MuJoCo를 모른다.
//   - mjModel, mjData, qpos_adr, ctrl_adr을 모른다.
//   - CAN frame decode/encode, mode state, enable/fault state만 담당한다.
//
// VirtualActuatorDevice의 의도:
//   - MuJoCo binding을 안다.
//   - StateBuffer에서 ActuatorFeedbackSample을 만든다.
//   - firmware가 만든 ActuatorCommand를 CommandBuffer에 반영한다.
//
class ActuatorFirmwareBase {
public:
  virtual ~ActuatorFirmwareBase();

  ActuatorFirmwareBase(const ActuatorFirmwareBase&) = delete;
  ActuatorFirmwareBase& operator=(const ActuatorFirmwareBase&) = delete;

  ActuatorFirmwareBase(ActuatorFirmwareBase&&) = delete;
  ActuatorFirmwareBase& operator=(ActuatorFirmwareBase&&) = delete;

  // Debug/logging용 이름입니다.
  virtual std::string name() const = 0;

  // 이 펌웨어가 특정 CAN frame을 처리할 수 있는지 판단합니다.
  // command_id 하나만 받는 제품도 있고, 여러 CAN ID를 쓰는 제품도 있으므로
  // 단순 command_id getter보다 accepts()가 더 유연합니다.
  virtual bool accepts(const CanFrame& frame) const = 0;

  // 모델 reset, simulation reset, device power-cycle 등에 사용합니다.
  virtual void reset(double sim_time) = 0;

  // Host에서 device로 들어온 CAN frame을 처리합니다.
  //
  // 이 함수 안에서:
  //   - opcode decode
  //   - enable/disable 상태 갱신
  //   - fault clear
  //   - command latch
  //   - pending command 생성
  //
  // 등을 수행합니다.
  virtual void on_can_frame(const CanFrame& frame, double sim_time) = 0;

  // 새 actuator command가 준비되었는지 확인합니다.
  virtual bool has_pending_command() const = 0;

  // 새 actuator command를 꺼냅니다.
  //
  // 권장 정책:
  //   - command를 반환한 뒤 pending flag를 false로 내립니다.
  //   - pending command가 없으면 std::nullopt를 반환합니다.
  virtual std::optional<ActuatorCommand> consume_pending_command() = 0;

  // 현재 actuator 상태 샘플을 제품별 CAN feedback frame으로 encode합니다.
  //
  // 일부 펌웨어는 주기적으로 feedback을 보냅니다.
  // 일부 펌웨어는 request-response 방식으로만 feedback을 보냅니다.
  // 그래서 반환 타입은 vector로 둡니다.
  virtual std::vector<CanFrame> make_feedback_frames(
      const ActuatorFeedbackSample& sample,
      double sim_time) = 0;

protected:
  ActuatorFirmwareBase();

  // ---------------------------------------------------------------------------
  // Common byte helpers
  // ---------------------------------------------------------------------------
  //
  // 제품별 firmware 구현에서 자주 쓰는 little-endian packing helper입니다.
  // 이 helper들은 protocol logic이 아니라 byte utility이므로 base에 두어도 됩니다.
  //

  static bool has_dlc_at_least(const CanFrame& frame, uint8_t required_dlc);

  static uint16_t read_u16_le(const CanFrame& frame, int offset);
  static int16_t read_i16_le(const CanFrame& frame, int offset);
  static uint32_t read_u32_le(const CanFrame& frame, int offset);
  static int32_t read_i32_le(const CanFrame& frame, int offset);
  static float read_f32_le(const CanFrame& frame, int offset);

  static void write_u16_le(CanFrame* frame, int offset, uint16_t value);
  static void write_i16_le(CanFrame* frame, int offset, int16_t value);
  static void write_u32_le(CanFrame* frame, int offset, uint32_t value);
  static void write_i32_le(CanFrame* frame, int offset, int32_t value);
  static void write_f32_le(CanFrame* frame, int offset, float value);

  static int16_t saturate_to_i16(double value);
  static uint16_t saturate_to_u16(double value);

  static CanFrame make_empty_frame(uint32_t can_id, uint8_t dlc);
};

}  // namespace mjcan