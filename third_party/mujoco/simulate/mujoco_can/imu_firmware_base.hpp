#pragma once

#include "actuator_firmware_base.hpp"

#include <array>
#include <string>
#include <vector>

namespace mjcan {

struct ImuSample {
  // Host-side Python protocol convention:
  //   quat_xyzw = qx, qy, qz, qw
  std::array<double, 4> quat_xyzw{0.0, 0.0, 0.0, 1.0};

  // 이 값은 Python decode 결과로 얻고 싶은 최종 angular velocity와 같은 convention이어야 합니다.
  // 즉, firmware 내부에서 E2Box raw convention으로 변환해 pack합니다.
  std::array<double, 3> angular_velocity_rad_s{0.0, 0.0, 0.0};

  double sim_time = 0.0;
};

class ImuFirmwareBase {
public:
  virtual ~ImuFirmwareBase() = default;

  ImuFirmwareBase(const ImuFirmwareBase&) = delete;
  ImuFirmwareBase& operator=(const ImuFirmwareBase&) = delete;

  ImuFirmwareBase(ImuFirmwareBase&&) = delete;
  ImuFirmwareBase& operator=(ImuFirmwareBase&&) = delete;

  virtual std::string name() const = 0;

  virtual bool accepts(const CanFrame& frame) const = 0;

  virtual void reset(double sim_time) = 0;

  // Request-response 방식 IMU를 기본으로 둡니다.
  // frame 하나를 받으면, sample을 제품별 protocol로 encode하여 0개 이상의 response frame을 반환합니다.
  virtual std::vector<CanFrame> on_can_frame(
      const CanFrame& frame,
      const ImuSample& sample,
      double sim_time) = 0;

protected:
  ImuFirmwareBase() = default;
};

}  // namespace mjcan