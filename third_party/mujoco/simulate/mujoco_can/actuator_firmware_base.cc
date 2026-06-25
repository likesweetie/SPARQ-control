#include "actuator_firmware_base.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace mjcan {

ActuatorFirmwareBase::ActuatorFirmwareBase() = default;

ActuatorFirmwareBase::~ActuatorFirmwareBase() = default;

bool ActuatorFirmwareBase::has_dlc_at_least(
    const CanFrame& frame,
    uint8_t required_dlc) {
  return frame.dlc >= required_dlc && frame.dlc <= frame.data.size();
}

uint16_t ActuatorFirmwareBase::read_u16_le(
    const CanFrame& frame,
    int offset) {
  if (offset < 0 || offset + 1 >= static_cast<int>(frame.data.size())) {
    return 0;
  }

  return static_cast<uint16_t>(
      static_cast<uint16_t>(frame.data[offset + 0]) |
      static_cast<uint16_t>(frame.data[offset + 1] << 8));
}

int16_t ActuatorFirmwareBase::read_i16_le(
    const CanFrame& frame,
    int offset) {
  return static_cast<int16_t>(read_u16_le(frame, offset));
}

uint32_t ActuatorFirmwareBase::read_u32_le(
    const CanFrame& frame,
    int offset) {
  if (offset < 0 || offset + 3 >= static_cast<int>(frame.data.size())) {
    return 0;
  }

  return static_cast<uint32_t>(
      static_cast<uint32_t>(frame.data[offset + 0]) |
      static_cast<uint32_t>(frame.data[offset + 1]) << 8 |
      static_cast<uint32_t>(frame.data[offset + 2]) << 16 |
      static_cast<uint32_t>(frame.data[offset + 3]) << 24);
}

int32_t ActuatorFirmwareBase::read_i32_le(
    const CanFrame& frame,
    int offset) {
  return static_cast<int32_t>(read_u32_le(frame, offset));
}

float ActuatorFirmwareBase::read_f32_le(
    const CanFrame& frame,
    int offset) {
  if (offset < 0 || offset + 3 >= static_cast<int>(frame.data.size())) {
    return 0.0f;
  }

  float value = 0.0f;
  const uint8_t bytes[4] = {
      frame.data[offset + 0],
      frame.data[offset + 1],
      frame.data[offset + 2],
      frame.data[offset + 3],
  };

  static_assert(sizeof(float) == 4, "float must be 4 bytes");
  std::memcpy(&value, bytes, sizeof(float));
  return value;
}

void ActuatorFirmwareBase::write_u16_le(
    CanFrame* frame,
    int offset,
    uint16_t value) {
  if (frame == nullptr ||
      offset < 0 ||
      offset + 1 >= static_cast<int>(frame->data.size())) {
    return;
  }

  frame->data[offset + 0] = static_cast<uint8_t>(value & 0xFF);
  frame->data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void ActuatorFirmwareBase::write_i16_le(
    CanFrame* frame,
    int offset,
    int16_t value) {
  write_u16_le(frame, offset, static_cast<uint16_t>(value));
}

void ActuatorFirmwareBase::write_u32_le(
    CanFrame* frame,
    int offset,
    uint32_t value) {
  if (frame == nullptr ||
      offset < 0 ||
      offset + 3 >= static_cast<int>(frame->data.size())) {
    return;
  }

  frame->data[offset + 0] = static_cast<uint8_t>(value & 0xFF);
  frame->data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  frame->data[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  frame->data[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

void ActuatorFirmwareBase::write_i32_le(
    CanFrame* frame,
    int offset,
    int32_t value) {
  write_u32_le(frame, offset, static_cast<uint32_t>(value));
}

void ActuatorFirmwareBase::write_f32_le(
    CanFrame* frame,
    int offset,
    float value) {
  if (frame == nullptr ||
      offset < 0 ||
      offset + 3 >= static_cast<int>(frame->data.size())) {
    return;
  }

  uint8_t bytes[4] = {};
  static_assert(sizeof(float) == 4, "float must be 4 bytes");
  std::memcpy(bytes, &value, sizeof(float));

  frame->data[offset + 0] = bytes[0];
  frame->data[offset + 1] = bytes[1];
  frame->data[offset + 2] = bytes[2];
  frame->data[offset + 3] = bytes[3];
}

int16_t ActuatorFirmwareBase::saturate_to_i16(double value) {
  const double rounded = std::round(value);

  const double clamped = std::clamp(
      rounded,
      static_cast<double>(std::numeric_limits<int16_t>::min()),
      static_cast<double>(std::numeric_limits<int16_t>::max()));

  return static_cast<int16_t>(clamped);
}

uint16_t ActuatorFirmwareBase::saturate_to_u16(double value) {
  const double rounded = std::round(value);

  const double clamped = std::clamp(
      rounded,
      static_cast<double>(std::numeric_limits<uint16_t>::min()),
      static_cast<double>(std::numeric_limits<uint16_t>::max()));

  return static_cast<uint16_t>(clamped);
}

CanFrame ActuatorFirmwareBase::make_empty_frame(
    uint32_t can_id,
    uint8_t dlc) {
  CanFrame frame;
  frame.can_id = can_id;
  frame.dlc = std::min<uint8_t>(dlc, 8);
  frame.data.fill(0);
  return frame;
}

}  // namespace mjcan