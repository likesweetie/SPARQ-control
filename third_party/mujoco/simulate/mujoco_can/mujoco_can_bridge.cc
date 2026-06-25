#include "mujoco_can_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace mjcan {
namespace {

template <typename T>
T clamp_value(T value, T lo, T hi) {
  return std::max(lo, std::min(value, hi));
}

bool is_finite(double x) {
  return std::isfinite(x);
}

}  // namespace

// -----------------------------------------------------------------------------
// VirtualCanBus
// -----------------------------------------------------------------------------

void VirtualCanBus::push_host_frame(const CanFrame& frame) {
  host_to_device_.push_back(frame);
}

bool VirtualCanBus::pop_host_frame(CanFrame* frame) {
  if (host_to_device_.empty()) {
    return false;
  }

  if (frame != nullptr) {
    *frame = host_to_device_.front();
  }

  host_to_device_.pop_front();
  return true;
}

void VirtualCanBus::push_device_frame(const CanFrame& frame) {
  device_to_host_.push_back(frame);
}

bool VirtualCanBus::pop_device_frame(CanFrame* frame) {
  if (device_to_host_.empty()) {
    return false;
  }

  if (frame != nullptr) {
    *frame = device_to_host_.front();
  }

  device_to_host_.pop_front();
  return true;
}

void VirtualCanBus::clear() {
  host_to_device_.clear();
  device_to_host_.clear();
}

std::size_t VirtualCanBus::host_frame_count() const {
  return host_to_device_.size();
}

std::size_t VirtualCanBus::device_frame_count() const {
  return device_to_host_.size();
}

// -----------------------------------------------------------------------------
// VirtualActuatorDevice
// -----------------------------------------------------------------------------

MujocoCanBridge::VirtualActuatorDevice::VirtualActuatorDevice(
    int binding_index,
    ActuatorBinding binding,
    std::unique_ptr<ActuatorFirmwareBase> firmware)
    : binding_index_(binding_index),
      binding_(std::move(binding)),
      firmware_(std::move(firmware)) {}

MujocoCanBridge::VirtualActuatorDevice::~VirtualActuatorDevice() = default;

MujocoCanBridge::VirtualActuatorDevice::VirtualActuatorDevice(
    VirtualActuatorDevice&&) noexcept = default;

MujocoCanBridge::VirtualActuatorDevice&
MujocoCanBridge::VirtualActuatorDevice::operator=(
    VirtualActuatorDevice&&) noexcept = default;

bool MujocoCanBridge::VirtualActuatorDevice::accepts(
    const CanFrame& frame) const {
  return firmware_ && firmware_->accepts(frame);
}

void MujocoCanBridge::VirtualActuatorDevice::reset(double sim_time) {
  if (firmware_) {
    firmware_->reset(sim_time);
  }
}

void MujocoCanBridge::VirtualActuatorDevice::on_frame(
    const CanFrame& frame,
    CommandBuffer* command_buffer,
    double sim_time) {
  if (!firmware_ || command_buffer == nullptr) {
    return;
  }

  if (!firmware_->accepts(frame)) {
    return;
  }

  firmware_->on_can_frame(frame, sim_time);

  std::optional<ActuatorCommand> command =
      firmware_->consume_pending_command();

  if (!command.has_value()) {
    return;
  }

  if (binding_index_ < 0 ||
      binding_index_ >= static_cast<int>(command_buffer->actuator_commands.size())) {
    return;
  }

  command_buffer->actuator_commands[binding_index_] = *command;
}

void MujocoCanBridge::VirtualActuatorDevice::publish_feedback(
    const StateBuffer& state,
    const CommandBuffer& command_buffer,
    VirtualCanBus* bus,
    double sim_time) {
  if (!firmware_ || bus == nullptr) {
    return;
  }

  const ActuatorFeedbackSample sample =
      make_feedback_sample(state, command_buffer, sim_time);

  std::vector<CanFrame> frames =
      firmware_->make_feedback_frames(sample, sim_time);

  for (const CanFrame& frame : frames) {
    bus->push_device_frame(frame);
  }
}

ActuatorFeedbackSample
MujocoCanBridge::VirtualActuatorDevice::make_feedback_sample(
    const StateBuffer& state,
    const CommandBuffer& command_buffer,
    double sim_time) const {
  ActuatorFeedbackSample sample;
  sample.sim_time = sim_time;

  const bool qpos_ok =
      binding_.qpos_adr >= 0 &&
      binding_.qpos_adr < static_cast<int>(state.qpos.size());

  const bool qvel_ok =
      binding_.qvel_adr >= 0 &&
      binding_.qvel_adr < static_cast<int>(state.qvel.size());

  const bool actuator_force_ok =
      binding_.actuator_id >= 0 &&
      binding_.actuator_id < static_cast<int>(state.actuator_force.size());

  if (qpos_ok) {
    sample.position_rad =
        binding_.sign * (state.qpos[binding_.qpos_adr] - binding_.offset_rad);
  }

  if (qvel_ok) {
    sample.velocity_rad_s =
        binding_.sign * state.qvel[binding_.qvel_adr];
  }

  if (actuator_force_ok) {
    sample.torque_nm =
        binding_.sign * state.actuator_force[binding_.actuator_id];
  }

  if (binding_index_ >= 0 &&
      binding_index_ <
          static_cast<int>(command_buffer.actuator_commands.size())) {
    const ActuatorCommand& command =
        command_buffer.actuator_commands[binding_index_];

    sample.enabled = command.enabled;
  }

  // Virtual skeleton default telemetry.
  // 실제 motor thermal/current model을 붙이면 여기 값을 plant model에서 채우면 됩니다.
  sample.temperature_c = 30.0;
  sample.current_a = 0.0;
  sample.voltage_v = 0.0;
  sample.fault = false;
  sample.fault_code = 0;

  return sample;
}

// -----------------------------------------------------------------------------
// VirtualImuDevice
// -----------------------------------------------------------------------------

MujocoCanBridge::VirtualImuDevice::VirtualImuDevice(
    std::unique_ptr<ImuFirmwareBase> firmware)
    : firmware_(std::move(firmware)) {}

MujocoCanBridge::VirtualImuDevice::~VirtualImuDevice() = default;

MujocoCanBridge::VirtualImuDevice::VirtualImuDevice(
    VirtualImuDevice&&) noexcept = default;

MujocoCanBridge::VirtualImuDevice&
MujocoCanBridge::VirtualImuDevice::operator=(
    VirtualImuDevice&&) noexcept = default;

bool MujocoCanBridge::VirtualImuDevice::accepts(const CanFrame& frame) const {
  return firmware_ && firmware_->accepts(frame);
}

void MujocoCanBridge::VirtualImuDevice::reset(double sim_time) {
  if (firmware_) {
    firmware_->reset(sim_time);
  }
}

void MujocoCanBridge::VirtualImuDevice::on_frame(
    const CanFrame& frame,
    const ImuSample& sample,
    VirtualCanBus* bus,
    double sim_time) {
  if (!firmware_ || bus == nullptr) {
    return;
  }

  if (!firmware_->accepts(frame)) {
    return;
  }

  std::vector<CanFrame> frames =
      firmware_->on_can_frame(frame, sample, sim_time);

  for (const CanFrame& response : frames) {
    bus->push_device_frame(response);
  }
}

// -----------------------------------------------------------------------------
// MujocoCanBridge
// -----------------------------------------------------------------------------

MujocoCanBridge::MujocoCanBridge() = default;

MujocoCanBridge::~MujocoCanBridge() {
  shutdown();
}

void MujocoCanBridge::reset_model(const mjModel* model, mjData* data) {
  if (model == nullptr || data == nullptr) {
    initialized_ = false;
    return;
  }

  bus_.clear();
  command_buffer_.clear();
  actuator_bindings_.clear();
  actuators_.clear();
  imus_.clear();

  state_.sim_time = data->time;
  state_.nq = model->nq;
  state_.nv = model->nv;
  state_.nu = model->nu;
  state_.nbody = model->nbody;

  state_.qpos.assign(model->nq, 0.0);
  state_.qvel.assign(model->nv, 0.0);
  state_.ctrl.assign(model->nu, 0.0);
  state_.actuator_force.assign(model->nu, 0.0);

  resolve_base_imu_sensors(model);

  build_default_actuator_bindings(model);
  command_buffer_.actuator_commands.resize(actuator_bindings_.size());

  build_default_devices();

  for (auto& actuator : actuators_) {
    actuator.reset(data->time);
  }

  for (auto& imu : imus_) {
    imu.reset(data->time);
  }

  update_state_buffer(model, data);

  initialized_ = true;

  std::cout << "[MujocoCanBridge] reset_model\n"
            << "  nq=" << state_.nq
            << " nv=" << state_.nv
            << " nu=" << state_.nu
            << " nbody=" << state_.nbody << "\n"
            << "  actuator_bindings=" << actuator_bindings_.size() << "\n"
            << "  imu_quat_sensor=" << imu_quat_sensor_name_
            << " imu_gyro_sensor=" << imu_gyro_sensor_name_
            << " base_quat_sensor_id=" << base_quat_sensor_id_
            << " base_gyro_sensor_id=" << base_gyro_sensor_id_
            << "\n";
}

void MujocoCanBridge::before_step(const mjModel* model, mjData* data) {
  if (!initialized_ || model == nullptr || data == nullptr) {
    return;
  }

  // IMU request가 들어왔을 때 가능한 최신 state로 응답하기 위해 step 전에도 갱신합니다.
  update_state_buffer(model, data);

  process_host_frames(data->time);
  apply_commands_to_mujoco(model, data);
}

void MujocoCanBridge::after_step(const mjModel* model, mjData* data) {
  if (!initialized_ || model == nullptr || data == nullptr) {
    return;
  }

  update_state_buffer(model, data);
  publish_device_feedback(data->time);
}

void MujocoCanBridge::before_forward(const mjModel* model, mjData* data) {
  if (!initialized_ || model == nullptr || data == nullptr) {
    return;
  }

  update_state_buffer(model, data);

  process_host_frames(data->time);
  apply_commands_to_mujoco(model, data);
}

void MujocoCanBridge::after_forward(const mjModel* model, mjData* data) {
  if (!initialized_ || model == nullptr || data == nullptr) {
    return;
  }

  update_state_buffer(model, data);
  publish_device_feedback(data->time);
}

void MujocoCanBridge::shutdown() {
  bus_.clear();
  command_buffer_.clear();

  actuator_bindings_.clear();
  actuators_.clear();
  imus_.clear();

  initialized_ = false;
}

void MujocoCanBridge::push_host_frame(const CanFrame& frame) {
  bus_.push_host_frame(frame);
}

bool MujocoCanBridge::pop_device_frame(CanFrame* frame) {
  return bus_.pop_device_frame(frame);
}

std::size_t MujocoCanBridge::host_frame_count() const {
  return bus_.host_frame_count();
}

std::size_t MujocoCanBridge::device_frame_count() const {
  return bus_.device_frame_count();
}

void MujocoCanBridge::set_imu_sensor_names(
    const std::string& quat_sensor_name,
    const std::string& gyro_sensor_name) {
  imu_quat_sensor_name_ = quat_sensor_name;
  imu_gyro_sensor_name_ = gyro_sensor_name;
}

void MujocoCanBridge::set_spg_mit_config(const SPGMITConfig& config) {
  spg_mit_config_ = config;
}

void MujocoCanBridge::set_e2box_imu_config(
    const E2BoxImuFirmwareConfig& config) {
  e2box_imu_config_ = config;
}

void MujocoCanBridge::set_device_config(const DeviceConfig& config) {
  device_config_ = config;
}

int MujocoCanBridge::find_actuator_id_for_joint(
    const mjModel* model,
    int joint_id) const {
  if (model == nullptr || joint_id < 0 || joint_id >= model->njnt) {
    return -1;
  }

  for (int actuator_id = 0; actuator_id < model->nu; ++actuator_id) {
    const int trn_type = model->actuator_trntype[actuator_id];

    if (trn_type != mjTRN_JOINT && trn_type != mjTRN_JOINTINPARENT) {
      continue;
    }

    const int actuator_joint_id = model->actuator_trnid[2 * actuator_id + 0];

    if (actuator_joint_id == joint_id) {
      return actuator_id;
    }
  }

  return -1;
}

bool MujocoCanBridge::build_actuator_binding_from_config(
    const mjModel* model,
    const ActuatorDeviceConfig& config,
    ActuatorBinding* binding) const {
  if (model == nullptr || binding == nullptr || !config.enabled) {
    return false;
  }

  if (config.can_id == 0) {
    throw std::runtime_error(
        "[MujocoCanBridge] actuator config requires nonzero can_id");
  }

  int actuator_id = -1;
  int joint_id = -1;

  if (!config.mujoco_actuator_name.empty()) {
    actuator_id = mj_name2id(
        model,
        mjOBJ_ACTUATOR,
        config.mujoco_actuator_name.c_str());

    if (actuator_id < 0) {
      throw std::runtime_error(
          "[MujocoCanBridge] actuator not found: " +
          config.mujoco_actuator_name);
    }
  }

  if (!config.mujoco_joint_name.empty()) {
    joint_id = mj_name2id(
        model,
        mjOBJ_JOINT,
        config.mujoco_joint_name.c_str());

    if (joint_id < 0) {
      throw std::runtime_error(
          "[MujocoCanBridge] joint not found: " +
          config.mujoco_joint_name);
    }
  }

  if (actuator_id < 0 && joint_id >= 0) {
    actuator_id = find_actuator_id_for_joint(model, joint_id);

    if (actuator_id < 0) {
      throw std::runtime_error(
          "[MujocoCanBridge] no joint actuator found for joint: " +
          config.mujoco_joint_name);
    }
  }

  if (actuator_id >= 0 && joint_id < 0) {
    const int trn_type = model->actuator_trntype[actuator_id];

    if (trn_type != mjTRN_JOINT && trn_type != mjTRN_JOINTINPARENT) {
      throw std::runtime_error(
          "[MujocoCanBridge] actuator is not a joint actuator: " +
          config.mujoco_actuator_name);
    }

    joint_id = model->actuator_trnid[2 * actuator_id + 0];
  }

  if (actuator_id < 0 || joint_id < 0) {
    throw std::runtime_error(
        "[MujocoCanBridge] actuator config needs 'mujoco_actuator' or 'mujoco_joint'");
  }

  if (joint_id < 0 || joint_id >= model->njnt) {
    throw std::runtime_error("[MujocoCanBridge] joint ID is out of range");
  }

  if (model->jnt_type[joint_id] != mjJNT_HINGE) {
    throw std::runtime_error(
        "[MujocoCanBridge] only hinge joints are supported for MIT actuator binding: " +
        config.mujoco_joint_name);
  }

  ActuatorBinding out;
  out.can_id = config.can_id;

  out.joint_id = joint_id;
  out.actuator_id = actuator_id;
  out.ctrl_adr = actuator_id;
  out.qpos_adr = model->jnt_qposadr[joint_id];
  out.qvel_adr = model->jnt_dofadr[joint_id];

  out.mujoco_actuator_name = config.mujoco_actuator_name;
  out.mujoco_joint_name = config.mujoco_joint_name;

  out.logical_name = config.logical_name;

  out.sign = config.sign;
  out.offset_rad = config.offset_rad;
  out.spg_mit_config = config.spg_mit_config;

  *binding = std::move(out);
  return true;
}

void MujocoCanBridge::build_default_actuator_bindings(
    const mjModel* model) {
  if (model == nullptr) {
    throw std::runtime_error("[MujocoCanBridge] reset_model requires a model");
  }

  actuator_bindings_.clear();

  if (!device_config_.actuators.empty()) {
    actuator_bindings_.reserve(device_config_.actuators.size());

    for (const ActuatorDeviceConfig& config : device_config_.actuators) {
      ActuatorBinding binding;

      if (!build_actuator_binding_from_config(
              model,
              config,
              &binding)) {
        throw std::runtime_error(
            "[MujocoCanBridge] enabled actuator config did not produce a binding");
      }

      actuator_bindings_.push_back(std::move(binding));
    }

    if (actuator_bindings_.empty()) {
      throw std::runtime_error("[MujocoCanBridge] no enabled actuator CAN mappings configured");
    }

    return;
  }

  throw std::runtime_error("[MujocoCanBridge] no actuator CAN mappings configured");
}

void MujocoCanBridge::build_default_devices() {
  actuators_.clear();
  actuators_.reserve(actuator_bindings_.size());

  for (int i = 0; i < static_cast<int>(actuator_bindings_.size()); ++i) {
    const ActuatorBinding& binding = actuator_bindings_[i];

    auto firmware = std::make_unique<SPGFirmware>(
        binding.can_id,
        binding.spg_mit_config);

    actuators_.emplace_back(
        i,
        binding,
        std::move(firmware));
  }

  imus_.clear();

  if (device_config_.imus.empty()) {
    throw std::runtime_error("[MujocoCanBridge] no IMU CAN devices configured");
  }

  for (const ImuDeviceConfig& config : device_config_.imus) {
    if (!config.enabled) {
      continue;
    }

    if (config.type != "e2box" &&
        config.type != "E2Box" &&
        config.type != "E2BOX") {
      std::cerr << "[MujocoCanBridge] unsupported IMU firmware type: "
                << config.type << "\n";
      throw std::runtime_error(
          "[MujocoCanBridge] unsupported IMU firmware type: " + config.type);
    }

    auto imu_firmware =
        std::make_unique<E2BoxImuFirmware>(config.e2box_config);

    imus_.emplace_back(std::move(imu_firmware));
  }

  if (imus_.empty()) {
    throw std::runtime_error("[MujocoCanBridge] no enabled IMU CAN devices configured");
  }
}

void MujocoCanBridge::update_state_buffer(
    const mjModel* model,
    const mjData* data) {
  if (model == nullptr || data == nullptr) {
    return;
  }

  state_.sim_time = data->time;

  state_.nq = model->nq;
  state_.nv = model->nv;
  state_.nu = model->nu;
  state_.nbody = model->nbody;

  state_.qpos.assign(data->qpos, data->qpos + model->nq);
  state_.qvel.assign(data->qvel, data->qvel + model->nv);
  state_.ctrl.assign(data->ctrl, data->ctrl + model->nu);
  state_.actuator_force.assign(
      data->actuator_force,
      data->actuator_force + model->nu);

  if (base_quat_sensor_adr_ < 0 ||
      base_quat_sensor_adr_ + 3 >= model->nsensordata ||
      base_gyro_sensor_adr_ < 0 ||
      base_gyro_sensor_adr_ + 2 >= model->nsensordata) {
    throw std::runtime_error(
        "[MujocoCanBridge] required base IMU sensors are not resolved");
  }

  state_.base_quat_wxyz[0] = data->sensordata[base_quat_sensor_adr_ + 0];
  state_.base_quat_wxyz[1] = data->sensordata[base_quat_sensor_adr_ + 1];
  state_.base_quat_wxyz[2] = data->sensordata[base_quat_sensor_adr_ + 2];
  state_.base_quat_wxyz[3] = data->sensordata[base_quat_sensor_adr_ + 3];

  state_.base_gyro_xyz[0] = data->sensordata[base_gyro_sensor_adr_ + 0];
  state_.base_gyro_xyz[1] = data->sensordata[base_gyro_sensor_adr_ + 1];
  state_.base_gyro_xyz[2] = data->sensordata[base_gyro_sensor_adr_ + 2];
}

void MujocoCanBridge::process_host_frames(double sim_time) {
  CanFrame frame;

  constexpr int kMaxFramesPerStep = 4096;
  int processed = 0;

  while (bus_.pop_host_frame(&frame)) {
    bool accepted = false;

    for (auto& actuator : actuators_) {
      if (!actuator.accepts(frame)) {
        continue;
      }

      actuator.on_frame(
          frame,
          &command_buffer_,
          sim_time);

      accepted = true;
      break;
    }

    if (!accepted) {
      const ImuSample imu_sample = make_imu_sample();

      for (auto& imu : imus_) {
        if (!imu.accepts(frame)) {
          continue;
        }

        imu.on_frame(
            frame,
            imu_sample,
            &bus_,
            sim_time);

        accepted = true;
        break;
      }
    }

    ++processed;

    if (processed >= kMaxFramesPerStep) {
      std::cerr << "[MujocoCanBridge] host frame processing limit reached\n";
      break;
    }
  }
}

void MujocoCanBridge::apply_commands_to_mujoco(
    const mjModel* model,
    mjData* data) {
  if (model == nullptr || data == nullptr) {
    return;
  }

  for (int i = 0; i < static_cast<int>(actuator_bindings_.size()); ++i) {
    const ActuatorBinding& binding = actuator_bindings_[i];

    if (binding.ctrl_adr < 0 || binding.ctrl_adr >= model->nu) {
      continue;
    }

    double tau_logical = 0.0;

    if (i < static_cast<int>(command_buffer_.actuator_commands.size())) {
      const ActuatorCommand& command =
          command_buffer_.actuator_commands[i];

      // The virtual SPG MIT firmware latches the last accepted command.
      // Enter Motor Mode latches zero torque first; each MIT control RX frame
      // replaces that latched command until Exit Motor Mode or reset.
      if (command.valid && command.enabled) {
        const double q_logical =
            get_logical_position(binding, data);

        const double dq_logical =
            get_logical_velocity(binding, data);

        tau_logical =
            compute_logical_torque_command(
                command,
                q_logical,
                dq_logical);
      }
    }

    // logical torque를 MuJoCo actuator input 방향으로 변환합니다.
    double ctrl = binding.sign * tau_logical;

    ctrl = clamp_ctrl_if_needed(model, binding.ctrl_adr, ctrl);

    data->ctrl[binding.ctrl_adr] = ctrl;
  }
}

void MujocoCanBridge::publish_device_feedback(double sim_time) {
  for (auto& actuator : actuators_) {
    actuator.publish_feedback(
        state_,
        command_buffer_,
        &bus_,
        sim_time);
  }

  // E2Box IMU는 현재 request-response 방식입니다.
  // 따라서 periodic publish는 하지 않습니다.
  // IMU request frame이 들어오면 process_host_frames()에서 즉시 응답합니다.
}

void MujocoCanBridge::resolve_base_imu_sensors(const mjModel* model) {
  if (model == nullptr) {
    throw std::runtime_error("[MujocoCanBridge] cannot resolve IMU sensors without model");
  }

  base_quat_sensor_id_ = find_required_sensor_by_name(
      model,
      imu_quat_sensor_name_,
      mjSENS_FRAMEQUAT,
      4,
      "framequat");
  base_gyro_sensor_id_ = find_required_sensor_by_name(
      model,
      imu_gyro_sensor_name_,
      mjSENS_GYRO,
      3,
      "gyro");

  base_quat_sensor_adr_ = model->sensor_adr[base_quat_sensor_id_];
  base_gyro_sensor_adr_ = model->sensor_adr[base_gyro_sensor_id_];
}

int MujocoCanBridge::find_required_sensor_by_name(
    const mjModel* model,
    const std::string& sensor_name,
    int sensor_type,
    int sensor_dim,
    const char* label) const {
  if (model == nullptr) {
    throw std::runtime_error("[MujocoCanBridge] sensor lookup requires model");
  }

  if (sensor_name.empty()) {
    throw std::runtime_error(
        std::string("[MujocoCanBridge] missing required ") + label +
        " sensor name");
  }

  const int sensor_id =
      mj_name2id(model, mjOBJ_SENSOR, sensor_name.c_str());
  if (sensor_id < 0) {
    throw std::runtime_error(
        std::string("[MujocoCanBridge] missing required ") + label +
        " sensor: " + sensor_name);
  }

  if (model->sensor_type[sensor_id] != sensor_type) {
    throw std::runtime_error(
        std::string("[MujocoCanBridge] sensor '") + sensor_name +
        "' is not a " + label + " sensor");
  }

  if (model->sensor_dim[sensor_id] != sensor_dim) {
    throw std::runtime_error(
        std::string("[MujocoCanBridge] sensor '") + sensor_name +
        "' has invalid dimension for " + label);
  }

  return sensor_id;
}

ImuSample MujocoCanBridge::make_imu_sample() const {
  ImuSample sample;

  // MuJoCo xquat: w, x, y, z
  // Host-side E2Box protocol: x, y, z, w
  sample.quat_xyzw = {
      state_.base_quat_wxyz[1],
      state_.base_quat_wxyz[2],
      state_.base_quat_wxyz[3],
      state_.base_quat_wxyz[0],
  };

  sample.angular_velocity_rad_s = {
      state_.base_gyro_xyz[0],
      state_.base_gyro_xyz[1],
      state_.base_gyro_xyz[2],
  };

  sample.sim_time = state_.sim_time;

  return sample;
}

double MujocoCanBridge::get_logical_position(
    const ActuatorBinding& binding,
    const mjData* data) const {
  if (data == nullptr ||
      binding.qpos_adr < 0 ||
      binding.qpos_adr >= state_.nq) {
    return 0.0;
  }

  return binding.sign * (data->qpos[binding.qpos_adr] - binding.offset_rad);
}

double MujocoCanBridge::get_logical_velocity(
    const ActuatorBinding& binding,
    const mjData* data) const {
  if (data == nullptr ||
      binding.qvel_adr < 0 ||
      binding.qvel_adr >= state_.nv) {
    return 0.0;
  }

  return binding.sign * data->qvel[binding.qvel_adr];
}

double MujocoCanBridge::compute_logical_torque_command(
    const ActuatorCommand& command,
    double q_logical,
    double dq_logical) const {
  if (!command.valid || !command.enabled) {
    return 0.0;
  }

  switch (command.mode) {
    case ActuatorControlMode::kDisabled:
      return 0.0;

    case ActuatorControlMode::kZeroTorque:
      return 0.0;

    case ActuatorControlMode::kTorque:
      return command.torque_nm;

    case ActuatorControlMode::kDamping:
      return command.torque_nm +
             command.kd * (command.velocity_rad_s - dq_logical);

    case ActuatorControlMode::kImpedance:
      return command.kp * (command.position_rad - q_logical) +
             command.kd * (command.velocity_rad_s - dq_logical) +
             command.torque_nm;

    default:
      return 0.0;
  }
}

double MujocoCanBridge::clamp_ctrl_if_needed(
    const mjModel* model,
    int ctrl_adr,
    double ctrl) const {
  if (model == nullptr ||
      ctrl_adr < 0 ||
      ctrl_adr >= model->nu) {
    return 0.0;
  }

  if (!is_finite(ctrl)) {
    static bool warned_nonfinite = false;
    if (!warned_nonfinite) {
      std::cerr << "[MujocoCanBridge] non-finite ctrl replaced with zero\n";
      warned_nonfinite = true;
    }
    return 0.0;
  }

  if (model->actuator_ctrllimited != nullptr &&
      model->actuator_ctrllimited[ctrl_adr]) {
    const double lo = model->actuator_ctrlrange[2 * ctrl_adr + 0];
    const double hi = model->actuator_ctrlrange[2 * ctrl_adr + 1];

    if (ctrl < lo || ctrl > hi) {
      static bool warned_clamped = false;
      if (!warned_clamped) {
        std::cerr << "[MujocoCanBridge] ctrl clamped to MJCF actuator_ctrlrange\n";
        warned_clamped = true;
      }
    }

    return clamp_value(ctrl, lo, hi);
  }

  return ctrl;
}

}  // namespace mjcan
