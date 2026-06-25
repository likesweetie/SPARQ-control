const $ = (id) => document.getElementById(id);

let latestState = null;
let ws = null;
let dashboardConfig = null;

const SAFETY_STATES = [
  "DISABLED",
  "ENABLING",
  "NORMAL",
  "DAMPING",
  "ZERO_SETTING",
  "ESTOP",
];

const fmt = {
  fixed(value, digits = 1, fallback = "-") {
    return Number.isFinite(value) ? value.toFixed(digits) : fallback;
  },
  age(value) {
    if (value === null || value === undefined) return "never";
    if (value < 1) return `${(value * 1000).toFixed(0)} ms`;
    return `${value.toFixed(2)} s`;
  },
  vector(values) {
    if (!Array.isArray(values)) return "-";
    return values.map((value) => this.fixed(value, 3)).join(", ");
  },
  maybe(value, digits = 2) {
    if (value === null || value === undefined) return "-";
    if (typeof value === "number") return value.toFixed(digits);
    return String(value);
  },
};

function badgeClass(status) {
  if (status === "online" || status === "connected") return "badge ok";
  if (status === "timeout" || status === "disconnected" || status === "error") return "badge danger";
  if (status === "stale") return "badge warn";
  return "badge muted";
}

function dotClass(status) {
  if (status === "online") return "status-dot online";
  if (status === "timeout") return "status-dot timeout";
  if (status === "stale") return "status-dot stale";
  return "status-dot";
}

function setText(id, value) {
  const el = $(id);
  if (el) el.textContent = value;
}

function render(state) {
  latestState = state;
  renderTopbar(state);
  if ($("loadPercent")) renderBus(state);
  if ($("safetyMachine")) renderSafetyMachine(state);
  if ($("robotControllerBadge")) renderImu(state);
  if ($("policyCommandPanel")) renderCurrentCommand(state.current_command || {});
  if ($("nodeRows")) renderNodes(state.nodes || []);
  if ($("motorRows")) renderMotors(state.motors || [], state.safety || {}, controllerSafetyState(state));
  if ($("enabledMotorCards")) renderEnabledMotorCards(state.motors || []);
  if ($("recentFrames")) renderFrames(state.recent_frames || []);
  if ($("processTiles")) renderProcesses(state.processes || []);
  if ($("shmStatusBadge")) renderShm(state);
  if ($("armButton") || $("runButton") || $("dampingButton") || $("faultClearButton") || $("estopButton")) syncOperatorControls(state);
  syncControls(state);
  syncZeroSetControls(state);
}

function hexCanId(value) {
  if (typeof value === "number") return `0x${value.toString(16).toUpperCase()}`;
  return String(value);
}

function normalizePayload(value) {
  return String(value || "").replaceAll(" ", "").toUpperCase();
}

function renderTopbar(state) {
  if ($("ifaceBadge")) $("ifaceBadge").textContent = state.can.iface;
  if ($("socketBadge")) {
    $("socketBadge").textContent = state.can.socket_status;
    $("socketBadge").className = badgeClass(state.can.socket_status);
  }
  if ($("txBadge")) {
    $("txBadge").textContent = state.safety.tx_enabled ? "TX ENABLED" : "TX LOCKED";
    $("txBadge").className = state.safety.tx_enabled ? "badge warn" : "badge danger";
  }
}

function renderBus(state) {
  const load = Math.max(0, state.can.load_percent || 0);
  const width = Math.min(load, 100);
  setText("loadPercent", `${fmt.fixed(load, 1)}%`);
  setText("rxRate", `${fmt.fixed(state.can.rx_rate, 1)} fps`);
  setText("txRate", `${fmt.fixed(state.can.tx_rate, 1)} fps`);
  setText("kbps", `${fmt.fixed(state.can.estimated_kbps, 1)} kbps`);
  setText("totals", `${state.can.total_rx} / ${state.can.total_tx}`);
  $("socketError").textContent = state.can.socket_error || "";

  const bar = $("loadBar");
  bar.style.width = `${width}%`;
  bar.className = "progress-fill";
  if (load >= 75) bar.classList.add("danger");
  else if (load >= 45) bar.classList.add("warn");
}

function safetyTone(stateName) {
  if (stateName === "NORMAL") return "ok";
  if (stateName === "DAMPING" || stateName === "ENABLING" || stateName === "ZERO_SETTING") return "warn";
  if (stateName === "FAULT_LATCHED" || stateName === "ESTOP") return "danger";
  if (stateName === "DISABLED" || stateName === "DISARMED" || stateName === "STOPPED" || stateName === "CREATED") return "muted";
  return "muted";
}

function normalizeSafetyStateName(value) {
  const normalized = String(value || "UNKNOWN")
    .trim()
    .toUpperCase()
    .replace(/[^A-Z0-9]+/g, "_")
    .replace(/^_+|_+$/g, "");
  return normalized || "UNKNOWN";
}

function renderSafetyMachine(state) {
  const controller = state.robot_controller || {};
  const current = controllerSafetyState(state);
  const action = controller.control_action || (state.safety && state.safety.action) || "-";
  const reason = controller.safety_reason || (state.safety && state.safety.reason) || "-";
  const fault = controller.fault_code || (state.safety && state.safety.fault_code) || "-";
  const tone = safetyTone(current);

  $("machineStateBadge").textContent = current.toLowerCase();
  $("machineStateBadge").className = `badge ${tone}`;
  setText("machineControllerState", controller.controller_state || "-");
  setText("machineAction", action);
  setText("machineFault", fault || "-");
  setText("machineAge", fmt.age(controller.age_s));
  setText("machineReason", reason || "-");

  $("safetyMachine").innerHTML = SAFETY_STATES.map((stateName) => {
    const active = stateName === current;
    const itemTone = active ? safetyTone(stateName) : "idle";
    const label = stateName.replaceAll("_", " ");
    return `
      <div class="state-block ${active ? "active" : ""} ${itemTone}" data-state="${stateName}">
        <span></span>
        <strong>${escapeHtml(label)}</strong>
      </div>
    `;
  }).join("");
}

function controllerSafetyState(state) {
  const controller = state.robot_controller || {};
  return normalizeSafetyStateName(controller.safety_state || (state.safety && state.safety.state));
}

function enableBlockedByControllerState(stateName) {
  return (
    stateName === "CREATED"
    || stateName === "DISARMED"
    || stateName === "FAULT_LATCHED"
    || stateName === "ESTOP"
    || stateName === "SHUTTING_DOWN"
    || stateName === "STOPPED"
    || stateName === "UNKNOWN"
  );
}

function operatorEstopDampingActive(state) {
  const controller = state.robot_controller || {};
  const reason = String(controller.safety_reason || (state.safety && state.safety.reason) || "").toUpperCase();
  return controllerSafetyState(state) === "DAMPING" && reason.includes("E-STOP");
}

function renderImu(state) {
  const imu = state.imu || {};
  const controller = state.robot_controller || {};
  const controllerStatus = controller.status || "unknown";
  $("robotControllerBadge").textContent = controller.controller_state
    ? `${controller.controller_state.toLowerCase()} · ${controllerStatus}`
    : controllerStatus;
  $("robotControllerBadge").className = badgeClass(controllerStatus);
  const quatAge = imu.quat_age_s === null || imu.quat_age_s === undefined ? Infinity : imu.quat_age_s;
  const gyroAge = imu.gyro_age_s === null || imu.gyro_age_s === undefined ? Infinity : imu.gyro_age_s;
  const maxAge = Math.max(quatAge, gyroAge);
  const fresh = Number.isFinite(maxAge) && maxAge <= 0.25;
  $("imuFreshBadge").textContent = fresh ? "online" : maxAge === Infinity ? "never" : "timeout";
  $("imuFreshBadge").className = fresh ? "badge ok" : maxAge === Infinity ? "badge muted" : "badge danger";
  setText("quatValue", fmt.vector(imu.quat_xyzw));
  setText("gravityValue", fmt.vector(imu.projected_gravity_b));
  setText("gyroValue", fmt.vector(imu.angular_velocity_rad_s));
  renderRobotAttitude(imu.quat_xyzw);
  setText("imuReq", imu.req_count || 0);
  setText("imuQuat", imu.quat_count || 0);
  setText("imuGyro", imu.gyro_count || 0);
  setText("imuAge", fmt.age(maxAge === Infinity ? null : maxAge));
}

function normalizeQuat(quat) {
  if (!Array.isArray(quat) || quat.length < 4) return null;
  const [x, y, z, w] = quat.map(Number);
  const norm = Math.hypot(x, y, z, w);
  if (!Number.isFinite(norm) || norm <= 1e-9) return null;
  return { x: x / norm, y: y / norm, z: z / norm, w: w / norm };
}

function quatToEulerDeg(quat) {
  const sinrCosp = 2 * (quat.w * quat.x + quat.y * quat.z);
  const cosrCosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
  const roll = Math.atan2(sinrCosp, cosrCosp);

  const sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
  const pitch = Math.abs(sinp) >= 1
    ? Math.sign(sinp) * Math.PI / 2
    : Math.asin(sinp);

  const sinyCosp = 2 * (quat.w * quat.z + quat.x * quat.y);
  const cosyCosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);

  const toDeg = 180 / Math.PI;
  return { roll: roll * toDeg, pitch: pitch * toDeg, yaw: yaw * toDeg };
}

function quatToCssMatrix3d(quat) {
  const { x, y, z, w } = quat;
  const xx = x * x;
  const yy = y * y;
  const zz = z * z;
  const xy = x * y;
  const xz = x * z;
  const yz = y * z;
  const wx = w * x;
  const wy = w * y;
  const wz = w * z;

  const m11 = 1 - 2 * (yy + zz);
  const m12 = 2 * (xy - wz);
  const m13 = 2 * (xz + wy);
  const m21 = 2 * (xy + wz);
  const m22 = 1 - 2 * (xx + zz);
  const m23 = 2 * (yz - wx);
  const m31 = 2 * (xz - wy);
  const m32 = 2 * (yz + wx);
  const m33 = 1 - 2 * (xx + yy);

  return [
    m11, m21, m31, 0,
    m12, m22, m32, 0,
    m13, m23, m33, 0,
    0, 0, 0, 1,
  ].map((value) => value.toFixed(6)).join(",");
}

function renderRobotAttitude(quatValues) {
  const body = $("robotAttitudeBody");
  const quat = normalizeQuat(quatValues);
  if (!quat) {
    body.style.transform = "translate(-50%, -50%)";
    setText("rollValue", "-");
    setText("pitchValue", "-");
    setText("yawValue", "-");
    return;
  }

  const euler = quatToEulerDeg(quat);
  body.style.transform = `translate(-50%, -50%) matrix3d(${quatToCssMatrix3d(quat)})`;
  setText("rollValue", `${euler.roll.toFixed(1)} deg`);
  setText("pitchValue", `${euler.pitch.toFixed(1)} deg`);
  setText("yawValue", `${euler.yaw.toFixed(1)} deg`);
}

function renderNodes(nodes) {
  const rows = nodes.map((node) => `
    <tr>
      <td>${escapeHtml(node.name)}</td>
      <td><code>${node.can_id}</code></td>
      <td>${escapeHtml(node.role)}</td>
      <td>${fmt.fixed(node.heartbeat_hz, 1)}</td>
      <td>${fmt.age(node.last_seen_s)}</td>
      <td><span class="${dotClass(node.status)}">${node.status}</span></td>
    </tr>
  `);
  $("nodeRows").innerHTML = rows.join("");
}

function renderMotors(motors, safety, controllerState) {
  const motorGated = !safety.tx_enabled || !safety.allow_actuator_commands;
  const enableBlocked = enableBlockedByControllerState(controllerState)
    || operatorEstopDampingActive(latestState || {});
  const seen = new Set();
  const tbody = $("motorRows");
  syncAllActuatorToggle(motors, motorGated, enableBlocked, controllerState);

  motors.forEach((motor) => {
    const key = String(motor.can_id);
    seen.add(key);

    let row = tbody.querySelector(`tr[data-can-id="${key}"]`);
    if (!row) {
      row = document.createElement("tr");
      row.dataset.canId = key;
      row.innerHTML = `
        <td data-field="name"></td>
        <td><code data-field="can_id"></code></td>
        <td><span data-field="last_kind"></span></td>
        <td data-field="age"></td>
        <td data-field="temperature"></td>
        <td data-field="iq"></td>
        <td data-field="speed"></td>
        <td data-field="position"></td>
        <td><code data-field="raw"></code></td>
        <td>
          <div class="row-actions">
            <button class="button secondary compact" data-motor-action="toggle-enable" data-can-id="${key}">Enable</button>
            <button class="button secondary compact" data-motor-action="zero" data-can-id="${key}">Zero set</button>
            <button class="button secondary compact" data-motor-action="mit-poll" data-can-id="${key}">MIT Poll</button>
          </div>
        </td>
      `;
      tbody.appendChild(row);
    }

    row.querySelector('[data-field="name"]').textContent = motor.name || motor.can_id;
    row.querySelector('[data-field="can_id"]').textContent = motor.can_id;
    const stateCell = row.querySelector('[data-field="last_kind"]');
    stateCell.className = dotClass(motor.status);
    stateCell.textContent = motor.last_kind;
    row.querySelector('[data-field="age"]').textContent = fmt.age(motor.age_s);
    row.querySelector('[data-field="temperature"]').textContent = fmt.maybe(motor.temperature_c, 0);
    row.querySelector('[data-field="iq"]').textContent = fmt.maybe(motor.current_a ?? motor.iq_a_approx, 2);
    row.querySelector('[data-field="speed"]').textContent = fmt.maybe(motor.speed_dps, 0);
    row.querySelector('[data-field="position"]').textContent = fmt.maybe(motor.position_rad, 3);
    row.querySelector('[data-field="raw"]').textContent = motor.raw || "";

    row.querySelectorAll("[data-motor-action]").forEach((button) => {
      if (button.dataset.motorAction === "toggle-enable") {
        const enabled = motor.enabled_hint === true;
        button.textContent = enabled ? "Disable" : "Enable";
        button.dataset.nextAction = enabled ? "exit" : "enter";
        button.classList.toggle("danger", enabled);
        button.classList.toggle("secondary", !enabled);
        button.disabled = !enabled && enableBlocked;
      }
      if (button.dataset.motorAction === "mit-poll") {
        button.textContent = motor.mit_polling ? "Stop Poll" : "MIT Poll";
        button.classList.toggle("danger", motor.mit_polling);
        button.classList.toggle("secondary", !motor.mit_polling);
      }
      if (button.dataset.motorAction === "zero") {
        button.disabled = controllerState !== "NORMAL";
      }
      const blockedEnable = button.dataset.motorAction === "toggle-enable"
        && button.dataset.nextAction === "enter"
        && enableBlocked;
      const isZeroAction = button.dataset.motorAction === "zero";
      const blockedZero = isZeroAction
        && controllerState !== "NORMAL";
      button.classList.toggle("gated", (!isZeroAction && motorGated) || blockedEnable || blockedZero);
      button.title = blockedEnable
        ? `Enable blocked while controller is ${controllerState}`
      : blockedZero ? "Zero set is available only while controller is NORMAL"
        : (!isZeroAction && motorGated) ? "Requires TX unlock and allow_actuator_commands=true" : "";
    });
  });

  tbody.querySelectorAll("tr[data-can-id]").forEach((row) => {
    if (!seen.has(row.dataset.canId)) {
      row.remove();
    }
  });
}

function syncAllActuatorToggle(motors, gated, enableBlocked, controllerState) {
  const button = $("allActuatorToggle");
  if (!button) return;
  const configured = Array.isArray(motors) ? motors : [];
  const anyEnabled = configured.some((motor) => motor.enabled_hint === true);
  const allEnabled = configured.length > 0 && configured.every((motor) => motor.enabled_hint === true);
  const shouldDisable = anyEnabled || allEnabled;
  button.textContent = shouldDisable ? "Disable All" : "Enable All";
  button.dataset.nextAction = shouldDisable ? "exit" : "enter";
  button.className = shouldDisable ? "button danger" : "button secondary";
  const blockedEnable = !shouldDisable && enableBlocked;
  button.classList.toggle("gated", gated || blockedEnable || configured.length === 0);
  button.disabled = configured.length === 0 || blockedEnable;
  button.title = blockedEnable
    ? `Enable all blocked while controller is ${controllerState}`
    : gated ? "Requires TX unlock and allow_actuator_commands=true" : "";
}

function renderEnabledMotorCards(motors) {
  const enabled = motors
    .filter((motor) => motor.enabled_hint === true)
    .sort((a, b) => String(a.can_id).localeCompare(String(b.can_id)))
    .slice(0, 3);

  const container = $("enabledMotorCards");
  if (!enabled.length) {
    container.innerHTML = "";
    container.hidden = true;
    return;
  }

  container.hidden = false;
  container.innerHTML = enabled.map((motor) => {
    const positionRad = Number.isFinite(motor.position_rad) ? motor.position_rad : 0;
    const positionDeg = positionRad * 180 / Math.PI;
    const dialDeg = positionDeg - 90;
    const hasPosition = Number.isFinite(motor.position_rad);
    return `
      <article class="enabled-motor-card">
        <div class="enabled-motor-copy">
          <span class="badge ok">${escapeHtml(motor.can_id)}</span>
          <strong>${escapeHtml(motor.name || motor.can_id)}</strong>
          <span>${escapeHtml(motor.last_kind)}</span>
        </div>
        <div class="angle-dial ${hasPosition ? "" : "muted"}" style="--angle:${dialDeg.toFixed(2)}deg">
          <span class="dial-arrow"></span>
          <span class="dial-center"></span>
        </div>
        <div class="enabled-motor-metrics">
          <div><span>Angle</span><strong>${hasPosition ? `${fmt.maybe(motor.position_rad, 3)} rad` : "-"}</strong></div>
          <div><span>Deg</span><strong>${hasPosition ? `${positionDeg.toFixed(1)} deg` : "-"}</strong></div>
          <div><span>Speed</span><strong>${fmt.maybe(motor.speed_dps, 0)}</strong></div>
          <div><span>Current</span><strong>${fmt.maybe(motor.current_a ?? motor.iq_a_approx, 2)} A</strong></div>
        </div>
      </article>
    `;
  }).join("");
}

function renderCurrentCommand(command) {
  const status = command.status || "waiting";
  const targets = Array.isArray(command.targets) ? command.targets : [];
  const source = command.source || "-";
  $("policyCommandBadge").textContent = status;
  $("policyCommandBadge").className = badgeClass(status);
  setText("policyCommandSource", source);
  setText("policyCommandTargets", String(command.target_count ?? targets.length));
  setText("policyCommandAge", fmt.age(command.age_s));
  setText("policyCommandError", command.error || "");

  if (!targets.length) {
    $("policyCommandRows").innerHTML = `
      <tr>
        <td colspan="6">${source === "-" || source === "NONE" ? "-" : `${escapeHtml(source)} command has no MIT target fields`}</td>
      </tr>
    `;
    return;
  }

  $("policyCommandRows").innerHTML = targets.map((target) => `
    <tr>
      <td><code>${escapeHtml(target.can_id || "-")}</code></td>
      <td>${fmt.maybe(target.p_target_rad ?? target.q, 3)}</td>
      <td>${fmt.maybe(target.v_target_rad_s ?? target.dq, 3)}</td>
      <td>${fmt.maybe(target.kp, 2)}</td>
      <td>${fmt.maybe(target.kd, 2)}</td>
      <td>${fmt.maybe(target.tau_target_nm ?? target.tau, 3)}</td>
    </tr>
  `).join("");
}

function renderFrames(frames) {
  if (!frames.length) {
    $("recentFrames").innerHTML = '<div class="frame-item"><strong>No frames</strong><p>Waiting for CAN traffic</p></div>';
    return;
  }
  $("recentFrames").innerHTML = frames.map((frame) => `
    <div class="frame-item">
      <strong><span>${frame.can_id}</span><span>${fmt.age(frame.age_s)}</span></strong>
      <p>DLC ${frame.dlc} · count ${frame.count}</p>
      <p><code>${escapeHtml(frame.data || "")}</code></p>
    </div>
  `).join("");
}

function processStatus(process) {
  if (process.alive === true) return "online";
  if (process.returncode === null || process.returncode === undefined) return "never";
  return process.returncode === 0 ? "stopped" : "error";
}

function renderProcesses(processes) {
  const items = Array.isArray(processes) ? processes : [];
  const onlineCount = items.filter((process) => process.alive === true).length;
  const badge = $("processSummaryBadge");
  badge.textContent = items.length ? `${onlineCount}/${items.length} online` : "no data";
  badge.className = items.length && onlineCount === items.length ? "badge ok" : items.length ? "badge warn" : "badge muted";

  if (!items.length) {
    $("processTiles").innerHTML = '<article class="process-tile"><header><h3>No process data</h3><span class="badge muted">waiting</span></header></article>';
    return;
  }

  $("processTiles").innerHTML = items.map((process) => {
    const config = process.config || {};
    const command = Array.isArray(config.command) ? config.command.join(" ") : "-";
    const terminal = Array.isArray(config.terminal_command) ? config.terminal_command.join(" ") : "-";
    const status = processStatus(process);
    const manageable = process.manageable !== false;
    const actionsHtml = manageable ? `
        <div class="process-actions">
          <button
            class="button primary compact"
            type="button"
            data-process-action="start"
            data-process-name="${escapeHtml(process.name || "")}"
            ${process.alive === true ? "disabled" : ""}
          >Start</button>
          <button
            class="button danger compact"
            type="button"
            data-process-action="stop"
            data-process-name="${escapeHtml(process.name || "")}"
            ${process.alive === true ? "" : "disabled"}
          >Stop</button>
        </div>
    ` : "";
    return `
      <article class="process-tile">
        <header>
          <h3>${escapeHtml(process.name || "-")}</h3>
          <span class="${badgeClass(status)}">${escapeHtml(status)}</span>
        </header>
        <div class="process-meta">
          <div><span>Terminal PID</span><strong>${fmt.maybe(process.pid, 0)}</strong></div>
          <div><span>Managed PID</span><strong>${fmt.maybe(process.managed_pid, 0)}</strong></div>
          <div><span>Return</span><strong>${fmt.maybe(process.returncode, 0)}</strong></div>
          <div><span>Start</span><strong>${fmt.maybe(config.start_order, 0)}</strong></div>
          <div><span>Stop</span><strong>${fmt.maybe(config.stop_order, 0)}</strong></div>
          <div><span>Terminal</span><strong>${config.new_terminal ? "new" : "inline"}</strong></div>
          <div><span>Workdir</span><strong>${escapeHtml(config.working_dir || "-")}</strong></div>
        </div>
        <p class="command-line" title="${escapeHtml(command)}">${escapeHtml(command)}</p>
        <p class="command-line" title="${escapeHtml(terminal)}">${escapeHtml(terminal)}</p>
        ${actionsHtml}
      </article>
    `;
  }).join("");
}

async function sendProcessAction(button) {
  if (!button || button.dataset.busy === "true") return;
  const name = button.dataset.processName;
  const action = button.dataset.processAction;
  if (!name || !action) return;
  if (name === "dashboard" && action === "stop") {
    const confirmed = window.confirm("Stop dashboard process? This page will disconnect.");
    if (!confirmed) return;
  }

  button.dataset.busy = "true";
  const previousText = button.textContent;
  button.textContent = action === "stop" ? "Stopping..." : "Starting...";
  button.disabled = true;
  try {
    const result = await postJson(`/api/processes/${encodeURIComponent(name)}/${action}`);
    showMessage(`${name} ${action} requested`);
    if (result && Array.isArray(result.processes)) {
      renderProcesses(result.processes);
    } else if (name !== "dashboard" || action !== "stop") {
      const response = await fetch("/api/processes");
      const data = await response.json();
      if (data && Array.isArray(data.processes)) {
        renderProcesses(data.processes);
      }
    }
  } catch (error) {
    showMessage(error.message, true);
  } finally {
    button.dataset.busy = "false";
    button.textContent = previousText;
    button.disabled = false;
  }
}

function kvHtml(entries) {
  return entries.map(([key, value]) => `
    <div>
      <span>${escapeHtml(key)}</span>
      <strong>${escapeHtml(value)}</strong>
    </div>
  `).join("");
}

function renderShm(state) {
  const shm = state.shm && state.shm.dashboard_state ? state.shm.dashboard_state : {};
  const payload = shm.payload || null;
  const status = shm.status || "disabled";

  $("shmStatusBadge").textContent = status;
  $("shmStatusBadge").className = badgeClass(status);
  setText("shmName", shm.shm_name || "-");
  setText("shmAge", fmt.age(shm.age_s));
  setText("shmControllerState", payload && payload.controller_state ? payload.controller_state : "-");
  setText("shmSchema", payload && payload.schema ? payload.schema : "-");

  const can = payload && payload.can ? payload.can : {};
  $("shmCanGrid").innerHTML = kvHtml([
    ["iface", can.iface || "-"],
    ["command timeout", can.command_timeout_s !== undefined ? `${fmt.maybe(can.command_timeout_s, 3)} s` : "-"],
    ["dashboard rx", state.can ? state.can.total_rx : "-"],
    ["dashboard tx", state.can ? state.can.total_tx : "-"],
  ]);

  const imu = payload && payload.imu ? payload.imu : {};
  const quatComm = imu.quat_comm || {};
  const gyroComm = imu.gyro_comm || {};
  $("shmImuGrid").innerHTML = kvHtml([
    ["quat", fmt.vector(imu.quat_xyzw)],
    ["gravity", fmt.vector(imu.projected_gravity_b)],
    ["gyro", fmt.vector(imu.angular_velocity_rad_s)],
    ["quat rx", fmt.maybe(quatComm.rx_count, 0)],
    ["gyro rx", fmt.maybe(gyroComm.rx_count, 0)],
    ["errors", `${fmt.maybe(quatComm.decode_error_count, 0)} / ${fmt.maybe(gyroComm.decode_error_count, 0)}`],
  ]);

  const actuators = payload && Array.isArray(payload.actuators) ? payload.actuators : [];
  $("shmActuatorCards").innerHTML = actuators.length
    ? actuators.map((actuator) => renderShmActuator(actuator)).join("")
    : '<article class="shm-actuator-card"><header><strong>No actuator data</strong><span class="badge muted">waiting</span></header></article>';

  $("shmJson").textContent = payload ? JSON.stringify(payload, null, 2) : JSON.stringify({ status, error: shm.error || null }, null, 2);
}

function renderShmActuator(actuator) {
  const comm = actuator.comm || {};
  const online = comm.is_online === true && comm.is_stale !== true;
  const status = online ? "online" : comm.rx_count > 0 ? "stale" : "never";
  return `
    <article class="shm-actuator-card">
      <header>
        <strong>${escapeHtml(actuator.name || hexCanId(actuator.can_id))}</strong>
        <span class="${badgeClass(status)}">${escapeHtml(status)}</span>
      </header>
      <div class="key-value-grid">
        <div><span>CAN</span><strong>${escapeHtml(hexCanId(actuator.can_id))}</strong></div>
        <div><span>Enabled</span><strong>${actuator.is_enabled === true ? "true" : actuator.is_enabled === false ? "false" : "-"}</strong></div>
        <div><span>Mode</span><strong>${escapeHtml(actuator.mode || "-")}</strong></div>
        <div><span>RX</span><strong>${fmt.maybe(comm.rx_count, 0)}</strong></div>
        <div><span>Pos</span><strong>${fmt.maybe(actuator.position_rad, 3)}</strong></div>
        <div><span>Vel</span><strong>${fmt.maybe(actuator.velocity_rad_s, 3)}</strong></div>
        <div><span>Torque</span><strong>${fmt.maybe(actuator.torque_nm, 3)}</strong></div>
        <div><span>Current</span><strong>${fmt.maybe(actuator.current_a, 2)}</strong></div>
        <div><span>Age</span><strong>${fmt.age(actuator.age_s)}</strong></div>
      </div>
    </article>
  `;
}

function syncControls(state) {
  if ($("txToggle")) {
    $("txToggle").textContent = state.safety.tx_enabled ? "Lock TX" : "Unlock TX";
    $("txToggle").className = state.safety.tx_enabled ? "button secondary" : "button danger";
  }
}

function offsetDegToCount(value) {
  const scaled = Number(value) * 100;
  if (!Number.isFinite(scaled)) return null;
  return scaled >= 0 ? Math.floor(scaled + 0.5) : Math.ceil(scaled - 0.5);
}

function updateZeroSetPreview() {
  const input = $("zeroSetOffsetDeg");
  const output = $("zeroSetOffsetCount");
  if (!input || !output) return null;
  const count = offsetDegToCount(input.value);
  output.value = count === null ? "-" : String(count);
  return count;
}

function syncZeroSetControls(state) {
  if (!$("zeroSetForm")) return;
  const stateName = controllerSafetyState(state);
  const count = updateZeroSetPreview();
  const countValid = count !== null && count >= -32768 && count <= 32767;
  const canSend = stateName === "NORMAL" && countValid;
  const select = $("zeroSetActuator");
  const submit = $("zeroSetSubmit");
  const badge = $("zeroSetStateBadge");
  const presetBadge = $("zeroSetPresetStateBadge");
  const presetCanSend = stateName === "NORMAL";

  if (badge) {
    badge.textContent = stateName === "NORMAL" ? "NORMAL" : `${stateName} blocked`;
    badge.className = stateName === "NORMAL" ? "badge ok" : "badge warn";
  }
  if (submit) {
    submit.disabled = !canSend || !select || !select.value;
    submit.title = canSend
      ? ""
      : countValid ? "Zero set is available only while controller is NORMAL" : "Offset count must fit int16";
  }
  if (presetBadge) {
    presetBadge.textContent = presetCanSend ? "NORMAL" : `${stateName} blocked`;
    presetBadge.className = presetCanSend ? "badge ok" : "badge warn";
  }
  document.querySelectorAll("[data-zero-preset]").forEach((button) => {
    button.disabled = !presetCanSend || button.dataset.busy === "true";
    button.title = presetCanSend ? "" : "Zero set preset is available only while controller is NORMAL";
  });
}

function syncOperatorControls(state) {
  const stateName = controllerSafetyState(state);
  if ($("armButton")) {
    const armBlocked = (
      stateName === "ESTOP"
      || stateName === "FAULT_LATCHED"
      || stateName === "SHUTTING_DOWN"
      || stateName === "STOPPED"
      || stateName === "ENABLING"
      || stateName === "DAMPING"
      || stateName === "NORMAL"
    );
    const canArm = !armBlocked || operatorEstopDampingActive(state);
    $("armButton").disabled = !canArm;
    $("armButton").title = canArm ? "" : `Arm is blocked while controller is ${stateName}`;
  }
  if ($("runButton")) {
    const canRun = stateName === "DAMPING";
    $("runButton").disabled = !canRun;
    $("runButton").title = canRun ? "" : "Run is available only after Arm reaches DAMPING";
  }
  if ($("dampingButton")) {
    const canDamping = stateName === "NORMAL";
    $("dampingButton").disabled = !canDamping;
    $("dampingButton").title = canDamping ? "" : "Damping is available only while NORMAL control is running";
  }
  if ($("faultClearButton")) {
    const canClear = stateName === "FAULT_LATCHED" || stateName === "ESTOP";
    $("faultClearButton").disabled = !canClear;
    $("faultClearButton").title = canClear ? "" : "Fault clear is available only in FAULT_LATCHED or ESTOP";
  }
  if ($("estopButton")) {
    $("estopButton").disabled = false;
    $("estopButton").title = operatorEstopDampingActive(state) ? "E-stop damping is active" : "";
  }
}

async function postJson(url, body = {}) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(data.detail || response.statusText);
  }
  return data;
}

async function loadDashboardConfig() {
  const response = await fetch("/api/config");
  dashboardConfig = await response.json();
  populateTransmitIds(dashboardConfig);
  populateZeroSetActuators(dashboardConfig);
  populateZeroSetPresets(dashboardConfig);
}

function populateTransmitIds(config) {
  const select = $("rawCanId");
  if (!select) return;
  const entries = (config.dashboard && Array.isArray(config.dashboard.transmit_ids))
    ? config.dashboard.transmit_ids
    : [];

  select.innerHTML = "";

  if (!entries.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "No transmit presets";
    option.disabled = true;
    select.appendChild(option);
    if ($("rawPayload")) $("rawPayload").value = "";
    return;
  }

  entries.forEach((entry, index) => {
    const canId = hexCanId(entry.can_id);
    const label = entry.label || canId;
    const option = document.createElement("option");
    option.value = canId;
    option.textContent = `${label} (${canId})`;
    option.dataset.payload = normalizePayload(entry.payload);
    select.appendChild(option);

    if (index === 0 && option.dataset.payload) {
      $("rawPayload").value = option.dataset.payload;
    }
  });
}

function populateZeroSetActuators(config) {
  const select = $("zeroSetActuator");
  if (!select) return;
  const actuators = Array.isArray(config.actuators) ? config.actuators : [];
  select.innerHTML = "";

  actuators.forEach((actuator) => {
    const canId = hexCanId(actuator.can_id);
    const option = document.createElement("option");
    option.value = canId;
    option.textContent = `${actuator.name || canId} (${canId})`;
    select.appendChild(option);
  });

  syncZeroSetControls(latestState || {});
}

function populateZeroSetPresets(config) {
  const panel = $("zeroSetPresetPanel");
  const container = $("zeroSetPresetButtons");
  if (!panel || !container) return;
  const presets = (config.dashboard && Array.isArray(config.dashboard.zero_set_presets))
    ? config.dashboard.zero_set_presets
    : [];

  container.innerHTML = "";
  panel.hidden = presets.length === 0;
  presets.forEach((preset, index) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "button secondary zero-preset-button";
    button.dataset.zeroPreset = preset.id || `preset_${index + 1}`;
    button.dataset.targets = JSON.stringify((preset.targets || []).map((target) => ({
      can_id: hexCanId(target.can_id),
      offset_count: target.offset_count,
    })));
    button.textContent = preset.label || `Preset ${index + 1}`;
    container.appendChild(button);
  });

  syncZeroSetControls(latestState || {});
}

function showMessage(text, isError = false) {
  const el = $("commandMessage");
  if (!el) return;
  el.textContent = text;
  el.style.color = isError ? "var(--danger)" : "var(--slate)";
}

async function sendMotorAction(button) {
  if (!button || button.dataset.busy === "true") return;

  button.dataset.busy = "true";
  try {
    const canId = button.dataset.canId;
    const action = button.dataset.motorAction;
    const apiAction = action === "toggle-enable" ? button.dataset.nextAction : action;
    if (action === "mit-poll") {
      const motors = latestState && Array.isArray(latestState.motors) ? latestState.motors : [];
      const motor = motors.find((item) => String(item.can_id) === String(canId));
      if (motor && motor.mit_polling) {
        await postJson(`/api/actuator/${canId}/mit-poll/stop`);
        showMessage(`${canId} MIT polling stopped`);
      } else {
        const confirmed = window.confirm(`Start MIT polling for ${canId}?`);
        if (!confirmed) return;
        await postJson(`/api/actuator/${canId}/mit-poll/start`, { confirmed });
        showMessage(`${canId} MIT polling started`);
      }
      return;
    }

    await postJson(`/api/actuator/${canId}/${apiAction}`, apiAction === "zero" ? { offset_count: 0 } : {});
    const label = apiAction === "enter" ? "enable" : apiAction === "exit" ? "disable" : "zero set";
    showMessage(`${canId} ${label} ${apiAction === "zero" ? "requested" : "sent"}`);
  } catch (error) {
    showMessage(error.message, true);
  } finally {
    button.dataset.busy = "false";
  }
}

async function sendAllActuatorToggle() {
  const button = $("allActuatorToggle");
  if (!button || button.dataset.busy === "true") return;
  const motors = latestState && Array.isArray(latestState.motors) ? latestState.motors : [];
  const action = button.dataset.nextAction || "enter";
  const targets = motors
    .filter((motor) => action === "exit" ? motor.enabled_hint === true : motor.enabled_hint !== true)
    .map((motor) => motor.can_id);

  if (!targets.length) {
    showMessage(action === "exit" ? "No enabled actuators" : "All actuators are already enabled");
    return;
  }

  button.dataset.busy = "true";
  try {
    for (const canId of targets) {
      await postJson(`/api/actuator/${canId}/${action}`);
    }
    showMessage(`${action === "exit" ? "Disable" : "Enable"} all sent (${targets.length})`);
  } catch (error) {
    showMessage(error.message, true);
  } finally {
    button.dataset.busy = "false";
  }
}

function connectWs() {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${scheme}://${window.location.host}/ws/state`);
  ws.onmessage = (event) => render(JSON.parse(event.data));
  ws.onclose = () => {
    if ($("socketBadge")) {
      $("socketBadge").textContent = "ws disconnected";
      $("socketBadge").className = "badge danger";
    }
    window.setTimeout(connectWs, 1000);
  };
}

function bindControls() {
  if ($("rawCanId")) {
    $("rawCanId").addEventListener("change", () => {
      const option = $("rawCanId").selectedOptions[0];
      if (option && option.dataset.payload && $("rawPayload")) {
        $("rawPayload").value = option.dataset.payload;
      }
    });
  }

  if ($("zeroSetOffsetDeg")) {
    $("zeroSetOffsetDeg").addEventListener("input", () => {
      syncZeroSetControls(latestState || {});
    });
  }

  if ($("zeroSetForm")) {
    $("zeroSetForm").addEventListener("submit", async (event) => {
      event.preventDefault();
      try {
        const canId = $("zeroSetActuator").value;
        const offsetDeg = Number($("zeroSetOffsetDeg").value);
        await postJson(`/api/actuator/${canId}/zero`, { offset_deg: offsetDeg });
        const count = updateZeroSetPreview();
        showMessage(`${canId} zero set requested (${fmt.maybe(offsetDeg, 2)} deg / ${count})`);
      } catch (error) {
        showMessage(error.message, true);
      }
    });
  }

  if ($("zeroSetPresetButtons")) {
    $("zeroSetPresetButtons").addEventListener("click", async (event) => {
      const button = event.target.closest("[data-zero-preset]");
      if (!button || button.disabled || button.dataset.busy === "true") return;
      button.dataset.busy = "true";
      syncZeroSetControls(latestState || {});
      try {
        const targets = JSON.parse(button.dataset.targets || "[]");
        await postJson("/api/operator/zero-set", { targets });
        showMessage(`${button.textContent} zero set requested`);
      } catch (error) {
        showMessage(error.message, true);
      } finally {
        button.dataset.busy = "false";
        syncZeroSetControls(latestState || {});
      }
    });
  }

  if ($("txToggle")) {
    $("txToggle").addEventListener("click", async () => {
      try {
        if (latestState && latestState.safety && latestState.safety.tx_enabled) {
          await postJson("/api/tx/lock");
          showMessage("TX locked");
          return;
        }
        await postJson("/api/tx/unlock");
        showMessage("TX enabled");
      } catch (error) {
        showMessage(error.message, true);
      }
    });
  }

  if ($("armButton")) {
    $("armButton").addEventListener("click", async () => {
      try {
        await postJson("/api/operator/arm");
        showMessage("Arm requested");
      } catch (error) {
        showMessage(error.message, true);
      }
    });
  }

  if ($("runButton")) {
    $("runButton").addEventListener("click", async () => {
      try {
        await postJson("/api/operator/run");
        showMessage("Run requested");
      } catch (error) {
        showMessage(error.message, true);
      }
    });
  }

  if ($("dampingButton")) {
    $("dampingButton").addEventListener("click", async () => {
      try {
        await postJson("/api/operator/damping");
        showMessage("Damping requested");
      } catch (error) {
        showMessage(error.message, true);
      }
    });
  }

  if ($("faultClearButton")) {
    $("faultClearButton").addEventListener("click", async () => {
      try {
        await postJson("/api/operator/fault-clear");
        showMessage("Fault clear requested");
      } catch (error) {
        showMessage(error.message, true);
      }
    });
  }

  if ($("estopButton")) {
    $("estopButton").addEventListener("click", async () => {
      try {
        await postJson("/api/operator/estop");
        showMessage("ESTOP requested");
      } catch (error) {
        showMessage(error.message, true);
      }
    });
  }

  if ($("rawForm")) {
    $("rawForm").addEventListener("submit", async (event) => {
      event.preventDefault();
      try {
        await postJson("/api/can/send", {
          can_id: $("rawCanId").value,
          data: $("rawPayload").value,
        });
        showMessage("Frame sent");
      } catch (error) {
        showMessage(error.message, true);
      }
    });
  }

  if ($("allActuatorToggle")) {
    $("allActuatorToggle").addEventListener("click", () => {
      sendAllActuatorToggle();
    });
  }

  if ($("processTiles")) {
    $("processTiles").addEventListener("pointerdown", (event) => {
      const button = event.target.closest("[data-process-action]");
      if (!button) return;
      event.preventDefault();
      sendProcessAction(button);
    });

    $("processTiles").addEventListener("click", (event) => {
      const button = event.target.closest("[data-process-action]");
      if (!button || event.detail !== 0) return;
      sendProcessAction(button);
    });
  }

  if ($("motorRows")) {
    $("motorRows").addEventListener("pointerdown", (event) => {
      const button = event.target.closest("[data-motor-action]");
      if (!button) return;
      event.preventDefault();
      sendMotorAction(button);
    });

    $("motorRows").addEventListener("click", (event) => {
      const button = event.target.closest("[data-motor-action]");
      if (!button || event.detail !== 0) return;
      sendMotorAction(button);
    });
  }
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

loadDashboardConfig()
  .catch((error) => showMessage(error.message, true))
  .finally(() => {
    bindControls();
    connectWs();
  });
