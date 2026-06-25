# QHRR0 Control

GitHub: [likesweetie/QHRR0-control](https://github.com/likesweetie/QHRR0-control)

QHRR 계열 로봇을 위한 Python 기반 제어기 프로젝트입니다. CAN 기반 actuator/IMU bringup, ONNX policy inference, MIT command 송신, child process supervision, Robot State dashboard를 하나의 런타임으로 묶어 관리합니다.

현재 기본 설정은 `runtime.mode: simulation`, CAN interface `vcan0` 대상입니다. 실제 로봇에서 실행하기 전에는 반드시 `docs/SAFETY.md`, `docs/CAN_INTERFACE.md`, `docs/CONFIG_SCHEMA.md`를 먼저 확인하세요.

## Simulation Quick Start

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

python3 -m robot_controller.main --config config/app_config/robot_controller.yaml
```

Dashboard는 기본 설정 기준으로 아래 주소에서 실행됩니다.

```text
http://127.0.0.1:8000
```

MuJoCo simulation은 별도 터미널에서 실행합니다.

```bash
python3 run_mujoco_simulation.py
```

## Hardware Mode

Hardware mode는 YAML 변경만으로 실행되지 않습니다. `config/app_config/robot_controller.yaml`에서 `runtime.mode: hardware`를 설정한 뒤, 실제 CAN interface와 hardware gate를 명시적으로 통과해야 합니다.

```bash
python3 -m robot_controller.main \
  --config config/app_config/robot_controller.yaml \
  --hardware \
  --i-understand-this-can-enable-motors \
  --estop-ok
```

Hardware mode startup validation:

| Gate | Required behavior |
| --- | --- |
| `--hardware` | hardware mode 실행 의도 확인 |
| `--i-understand-this-can-enable-motors` | 실제 motor enable 가능성을 명시적으로 확인 |
| `--estop-ok` | `hardware.require_estop: true`일 때 E-stop 확인 |
| `hardware.allow_real_can` | hardware mode에서 `true`여야 함 |
| `hardware.require_manual_arm` | hardware mode에서 `true`여야 함 |
| `hardware.allow_enable_on_start` | hardware mode에서 `false`여야 함 |
| `can.motors.enter_on_start` | hardware mode에서 금지 |

Hardware mode는 `ControllerMode.DISABLED`에서 시작하며, startup 중 motor enable command를 보내지 않습니다. Arm/enable은 dashboard 또는 다른 operator process가 `OperatorCommandShm`에 `ENABLE` command를 쓴 뒤 `RobotController` 상태 머신이 처리합니다. Arm은 motor enable 후 `DAMPING`으로만 들어가며, policy command 송신은 별도 `RUN` command가 있어야 시작됩니다.

## Project Layout

| Path | Description |
| --- | --- |
| `config/app_config/` | project-wide YAML config |
| `hal/` | product-independent CAN frame, daemon, dispatcher, process transport, base device driver/protocol abstractions |
| `qhrr0_hw/` | QHRR0-specific SPG/DongilC actuator protocol, E2BOX IMU protocol, CAN ID map, joint map, calibration, robot spec |
| `robot_controller/controller.py` | `RobotController` main runtime, state-machine update, direct HAL actuator command dispatch |
| `robot_controller/state_machine.py` | `ControllerMode` and `OperatorCommandCode` transition policy |
| `robot_controller/shm/` | ctypes C-compatible `ControlCommandShm`, `OperatorCommandShm`, `RobotStateShm` |
| `robot_controller/telemetry/` | `RobotSnapshot`, `ShmStatePublisher`, `DashboardPublisher` |
| `robot_controller/supervisor/` | child process lifecycle management |
| `robot_controller/subprocesses/` | child process entrypoints: CAN daemon, task controller, dashboard, aux reader |
| `docs/` | handoff, architecture, safety, runbook 문서 |
| `config/` | policy/controller 관련 설정 |
| `policy/` | ONNX policy artifacts |
| `third_party/` | external dependencies and assets |

## Main Documents

| Document | Purpose |
| --- | --- |
| `docs/HANDOFF.md` | 새 개발자를 위한 핵심 요약 |
| `docs/ARCHITECTURE.md` | runtime/process/data-flow 구조 |
| `docs/RUNBOOK.md` | 실행, 종료, 복구 절차 |
| `docs/SAFETY.md` | fallback, timeout, fault, 실제 로봇 checklist |
| `docs/CONTROL_LOOP.md` | controller tick, policy inference, MIT command 흐름 |
| `docs/IPC_SHM.md` | shared memory layout와 ownership |
| `docs/CAN_INTERFACE.md` | CAN ID, payload, debugging |
| `docs/CONFIG_SCHEMA.md` | YAML config schema |
| `docs/TEST_PLAN.md` | smoke/integration/fault injection test |

## Safety Notes

- `RobotController.tick()`는 operator command를 읽고 상태 머신을 업데이트한 뒤, 현재 `ControllerMode`별로 정확히 하나의 actuator output path만 실행합니다.
- `ENABLING` 상태에서는 enable command만 송신하며, policy/damping/zero/disable command를 섞지 않습니다.
- Arm 이후에는 `DAMPING` 상태로 머물며, dashboard `Run` 버튼이 `RUN` command를 보낼 때만 `NORMAL`로 전환됩니다.
- `NORMAL` 상태에서만 `ControlCommandShm.read_relaxed()`를 호출하고 policy MIT command를 송신합니다.
- 여러 actuator 대상 enable/disable/zero/damping/policy 송신은 `RobotController` private method의 단순 for-loop에서 직접 보입니다.
- `ControlCommandShm`은 ctypes C-compatible layout이며 motor command tearing을 의도적으로 허용합니다.
- seqlock, sequence counter, zero-set generation은 사용하지 않습니다.
- 외부 GUI/operator process는 safety mode를 SHM에 쓰지 않고 `OperatorCommandShm`에 command만 씁니다.
- telemetry는 control용 `ShmStatePublisher`와 dashboard용 `DashboardPublisher`로 분리되어 있습니다.
- `runtime.mode: simulation`에서 `can0` 같은 real CAN interface는 reject됩니다.
- `runtime.mode: hardware`에서 `vcan0`는 reject됩니다.
- `can.motors.enter_on_start: true`는 simulation/hardware startup gate에서 금지됩니다.
- `motor_id` 대신 CAN ID를 기준으로 actuator를 식별합니다.
- HAL은 `qhrr0_hw`를 import하지 않습니다. QHRR0 제품 종속 구현은 최상단 `qhrr0_hw/`에 둡니다.
- silent fallback은 금지합니다. fallback policy는 `FALLBACK_POLICY.md`를 따릅니다.

## Useful Commands

```bash
python3 -m robot_controller.main --config config/app_config/robot_controller.yaml
python3 -m robot_controller.subprocesses.can_daemon.main --config config/app_config/robot_controller.yaml --replace-existing-socket
python3 -m robot_controller.subprocesses.task_controller.main --help
python3 run_mujoco_simulation.py --help
candump -td vcan0
```
