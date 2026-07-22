# QHRR0 Control

GitHub: [likesweetie/QHRR0-control](https://github.com/likesweetie/QHRR0-control)

QHRR 계열 로봇을 위한 Python 기반 제어기 프로젝트입니다. CAN 기반 actuator/IMU bringup, ONNX policy inference, MIT command 송신, child process supervision, Robot State dashboard를 하나의 런타임으로 묶어 관리합니다.


## Environment Setup (Radxa Q6)

대상 보드는 Radxa Q6입니다. Q6는 PC의 `x86_64` 환경과 CPU 아키텍처가 다르므로, PC에서 만든 conda environment나 빌드된 실행 파일/공유 라이브러리를 그대로 복사해서 쓰지 않습니다. Q6 보드 위에서 `aarch64`용 conda와 Python wheel을 설치하고, C++ subprocess도 보드에서 다시 빌드합니다.

### 1. System packages

```bash
sudo apt update
sudo apt install -y build-essential make g++ cmake git can-utils
```

보드 아키텍처를 먼저 확인합니다.

```bash
uname -m
```

Q6에서는 보통 아래처럼 나와야 합니다.

```text
aarch64
```

### 2. Conda setup

Q6에는 ARM64/aarch64용 Miniforge 또는 Miniconda를 설치합니다. 이미 conda가 설치되어 있다면 이 단계는 건너뜁니다. 설치 파일 이름은 배포판에 따라 달라질 수 있지만, 반드시 `Linux-aarch64` 빌드를 사용합니다.

```bash
# 예시: Miniforge3 Linux aarch64 installer를 받은 뒤 실행
bash Miniforge3-Linux-aarch64.sh

# 새 터미널을 열거나 shell 초기화 후
conda create -n sparq-control python=3.11 -y
conda activate sparq-control
python -m pip install --upgrade pip
```

PC에서 만든 `environment.yml` 또는 `conda env export` 결과를 Q6에 그대로 적용하지 않습니다. `linux-64` 패키지가 섞일 수 있기 때문에 Q6에서는 새 environment를 만든 뒤 requirements를 다시 설치합니다.

### 3. Python requirements

루트 requirements를 설치합니다. 여기에는 controller, policy runner, dashboard 실행에 필요한 Python 패키지가 들어 있습니다.

```bash
conda activate sparq-control
python -m pip install -r requirements.txt
```

설치 후 ONNX Runtime이 Q6에서 로드되는지 확인합니다.

```bash
python - <<'PY'
import platform
import onnxruntime as ort

print("machine:", platform.machine())
print("onnxruntime:", ort.__version__)
print("providers:", ort.get_available_providers())
PY
```

### 4. Native binaries

현재 repository에 포함된 일부 `third_party` 라이브러리와 기존 빌드 산출물은 `x86_64`일 수 있습니다. Q6 hardware bringup에서는 MuJoCo simulation용 `third_party/mujoco`가 필요하지 않습니다. `sparq_can` 같은 C++ subprocess는 Q6에서 다시 빌드합니다.

```bash
make -C robot_controller/subprocesses/sparq_can clean
make -C robot_controller/subprocesses/sparq_can
file robot_controller/subprocesses/sparq_can/SPARQ_CAN
```

`file` 출력에 `aarch64` 또는 `ARM aarch64`가 보여야 Q6에서 실행 가능한 바이너리입니다.

Dashboard는 기본 설정 기준으로 아래 주소에서 실행됩니다.

```text
http://127.0.0.1:8000
```


## Hardware Mode

Hardware mode는 YAML 변경만으로 실행되지 않습니다. `config/app_config/robot_controller.yaml`에서 `runtime.mode: hardware`를 설정한 뒤, 실제 CAN interface와 hardware gate를 명시적으로 통과해야 합니다.

```bash
python3 -m robot_controller.main \
  --config config/app_config/robot_controller.yaml \
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

## How to start

- imu_serial_cpp는 clean 후 직접 cmake로 빌드
- subprocess의 feedback_bridge, policy_bridge, sparq_can은 clean 후 make로 빌드 해야 할 수 있습니다.
- imu_birdge는 실행하지 않습니다.
- 죽은 터미널의 에러메시지는 log의 최신폴더 바로 이전거에서 확인


```bash
python3 -m robot_controller.main \
  --config config/app_config/robot_controller.yaml \
```


CAN setup:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

sudo ip link set can2 down
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up

sudo ip link set can3 down
sudo ip link set can3 type can bitrate 1000000
sudo ip link set can3 up

sudo ip link set can4 down
sudo ip link set can4 type can bitrate 1000000
sudo ip link set can4 up
```

