#!/usr/bin/env bash
# MuJoCo 시뮬레이션 + task_controller obs 디버그 출력 실행 스크립트.
#
# processes.yaml의 task_controller 항목에 POLICY_DEBUG_OBS=1 이 이미 설정되어 있어서,
# task_controller 전용 gnome-terminal 창에 매 1초마다 컴포넌트별 observation 값이 출력됩니다.
#
# 사용법:
#   ./scripts/run_mujoco_debug_obs.sh
#
# 종료:
#   이 스크립트를 실행한 터미널에서 Ctrl+C 하면 robot_controller.main과
#   함께 백그라운드로 띄운 mujoco_simulate도 정리됩니다.
#   (can_daemon / sparq_can / aux_reader / task_controller / dashboard 창은
#    robot_controller.main 자체 ProcessSupervisor가 종료 처리합니다.)

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

if ! ip link show can0 >/dev/null 2>&1; then
    echo "[run_mujoco_debug_obs] can0 인터페이스가 없어 생성합니다 (sudo 필요)"
    sudo modprobe can
    sudo ip link add dev can0 type can
    sudo ip link set up can0
else
    echo "[run_mujoco_debug_obs] vcan0 인터페이스 이미 존재, 재사용"
fi

MUJOCO_PID=""
cleanup() {
    if [[ -n "${MUJOCO_PID}" ]] && kill -0 "${MUJOCO_PID}" 2>/dev/null; then
        echo "[run_mujoco_debug_obs] mujoco_simulate(PID ${MUJOCO_PID}) 종료"
        kill "${MUJOCO_PID}" 2>/dev/null || true
        wait "${MUJOCO_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "[run_mujoco_debug_obs] MuJoCo 시뮬레이션 시작 (백그라운드)"
python3 run_mujoco_simulation.py &
MUJOCO_PID=$!

echo "[run_mujoco_debug_obs] robot_controller.main 시작 (foreground, 새 터미널 창들이 뜹니다)"
echo "[run_mujoco_debug_obs] task_controller 창에서 [task_controller] obs(...) 로그를 확인하세요"
python3 -m robot_controller.main --config config/app_config/robot_controller.yaml
