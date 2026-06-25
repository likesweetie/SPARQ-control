sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

if ! ip link show vcan0 > /dev/null 2>&1; then
  sudo ip link add dev vcan0 type vcan
fi

sudo ip link set up vcan0

python3 -m Dashboard.backend.app


python3 -m robot_controller.main --config robot_controller/configs/robot_controller.yaml
