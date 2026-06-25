
BITRATE="1000000"
SAMPLE_POINT="0.875"

echo "--- CAN 인터페이스 초기화 시작 ---"
for CAN_INT in vcan0 vcan1 vcan2 vcan3; do
    sudo ip link set ${CAN_INT} up type vcan bitrate ${BITRATE} sample-point ${SAMPLE_POINT} 
done

gnome-terminal -- bash -c "canbusload vcan0@1000000 vcan1@1000000 vcan2@1000000 vcan3@1000000 -r -t -b; exec bash"
echo "프로젝트 빌드 상태를 확인합니다..."
make || { echo "빌드 실패! 종료합니다."; exit 1; }

echo "CAN 통신 프로그램을 실행합니다."
./SPARQ_CAN 2>&1 | tee output.log