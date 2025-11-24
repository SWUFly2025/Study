from time import sleep
from e_drone.drone import *
from e_drone.protocol import *

# 포트번호를 실제 장치 포트로 변경 (예: COM3, COM4 등)
PORT_NAME = "COM7"

drone = Drone()
drone.open(PORT_NAME)

# 조종기와 드론 연결 확인용: 버저 0.5초 울리기
drone.sendBuzzer(BuzzerMode.Scale, BuzzerScale.C4.value, 500)

sleep(1)
drone.close()
print("✅ 연결 및 통신 성공!")