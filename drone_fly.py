from time import sleep
from e_drone.drone import *

drone = Drone()
drone.open("COM7")

# 안전 확인: 드론 바닥에 평평하게 두기
print("🚨 드론 바닥에 평평하게 두고 준비하세요!")

# 이륙
drone.sendTakeOff()
sleep(5)  # 5초 동안 공중 유지

# 착륙 
drone.sendLanding()
sleep(3)

drone.close()
print("✅ 드론 이륙 및 착륙 테스트 완료!")