from time import sleep
from e_drone.drone import Drone, convertByteArrayToString
from e_drone.protocol import LightModeDrone

if __name__ == "__main__":

    PORT_NAME = "COM7"  # 실제 연결된 포트로 변경

    # 드론 객체 생성 (Mission, Event, Light, Sensor, Logger 활성화)
    drone = Drone(True, True, True, True, True)
    drone.open(PORT_NAME)

    print("💡 LED 색상 변경 테스트 시작!")

    try:
        # 색상 리스트 (RGB 순서대로)
        led_colors = [
            (255, 0, 0),  # 빨강
            (0, 255, 0),  # 초록
            (0, 0, 255)   # 파랑
        ]

        # 3초마다 색상 변경, 총 3번
        for i in range(3):
            r, g, b = led_colors[i]
            
            dataArray = drone.sendLightDefaultColor(
                LightModeDrone.BodyDimming,  # LED 모드
                1,                           # LED 번호 1 = 드론 본체
                r, g, b                      # RGB 색상
            )

            print(f"{i+1}/3 → 색상 R:{r} G:{g} B:{b} / {convertByteArrayToString(dataArray)}")
            sleep(3)  # 3초 유지

    finally:
        # 종료 시 LED 끄기
        drone.sendLightDefaultColor(LightModeDrone.BodyDimming, 1, 0, 0, 0)
        drone.close()
        print("💡 LED 색상 변경 테스트 종료!")