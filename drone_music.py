from time import sleep
from e_drone.drone import *
from e_drone.protocol import *

# COM 포트 설정
PORT_NAME = "COM7"

# 음계 정의
notes = {
    "C": BuzzerScale.C4.value,
    "D": BuzzerScale.D4.value,
    "E": BuzzerScale.E4.value,
    "F": BuzzerScale.F4.value,
    "G": BuzzerScale.G4.value,
    "A": BuzzerScale.A4.value,
    "B": BuzzerScale.B4.value
}

# Twinkle Twinkle Little Star 멜로디
song = [
    ("C", 500), ("C", 500), ("G", 500), ("G", 500), ("A", 500), ("A", 500), ("G", 1000),  # Twinkle Twinkle Little Star
    ("F", 500), ("F", 500), ("E", 500), ("E", 500), ("D", 500), ("D", 500), ("C", 1000),  # How I wonder what you are
    ("G", 500), ("G", 500), ("F", 500), ("F", 500), ("E", 500), ("E", 500), ("D", 1000),  # Up above the world so high
    ("G", 500), ("G", 500), ("F", 500), ("F", 500), ("E", 500), ("E", 500), ("D", 1000),  # Like a diamond in the sky
    ("C", 500), ("C", 500), ("G", 500), ("G", 500), ("A", 500), ("A", 500), ("G", 1000),  # Twinkle Twinkle Little Star
    ("F", 500), ("F", 500), ("E", 500), ("E", 500), ("D", 500), ("D", 500), ("C", 1000)   # How I wonder what you are
]

# 드론 연결
drone = Drone()
drone.open(PORT_NAME)

print("🎵 작은 별 연주 시작!")

# 연주
for note, duration in song:
    drone.sendBuzzer(BuzzerMode.Scale, notes[note], duration)
    sleep(duration / 1000 + 0.1)  # 다음 음 사이 약간의 텀

# 종료
drone.close()
print("🎵 연주 완료!")