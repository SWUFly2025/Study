import asyncio
from mavsdk import System


async def main():
    # PX4 SITL 기본 MAVLink 포트
    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("⏳ PX4 시스템 연결 대기 중...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ PX4 연결 성공!")
            break

    # ARM
    print("🔧 Arm...")
    await drone.action.arm()

    # Takeoff
    print("🚀 이륙 중...")
    await drone.action.takeoff()

    # 5초 대기 → Hover
    await asyncio.sleep(5)

    print("🛬 착륙 중...")
    await drone.action.land()

    # 완료 대기
    await asyncio.sleep(5)
    print("🏁 종료!")


if __name__ == "__main__":
    asyncio.run(main())
