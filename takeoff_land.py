import asyncio
from mavsdk import System

async def main():
    # PX4 SITL과 연결 (포트 14540)
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("드론 연결 `대기` 중...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✔ 드론 연결 완료!")
            break

    print("아밍 중...")
    await drone.action.arm()

    print("🚀 이륙 중...")
    await drone.action.takeoff()
    await asyncio.sleep(6)   # 5~7초 정도 호버

    print("🛬 착륙 중...")
    await drone.action.land()
    await asyncio.sleep(6)

    print("✔ 임무 종료")

if __name__ == "__main__":
    asyncio.run(main())