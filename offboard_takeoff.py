import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print(" 드론 연결 기다리는 중...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(" 드론 연결 완료")
            break

    print(" 위치 추정 준비 중...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(" GPS / 홈포지션 OK")
            break

    print("Arm")
    await drone.action.arm()

    # Offboard 시작을 위한 초기 setpoint
    print("📡 Offboard 시작 준비")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -0.1, 0.0))

    try:
        await drone.offboard.start()
        print("Offboard 모드 시작됨!")

    except OffboardError as error:
        print(f"Offboard error: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # ---- 실제 비행 명령 -----
    print("Takeoff: 3m 고도까지 상승")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(5)

    print("3m 앞으로 이동")
    await drone.offboard.set_position_ned(PositionNedYaw(3.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(5)

    print("3m 왼쪽 이동")
    await drone.offboard.set_position_ned(PositionNedYaw(3.0, -3.0, -3.0, 0.0))
    await asyncio.sleep(5)

    print("착륙 지점으로 돌아가기")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(5)

    print("🛬 RTL")
    await drone.action.return_to_launch()

    print("끝!")


if __name__ == "__main__":
    asyncio.run(run())
