import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


async def fly_to(drone, x, y, z, yaw=0.0, wait=6):
    """ 드론을 지정 좌표로 이동시키는 Helper Function """
    print(f" 이동 중 → x:{x:.2f}, y:{y:.2f}, z:{z:.2f}")
    await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw))
    await asyncio.sleep(wait)


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("🔌 PX4 연결 대기중...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(" PX4 연결 완료!")
            break

    print("🛰 GPS / 홈포지션 대기중...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(" GPS OK!")
            break

    print(" Arm")
    await drone.action.arm()

    # offboard 초기 setpoint
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -0.1, 0))
    await asyncio.sleep(1)

    try:
        await drone.offboard.start()
        print(" Offboard 모드 시작!")
    except OffboardError as e:
        print(f" Offboard 오류: {e}")
        await drone.action.disarm()
        return

    # -------------------------------
    # 도형 좌표 (고도 3m 유지: z = -3)
    # -------------------------------
    # 원기둥 → 첫 번째
    x_cyl, y_cyl = -1.544886, 11.502676

    # 구 → 두 번째
    x_sph, y_sph = -27.076600, 28.449200

    # 네모 → 세 번째
    x_box, y_box = 0.685050, -2.183419

    # 이륙 (3m 고도)
    print(" 이륙 중 (고도 3m)")
    await fly_to(drone, 0, 0, -3.0)

    # 1. 원기둥으로 이동
    print("1단계: 원기둥 이동중")
    await fly_to(drone, x_cyl, y_cyl, -3.0)

    # 2. 구로 이동
    print("2단계: 구 이동중")
    await fly_to(drone, x_sph, y_sph, -3.0)

    # 3. 네모로 이동
    print("3단계: 네모 이동중")
    await fly_to(drone, x_box, y_box, -3.0)

    # 홈으로 복귀
    print("홈(시작지점) 복귀 중")
    await fly_to(drone, 0, 0, -3.0)

    # RTL
    print("RTL 시작")
    await drone.action.return_to_launch()

    print(" 미션 완료!")


if __name__ == "__main__":
    asyncio.run(run())
