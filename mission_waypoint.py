# mission_waypoint.py (PX4 + MAVSDK v1.x 호환)
import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

async def main():
    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("⏳ PX4 연결 중…")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ PX4 연결 성공!")
            break

    print("📡 글로벌 위치 대기…")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("🌍 글로벌 위치 OK!")
            break

    await drone.action.arm()
    print("🔧 ARM 완료")

    # -------------------------
    # 📌 Waypoint 4개 정의
    # PX4 SITL Default 위치 기준
    # -------------------------

    mission_items = []

    def wp(lat, lon, alt):
        return MissionItem(
            lat, lon,
            alt,            # relative altitude
            5.0,            # speed m/s
            True,           # is_fly_through
            float('nan'),   # gimbal_pitch
            float('nan'),   # gimbal_yaw
            MissionItem.CameraAction.NONE,
            float('nan'),   # loiter_time_s
            float('nan'),   # camera_photo_interval
            float('nan'),   # acceptance_radius_m
            float('nan'),   # yaw_deg
            float('nan')    # camera_photo_distance_m
        )

    # WP1: 이륙 지점
    mission_items.append(wp(47.3981703, 8.5456490, 10))

    # WP2: 북쪽
    mission_items.append(wp(47.3982500, 8.5456490, 10))

    # WP3: 동쪽
    mission_items.append(wp(47.3982500, 8.5458000, 10))

    # WP4: 다시 시작점 복귀
    mission_items.append(wp(47.3981703, 8.5456490, 10))

    mission_plan = MissionPlan(mission_items)

    print("📤 미션 업로드 중…")
    await drone.mission.upload_mission(mission_plan)
    print("✅ 미션 업로드 완료!")

    print("🚀 미션 시작!")
    await drone.mission.start_mission()

    # 진행 상태 출력
    async for prog in drone.mission.mission_progress():
        print(f"📍 진행: {prog.current}/{prog.total}")
        if prog.current == prog.total:
            break

    print("🛬 착륙 중…")
    await drone.action.land()
    await asyncio.sleep(5)
    print("🏁 종료!")

if __name__ == "__main__":
    asyncio.run(main())
