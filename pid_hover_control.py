# pid_hover_control.py
import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw
import time


# -----------------------------
# PID Controller 클래스
# -----------------------------
class PID:
    def __init__(self, kp, ki, kd, limit=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.limit = limit

    def control(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        output = max(min(output, self.limit), -self.limit)
        self.prev_error = error
        return output


async def main():
    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("⏳ PX4 연결 중…")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ PX4 연결 성공!")
            break

    print("📡 위치 수신 대기…")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("📍 위치 OK!")
            break

    await drone.action.arm()
    print("🔧 Armed")

    # Offboard 시작 준비 — 반드시 0명령 한번 보내야 한다!
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    try:
        await drone.offboard.start()
        print("🚀 Offboard 시작!")
    except:
        print("⚠ Offboard 실패")
        return

    # 초기 위치 저장
    async for pos in drone.telemetry.position_velocity_ned():
        origin_n = pos.position.north_m
        origin_e = pos.position.east_m
        print(f"📌 기준점: N={origin_n:.2f}, E={origin_e:.2f}")
        break

    pid_n = PID(0.8, 0.03, 0.2)
    pid_e = PID(0.8, 0.03, 0.2)

    last_t = time.time()
    print("🔄 PID 균형 제어 시작!")

    while True:
        async for pos in drone.telemetry.position_velocity_ned():
            now = time.time()
            dt = now - last_t
            last_t = now

            error_n = origin_n - pos.position.north_m
            error_e = origin_e - pos.position.east_m

            vn = pid_n.control(error_n, dt)
            ve = pid_e.control(error_e, dt)
            vz = 0.0

            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(vn, ve, vz, 0.0)
            )

            print(f"[PID] errN={error_n:.2f} vn={vn:.2f} | errE={error_e:.2f} ve={ve:.2f}")
            break

        await asyncio.sleep(0.02)


if __name__ == "__main__":
    asyncio.run(main())
