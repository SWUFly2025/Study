import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
import time
import math

class OffboardControl(Node):
    """
    Offboard 모드를 사용하여 특정 좌표로 드론을 이동시키는 ROS 2 노드
    """
    def __init__(self):
        super().__init__('offboard_control')

        # 1. 서비스 클라이언트 생성 (시동, 모드 설정)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # 2. Setpoint Publisher 생성 (목표 위치 전송)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # 3. 상태 및 위치 Subscriber
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pos_cb, 10)

        # 4. 상태 변수 초기화
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.setpoint = PoseStamped()
        self.target_waypoint = [5.0, 5.0, 10.0]  # [X, Y, Z] 목표 좌표 설정 (예시)
        self.waypoint_reached = False
        self.target_yaw = 0.0 # 목표 요각 (북쪽 기준)

        # 5. 제어 루프 타이머 (Setpoint을 20Hz로 지속 발행)
        self.timer = self.create_timer(0.05, self.timer_cb) # 20 Hz (50ms)

    # --- 콜백 함수 ---
    def state_cb(self, msg):
        self.current_state = msg

    def pos_cb(self, msg):
        self.current_pose = msg

    # --- 서비스 호출 함수 ---
    def call_service(self, client, request):
        """서비스 호출을 동기적으로 처리"""
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {client.srv_type} service...')
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        return future.result()

    def arm(self, arm_value):
        """시동/시동 해제 서비스 호출"""
        request = CommandBool.Request()
        request.value = arm_value
        result = self.call_service(self.arming_client, request)
        self.get_logger().info(f'Arming request success: {result.success}')
        return result.success

    def set_mode(self, mode_name):
        """비행 모드 설정 서비스 호출"""
        request = SetMode.Request()
        request.custom_mode = mode_name
        result = self.call_service(self.set_mode_client, request)
        self.get_logger().info(f'Mode change to {mode_name} success: {result.mode_sent}')
        return result.mode_sent
        
    # --- 제어 루프 ---
    def timer_cb(self):
        # 1. Setpoint 데이터 설정
        self.setpoint.pose.position.x = self.target_waypoint[0]
        self.setpoint.pose.position.y = self.target_waypoint[1]
        self.setpoint.pose.position.z = self.target_waypoint[2]
        
        # 쿼터니언 변환 (단순 Yaw 설정, 복잡한 자세 제어는 생략)
        q = self.euler_to_quaternion(0, 0, self.target_yaw)
        self.setpoint.pose.orientation.x = q[0]
        self.setpoint.pose.orientation.y = q[1]
        self.setpoint.pose.orientation.z = q[2]
        self.setpoint.pose.orientation.w = q[3]

        # 2. Setpoint 지속 발행 (Offboard 모드 진입 필수 조건)
        self.setpoint_pub.publish(self.setpoint)

        # 3. 비행 로직 (모든 명령은 Setpoint 발행 후에 이루어져야 함)
        if self.current_state.connected:
            if not self.current_state.armed and self.current_state.mode == 'GUIDED':
                self.arm(True) # 시동
                self.get_logger().info("Drone Armed.")

            elif self.current_state.armed:
                # 안전 고도까지 올라간 후 OFFBOARD 모드 전환 시도
                current_z = self.current_pose.pose.position.z
                
                # 시동 후 5m까지는 GUIDED 모드로 이륙
                if self.current_state.mode == 'GUIDED' and current_z < 5.0 and self.target_waypoint[2] > 0:
                    self.get_logger().info(f"Taking off to 5.0m... Current Z: {current_z:.2f}")
                    # 이륙 고도를 setpoint로 설정 (z=5m)
                    self.setpoint.pose.position.z = 5.0 

                # 안전 고도 도달 후 OFFBOARD 모드 전환
                elif current_z >= 4.5 and self.current_state.mode != 'OFFBOARD':
                    self.set_mode('OFFBOARD')
                    self.get_logger().info("Switched to OFFBOARD mode. Starting Waypoint mission.")
                
                # OFFBOARD 모드에서 Waypoint 이동 및 도달 확인
                elif self.current_state.mode == 'OFFBOARD' and not self.waypoint_reached:
                    distance = self.get_distance(
                        self.current_pose.pose.position.x,
                        self.current_pose.pose.position.y,
                        self.current_pose.pose.position.z,
                        self.target_waypoint[0],
                        self.target_waypoint[1],
                        self.target_waypoint[2]
                    )

                    if distance < 0.5: # 50cm 이내 도달 시
                        self.waypoint_reached = True
                        self.get_logger().info("!!! Waypoint Reached! Mission Complete. !!!")
                        # 임무 완료 후 착륙 모드 (LAND) 등으로 전환 필요
                        # self.set_mode('LAND')

    # --- 유틸리티 함수 ---
    def get_distance(self, x1, y1, z1, x2, y2, z2):
        """두 3D 좌표 사이의 유클리드 거리를 계산"""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """오일러 각을 쿼터니언으로 변환 (단순 Yaw 제어용)"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        q[3] = cr * cp * cy + sr * sp * sy  # w
        return q

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()