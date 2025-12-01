# command_sender.py

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

class DroneCommander(Node):
    """
    MAVROS 서비스를 사용하여 드론에 명령을 보내는 ROS 2 노드
    """
    def __init__(self):
        super().__init__('drone_commander')

        # 1. 서비스 클라이언트 생성
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # 2. 상태 구독 (명령을 보내기 전에 현재 상태를 알아야 함)
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.current_state = State()

        # 3. 타이머 설정 (주기적으로 상태 확인 및 명령 시도)
        self.timer = self.create_timer(1.0, self.timer_cb) # 1초마다 타이머 실행
        self.get_logger().info('Drone Commander Node Initialized. Waiting for connection...')

    def state_cb(self, msg):
        """MAVROS State 토픽 콜백 함수"""
        self.current_state = msg

    def timer_cb(self):
        """1초마다 실행되는 타이머 콜백"""
        
        # 1. 연결 상태 확인
        if not self.current_state.connected:
            self.get_logger().info('Waiting for connection to Flight Controller...')
            return
        
        # 2. 비행 모드 설정 시도 (예: GUIDED 모드)
        if self.current_state.mode != 'GUIDED':
            self.call_set_mode('GUIDED')
            return

        # 3. 시동 시도
        if not self.current_state.armed and self.current_state.mode == 'GUIDED':
            self.call_arming(True)
            
        # 4. 최종 상태 확인
        if self.current_state.armed and self.current_state.mode == 'GUIDED':
            self.get_logger().info('SUCCESS! Drone is Armed and in GUIDED Mode.')
            # 시동 및 모드 설정이 완료되면 타이머를 중지하고 다음 단계(이륙/이동)로 넘어가야 합니다.
            # self.timer.cancel() 
            # 다음 단계 코드 (예: 이륙/경로 이동)를 여기에 추가합니다.


    def call_arming(self, arm_value):
        """시동/시동 해제 서비스 호출"""
        if self.arming_client.wait_for_service(timeout_sec=1.0):
            request = CommandBool.Request()
            request.value = arm_value
            
            future = self.arming_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result().success:
                self.get_logger().info(f'Arming request sent: {arm_value}. Success: {future.result().success}')
            else:
                self.get_logger().error('Arming failed (Service returned False)')
        else:
            self.get_logger().error('/mavros/cmd/arming service not available')


    def call_set_mode(self, mode_name):
        """비행 모드 설정 서비스 호출"""
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            request = SetMode.Request()
            request.custom_mode = mode_name # 'GUIDED', 'OFFBOARD', 'POSCTL' 등
            
            future = self.set_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result().mode_sent:
                self.get_logger().info(f'Mode change request sent: {mode_name}. Success: {future.result().mode_sent}')
            else:
                self.get_logger().error(f'Mode change to {mode_name} failed.')
        else:
            self.get_logger().error('/mavros/set_mode service not available')


def main(args=None):
    rclpy.init(args=args)
    commander = DroneCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()