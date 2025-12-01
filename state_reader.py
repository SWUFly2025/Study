# state_reader.py

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

class DroneStateReader(Node):
    """
    MAVROS 토픽을 구독하여 드론의 현재 상태와 위치를 읽는 ROS 2 노드
    """
    def __init__(self):
        super().__init__('drone_state_reader')
        
        # 1. MAVROS State 토픽 구독
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            10)
        
        # 2. Local Position 토픽 구독
        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.local_pos_cb,
            10)

        # 상태 변수 초기화
        self.current_state = State()
        self.current_pose = PoseStamped()

    def state_cb(self, msg):
        """MAVROS State 토픽 콜백 함수"""
        self.current_state = msg
        self.get_logger().info(f'--- STATE UPDATE ---')
        self.get_logger().info(f'Connected: {self.current_state.connected}')
        self.get_logger().info(f'Armed:     {self.current_state.armed}')
        self.get_logger().info(f'Mode:      {self.current_state.mode}')

    def local_pos_cb(self, msg):
        """Local Position 토픽 콜백 함수"""
        self.current_pose = msg
        pos = msg.pose.position
        
        # 위치 정보를 보기 쉽게 출력
        self.get_logger().info(f'Position:  X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    drone_reader = DroneStateReader()
    
    # 노드 실행 (Ctrl+C를 누를 때까지 반복)
    rclpy.spin(drone_reader)
    
    drone_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()