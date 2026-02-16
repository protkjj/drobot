import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')
        # Nav2의 'NavigateToPose' 액션 서버와 통신할 클라이언트 생성
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # 목표 좌표 설정
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # 정면 바라보기

        self.get_logger().info(f'목표 지점 전송 중: x={x}, y={y}')
        
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = GoalNavigator()
    # 예시: (2.0, 1.0) 좌표로 이동 명령
    future = navigator.send_goal(2.0, 1.0)
    rclpy.spin_until_future_complete(navigator, future)
    rclpy.shutdown()
