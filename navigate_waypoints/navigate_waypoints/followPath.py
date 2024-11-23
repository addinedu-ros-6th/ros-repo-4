import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class FollowPathTest(Node):
    def __init__(self, yaml_path):
        super().__init__('follow_path_test_node')
        
        # BasicNavigator 초기화
        self.navigator = BasicNavigator()
        
        # Nav2 활성화 확인
        # self.get_logger().info("Nav2 활성화 대기 중...")
        # self.navigator.waitUntilNav2Active()
        # self.get_logger().info("Nav2가 활성화되었습니다.")
        
        # YAML 파일에서 경유지와 목적지 로드
        self.path = self.load_points_from_yaml(yaml_path)
        
        # followPath 명령 실행
        self.get_logger().info("경로가 설정되었습니다. followPath 실행 중...")
        self.navigator.followPath(self.path)
        
        # 경로 실행 상태 확인
        self.check_path_execution()

    def load_points_from_yaml(self, yaml_path):
        # YAML 파일에서 경유지와 목적지 정보를 불러와 경로를 생성
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
        
        waypoints = data['waypoints']
        goal_points = data['goal_points']
        
        path = Path()
        path.header.frame_id = "map"

        # 경유지 1번 추가
        waypoint1 = PoseStamped()
        waypoint1.header.frame_id = "map"
        waypoint1.pose.position.x = waypoints[0]['point']['x']
        waypoint1.pose.position.y = waypoints[0]['point']['y']
        waypoint1.pose.orientation.w = 1.0
        path.poses.append(waypoint1)

        # 경유지 2번 추가
        waypoint2 = PoseStamped()
        waypoint2.header.frame_id = "map"
        waypoint2.pose.position.x = waypoints[1]['point']['x']
        waypoint2.pose.position.y = waypoints[1]['point']['y']
        waypoint2.pose.orientation.w = 1.0
        path.poses.append(waypoint2)

        # 최종 목표 5번 추가
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = goal_points[4]['point']['x']
        goal.pose.position.y = goal_points[4]['point']['y']
        goal.pose.orientation.w = 1.0
        path.poses.append(goal)

        return path

    def check_path_execution(self):
        # 경로 실행 상태 확인
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # 사용 가능한 속성 확인 후 로깅
                self.get_logger().info("경로를 따라 이동 중입니다...")
            time.sleep(1)  # 체크 주기 (1초 간격)

        # 경로 완료 상태 확인
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("목표 지점에 성공적으로 도달했습니다.")
        else:
            self.get_logger().error("목표 지점에 도달하지 못했습니다.")

def main(args=None):
    rclpy.init(args=args)

    # YAML 파일 경로 설정
    yaml_path = '/home/sehyung/minibot_ws/src/pinklab_minibot_robot/navigate_waypoints/navigate_waypoints/points.yaml'
    follow_path_node = FollowPathTest(yaml_path)

    try:
        rclpy.spin(follow_path_node)
    except KeyboardInterrupt:
        follow_path_node.get_logger().info("사용자에 의해 종료되었습니다.")
    finally:
        follow_path_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
