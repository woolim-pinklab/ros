import math
import time

import rclpy as rp
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from my_first_package_msgs.action import DistTurtle


class DistTurtleServer(Node):
    def __init__(self):
        super().__init__('dist_turtle_action_server')

        cb_group = ReentrantCallbackGroup()

        self.current_pose = None
        self.previous_pose = None

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10,
            callback_group=cb_group)
        self.action_server = ActionServer(
            self, DistTurtle, 'dist_turtle', self.execute_callback,
            callback_group=cb_group)

    def pose_callback(self, msg):
        self.current_pose = msg

    def calc_diff_pose(self):
        if self.previous_pose is None:
            self.previous_pose = self.current_pose

        diff = math.sqrt(
            (self.current_pose.x - self.previous_pose.x) ** 2
            + (self.current_pose.y - self.previous_pose.y) ** 2)
        self.previous_pose = self.current_pose

        return diff

    def execute_callback(self, goal_handle):
        request = goal_handle.request

        cmd = Twist()
        cmd.linear.x = request.linear_x
        cmd.angular.z = request.angular_z

        # 첫 pose가 도착해야 출발 위치를 알 수 있다
        while self.current_pose is None:
            time.sleep(0.1)

        feedback_msg = DistTurtle.Feedback()
        total_dist = 0.0
        self.previous_pose = self.current_pose  # 이번 goal의 출발 기준점

        while total_dist < request.dist:
            self.publisher.publish(cmd)
            time.sleep(0.01)

            total_dist += self.calc_diff_pose()

            feedback_msg.remained_dist = request.dist - total_dist
            goal_handle.publish_feedback(feedback_msg)

        self.publisher.publish(Twist())  # 목표 도달, 정지
        goal_handle.succeed()

        result = DistTurtle.Result()
        result.pos_x = self.current_pose.x
        result.pos_y = self.current_pose.y
        result.pos_theta = self.current_pose.theta
        result.result_dist = total_dist

        return result


def main(args=None):
    rp.init(args=args)

    node = DistTurtleServer()
    executor = MultiThreadedExecutor()

    try:
        rp.spin(node, executor=executor)
    finally:
        node.destroy_node()
        rp.shutdown()


if __name__ == '__main__':
    main()
