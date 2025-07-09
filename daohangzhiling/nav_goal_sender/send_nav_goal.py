#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler

# 预定义两个目标点 A 和 B
WAYPOINTS = {
    'A': [0.20, -0.01, 0.0],   # A 点: x, y, yaw(角度)
    'B': [3.64, 1.85, 0.0],   # B 点
    'C': [3.70, -1.69, 0.0]
}

class NavGoalSender(Node):
    def __init__(self, target_key: str):
        super().__init__('nav_goal_sender')

        if target_key not in WAYPOINTS:
            raise ValueError(f"目标应为 {list(WAYPOINTS.keys())}，而不是 '{target_key}'")

        self.target_key = target_key
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 设置定时器，发送一次导航请求
        self.timer = self.create_timer(0.1, self.send_goal_once)

    def send_goal_once(self):
        if not self.ac.wait_for_server(timeout_sec=0.0):
            return  # 等待服务器准备好

        self.timer.cancel()  # 只发送一次目标

        x, y, yaw_deg = WAYPOINTS[self.target_key]

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        q = quaternion_from_euler(0.0, 0.0, yaw_deg * 3.1416 / 180.0)
        pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
        pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f'📤 正在导航到 {self.target_key}: x={x:.2f}, y={y:.2f}')
        future = self.ac.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 目标被拒绝')
            rclpy.shutdown()
            return
        self.get_logger().info('✅ 目标已接受，正在前往…')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        result = future.result().result
        self.get_logger().info('🏁 到达目标点')
        rclpy.shutdown()

def main():
    rclpy.init()
    # 获取命令行参数：A 或 B
    target = sys.argv[1].upper() if len(sys.argv) > 1 else 'A'
    node = NavGoalSender(target)
    rclpy.spin(node)

