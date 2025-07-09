#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class DistinguishListener(Node):
    def __init__(self):
        super().__init__('voice_command_listener')
        self.subscription = self.create_subscription(
            String,
            '/voice_assistant/distinguish',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('🎤 已启动监听 /voice_assistant/distinguish')

    def listener_callback(self, msg):
        command = msg.data.strip().upper()
        self.get_logger().info(f'✅ 收到语音导航指令: {command}')

        if command in ['A', 'B', 'C']:  # 可扩展 C, D 等
            try:
                self.get_logger().info(f'🚀 正在启动导航: {command}')
                subprocess.Popen(['ros2', 'run', 'nav_goal_sender', 'send_nav_goal', command])
            except Exception as e:
                self.get_logger().error(f'❌ 启动失败: {e}')
        else:
            self.get_logger().warn(f'⚠️ 不支持的目标点: {command}')

def main(args=None):
    rclpy.init(args=args)
    node = DistinguishListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

