import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        # 建一个发布者：往 /turtle1/cmd_vel 发 Twist 消息
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 每 0.1 秒执行一次 tick
        self.create_timer(0.1, self.tick)

    def tick(self):
        m = Twist()
        m.linear.x = 1.0      # 前进 1.0 米/秒
        m.angular.z = 1.0     # 左转 1.0 弧度/秒
        self.pub.publish(m)
        self.get_logger().info('正在画圆...', throttle_duration_sec=1)


def main():
    rclpy.init()
    rclpy.spin(CircleMover())


if __name__ == '__main__':
    main()
