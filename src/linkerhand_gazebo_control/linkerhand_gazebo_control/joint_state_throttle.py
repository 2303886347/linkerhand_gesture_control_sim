"""把 Gazebo 高频关节状态限制到适合 ROS 可视化的发布频率。"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateThrottle(Node):
    def __init__(self):
        super().__init__('linkerhand_gazebo_joint_state_throttle')
        self.declare_parameter('input_topic', '/gazebo_joint_states_raw')
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('publish_rate', 60.0)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate 必须大于 0')

        self.latest_message = None
        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.subscription = self.create_subscription(
            JointState, input_topic, self.input_callback, 10
        )
        self.timer = self.create_timer(
            1.0 / publish_rate, self.publish_latest
        )
        self.get_logger().info(
            f'正在将 {input_topic} 限制为 {publish_rate:.1f} Hz，'
            f'并发布到 {output_topic}'
        )

    def input_callback(self, message):
        self.latest_message = message

    def publish_latest(self):
        if self.latest_message is not None:
            self.publisher.publish(self.latest_message)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
