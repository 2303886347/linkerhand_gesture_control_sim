"""将独立关节目标扩展为 RViz 所需的完整 JointState。"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from linkerhand_model_profiles import expand_joint_positions, load_model_profile


class RvizJointStateAdapter(Node):
    def __init__(self):
        super().__init__('linkerhand_rviz_joint_state_adapter')
        self.declare_parameter('input_topic', '/linkerhand/target_joint_states')
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('model_id', 'l30')
        self.declare_parameter('model_side', 'left')
        self.profile = load_model_profile(
            self.get_parameter('model_id').value,
            self.get_parameter('model_side').value,
        )

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.subscription = self.create_subscription(
            JointState, input_topic, self.target_callback, 10
        )
        self.invalid_messages = 0
        self.get_logger().info(
            f'已加载型号 {self.profile.model_id}/{self.profile.side}；'
            f'正在将 {input_topic} 展开为 {len(self.profile.full_joints)} 个关节，'
            f'并发布到 {output_topic}'
        )

    def target_callback(self, message):
        if len(message.name) != len(message.position):
            self.invalid_messages += 1
            if self.invalid_messages == 1 or self.invalid_messages % 30 == 0:
                self.get_logger().warning('收到名称和位置长度不一致的 JointState。')
            return

        positions = expand_joint_positions(
            self.profile, dict(zip(message.name, message.position))
        )

        output = JointState()
        output.header = message.header
        output.name = list(self.profile.full_joints)
        output.position = [
            positions.get(joint, 0.0) for joint in self.profile.full_joints
        ]
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = RvizJointStateAdapter()
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
