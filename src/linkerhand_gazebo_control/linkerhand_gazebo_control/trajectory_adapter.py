"""把重定向关节目标转换为 Gazebo 在线插件使用的单点轨迹。"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from linkerhand_gazebo_control.joints import CONTROLLED_JOINTS, FIXED_TARGETS
from linkerhand_retargeting.joints import MIMIC_JOINTS


def build_trajectory(message, duration_seconds):
    """从 JointState 构造单点轨迹；缺少非固定关节时拒绝该帧。"""
    if len(message.name) != len(message.position):
        raise ValueError('关节名称与位置数组长度不一致')

    positions = dict(zip(message.name, message.position))
    missing = [
        joint
        for joint in CONTROLLED_JOINTS
        if joint not in positions
        and joint not in FIXED_TARGETS
        and joint not in MIMIC_JOINTS
    ]
    if missing:
        raise ValueError(f'缺少控制关节：{missing}')

    point = JointTrajectoryPoint()
    point.positions = []
    for joint in CONTROLLED_JOINTS:
        if joint in positions:
            value = positions[joint]
        elif joint in MIMIC_JOINTS:
            value = positions[MIMIC_JOINTS[joint]]
        else:
            value = FIXED_TARGETS[joint]
        point.positions.append(float(value))
    if not all(math.isfinite(value) for value in point.positions):
        raise ValueError('轨迹中包含非有限关节角')

    duration_seconds = max(float(duration_seconds), 0.001)
    whole_seconds = int(duration_seconds)
    point.time_from_start.sec = whole_seconds
    point.time_from_start.nanosec = int(
        round((duration_seconds - whole_seconds) * 1_000_000_000)
    )
    if point.time_from_start.nanosec >= 1_000_000_000:
        point.time_from_start.sec += 1
        point.time_from_start.nanosec -= 1_000_000_000

    trajectory = JointTrajectory()
    trajectory.header = message.header
    trajectory.joint_names = list(CONTROLLED_JOINTS)
    trajectory.points = [point]
    return trajectory


class GazeboTrajectoryAdapter(Node):
    def __init__(self):
        super().__init__('linkerhand_gazebo_trajectory_adapter')
        self.declare_parameter(
            'input_topic', '/linkerhand/target_joint_states'
        )
        self.declare_parameter(
            'command_topic',
            '/gazebo_joint_trajectory',
        )
        self.declare_parameter('trajectory_duration', 0.15)

        input_topic = str(self.get_parameter('input_topic').value)
        command_topic = str(self.get_parameter('command_topic').value)
        self.trajectory_duration = float(
            self.get_parameter('trajectory_duration').value
        )
        self.publisher = self.create_publisher(
            JointTrajectory, command_topic, 10
        )
        self.subscription = self.create_subscription(
            JointState, input_topic, self.target_callback, 10
        )
        self.invalid_messages = 0
        self.get_logger().info(
            f'正在将 {input_topic} 转换为 Gazebo 轨迹 {command_topic}'
        )

    def target_callback(self, message):
        try:
            trajectory = build_trajectory(
                message, self.trajectory_duration
            )
        except ValueError as error:
            self.invalid_messages += 1
            if self.invalid_messages == 1 or self.invalid_messages % 30 == 0:
                self.get_logger().warning(f'忽略无效关节目标：{error}')
            return
        self.publisher.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboTrajectoryAdapter()
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
