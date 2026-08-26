"""将 MediaPipe 人手角度映射为 Linker Hand 独立关节目标。"""

import math
import time

from hand_pose_msgs.msg import HandPose
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from linkerhand_model_profiles import load_model_profile
from linkerhand_retargeting.filters import (
    JointDeadbandHysteresis,
    exponential_moving_average,
    move_towards,
)
from linkerhand_retargeting.mapping import (
    JointMapping,
    angle_to_radians,
    radians_to_angle,
)


class LinkerHandRetargetingNode(Node):
    def __init__(self):
        super().__init__('linkerhand_retargeting')

        self.declare_parameter('input_topic', '/mediapipe/hand_pose')
        self.declare_parameter('output_topic', '/linkerhand/target_joint_states')
        self.declare_parameter('status_topic', '/linkerhand/retargeting_status')
        self.declare_parameter('accepted_hand', 'any')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('filter_alpha', 0.35)
        self.declare_parameter('hold_timeout', 0.80)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_joint_velocity', 3.0)
        self.declare_parameter('return_joint_velocity', 0.8)
        self.declare_parameter('mapping_angle_unit', 'deg')
        self.declare_parameter('joint_deadband.enabled', True)
        self.declare_parameter('joint_deadband.start_moving_deg', 1.5)
        self.declare_parameter('joint_deadband.stop_moving_deg', 0.5)
        self.declare_parameter('joint_deadband.settle_frames', 3)
        self.declare_parameter('joint_deadband.thumb_start_moving_deg', 2.5)
        self.declare_parameter('joint_deadband.thumb_stop_moving_deg', 0.8)

        self.declare_parameter('model_id', 'l30')
        self.declare_parameter('model_side', 'left')
        self.profile = load_model_profile(
            self.get_parameter('model_id').value,
            self.get_parameter('model_side').value,
        )

        self.mappings, self.safe_pose = self._declare_and_load_mappings()
        self.deadband_enabled = bool(
            self.get_parameter('joint_deadband.enabled').value
        )
        self.deadband_filters = self._load_deadband_filters()
        self.accepted_hand = str(self.get_parameter('accepted_hand').value).lower()
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.filter_alpha = float(self.get_parameter('filter_alpha').value)
        self.hold_timeout = float(self.get_parameter('hold_timeout').value)
        self.publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.max_joint_velocity = float(
            self.get_parameter('max_joint_velocity').value
        )
        self.return_joint_velocity = float(
            self.get_parameter('return_joint_velocity').value
        )

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.subscription = self.create_subscription(
            HandPose, input_topic, self.hand_pose_callback, 10
        )

        self.filtered_target = dict(self.safe_pose)
        self.current_output = dict(self.safe_pose)
        self.last_valid_time = None
        self.last_message_time = None
        self.last_observation_valid = False
        self.last_timer_time = time.monotonic()
        self.current_status = ''
        self.missing_angle_warnings = 0
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_command)

        driven_count = sum(mapping.driven for mapping in self.mappings.values())
        self.get_logger().info(
            f'已加载型号 {self.profile.model_id}/{self.profile.side}；'
            f'正在订阅 {input_topic}；启用 {driven_count} 个映射关节；'
            f'映射配置单位={self.get_parameter("mapping_angle_unit").value}；'
            f'关节死区={"启用" if self.deadband_enabled else "关闭"}；'
            f'输出 {output_topic}'
        )

    def _load_deadband_filters(self):
        if not self.deadband_enabled:
            return {}

        settle_frames = int(
            self.get_parameter('joint_deadband.settle_frames').value
        )
        finger_start = math.radians(
            float(self.get_parameter('joint_deadband.start_moving_deg').value)
        )
        finger_stop = math.radians(
            float(self.get_parameter('joint_deadband.stop_moving_deg').value)
        )
        thumb_start = math.radians(
            float(
                self.get_parameter(
                    'joint_deadband.thumb_start_moving_deg'
                ).value
            )
        )
        thumb_stop = math.radians(
            float(
                self.get_parameter(
                    'joint_deadband.thumb_stop_moving_deg'
                ).value
            )
        )

        filters = {}
        for joint in self.profile.active_joints:
            is_thumb = joint in self.profile.thumb_joints
            filters[joint] = JointDeadbandHysteresis(
                initial_value=self.safe_pose[joint],
                start_threshold=thumb_start if is_thumb else finger_start,
                stop_threshold=thumb_stop if is_thumb else finger_stop,
                settle_frames=settle_frames,
            )
        return filters

    def _declare_and_load_mappings(self):
        mappings = {}
        safe_pose = {}
        mapping_angle_unit = str(
            self.get_parameter('mapping_angle_unit').value
        ).lower()
        # 提前校验单位，避免部分关节加载后才发现配置错误。
        angle_to_radians(0.0, mapping_angle_unit)

        for joint in self.profile.active_joints:
            defaults = self.profile.mapping_defaults[joint]
            source = defaults.source_angle
            input_min = defaults.input_min
            input_max = defaults.input_max
            output_min = defaults.output_min
            output_max = defaults.output_max
            invert = defaults.invert
            fixed = defaults.fixed_position
            safe_position = defaults.safe_position
            prefix = f'mapping.{joint}'
            self.declare_parameter(f'{prefix}.source', source)
            self.declare_parameter(
                f'{prefix}.input_min',
                radians_to_angle(input_min, mapping_angle_unit),
            )
            self.declare_parameter(
                f'{prefix}.input_max',
                radians_to_angle(input_max, mapping_angle_unit),
            )
            self.declare_parameter(
                f'{prefix}.output_min',
                radians_to_angle(output_min, mapping_angle_unit),
            )
            self.declare_parameter(
                f'{prefix}.output_max',
                radians_to_angle(output_max, mapping_angle_unit),
            )
            self.declare_parameter(f'{prefix}.invert', invert)
            self.declare_parameter(
                f'{prefix}.fixed_position',
                radians_to_angle(fixed, mapping_angle_unit),
            )
            self.declare_parameter(
                f'{prefix}.safe_position',
                radians_to_angle(safe_position, mapping_angle_unit),
            )

            joint_min, joint_max = self.profile.joint_limits[joint]
            mappings[joint] = JointMapping(
                target_joint=joint,
                source_angle=str(self.get_parameter(f'{prefix}.source').value),
                input_min=angle_to_radians(
                    self.get_parameter(f'{prefix}.input_min').value,
                    mapping_angle_unit,
                ),
                input_max=angle_to_radians(
                    self.get_parameter(f'{prefix}.input_max').value,
                    mapping_angle_unit,
                ),
                output_min=angle_to_radians(
                    self.get_parameter(f'{prefix}.output_min').value,
                    mapping_angle_unit,
                ),
                output_max=angle_to_radians(
                    self.get_parameter(f'{prefix}.output_max').value,
                    mapping_angle_unit,
                ),
                joint_min=joint_min,
                joint_max=joint_max,
                invert=bool(self.get_parameter(f'{prefix}.invert').value),
                fixed_position=angle_to_radians(
                    self.get_parameter(f'{prefix}.fixed_position').value,
                    mapping_angle_unit,
                ),
            )
            configured_safe_position = angle_to_radians(
                self.get_parameter(f'{prefix}.safe_position').value,
                mapping_angle_unit,
            )
            safe_pose[joint] = min(
                max(configured_safe_position, joint_min), joint_max
            )
        return mappings, safe_pose

    def hand_pose_callback(self, message):
        now = time.monotonic()
        self.last_message_time = now
        hand_allowed = (
            self.accepted_hand == 'any'
            or message.handedness.lower() == self.accepted_hand
        )
        valid = (
            message.detected
            and message.confidence >= self.confidence_threshold
            and hand_allowed
        )
        self.last_observation_valid = valid
        if not valid:
            return

        source_angles = dict(zip(message.joint_names, message.joint_angles))
        raw_target = {}
        missing = []
        for joint, mapping in self.mappings.items():
            if mapping.driven:
                source_value = source_angles.get(mapping.source_angle)
                if source_value is None or not math.isfinite(source_value):
                    missing.append(mapping.source_angle)
                    continue
                raw_target[joint] = mapping.map_angle(source_value)
            else:
                raw_target[joint] = mapping.map_angle(mapping.input_min)

        if missing:
            self.missing_angle_warnings += 1
            if self.missing_angle_warnings == 1 or self.missing_angle_warnings % 30 == 0:
                self.get_logger().warning(
                    f'MediaPipe 消息缺少角度：{sorted(set(missing))}'
                )
            self.last_observation_valid = False
            return

        for joint in self.profile.active_joints:
            stabilized_target = raw_target[joint]
            if self.deadband_enabled:
                stabilized_target = self.deadband_filters[joint].update(
                    stabilized_target
                )
            self.filtered_target[joint] = exponential_moving_average(
                self.filtered_target[joint],
                stabilized_target,
                self.filter_alpha,
            )
        self.last_valid_time = now

    def _set_status(self, status):
        if status == self.current_status:
            return
        self.current_status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        labels = {
            'waiting': '等待首次有效手部姿态',
            'tracking': '正在跟踪并输出关节目标',
            'holding': '短暂丢失，保持上一姿态',
            'returning': '跟踪超时，返回安全开掌姿态',
        }
        self.get_logger().info(labels[status])

    def publish_command(self):
        now = time.monotonic()
        delta_time = min(max(now - self.last_timer_time, 0.0), 0.2)
        self.last_timer_time = now

        if self.last_valid_time is None:
            desired = self.safe_pose
            status = 'waiting'
        else:
            valid_age = now - self.last_valid_time
            message_age = (
                math.inf
                if self.last_message_time is None
                else now - self.last_message_time
            )
            if self.last_observation_valid and message_age <= self.hold_timeout:
                desired = self.filtered_target
                status = 'tracking'
            elif valid_age <= self.hold_timeout:
                desired = self.filtered_target
                status = 'holding'
            else:
                desired = self.safe_pose
                status = 'returning'

        velocity_limit = (
            self.return_joint_velocity
            if status == 'returning'
            else self.max_joint_velocity
        )
        maximum_step = velocity_limit * delta_time
        for joint in self.profile.active_joints:
            self.current_output[joint] = move_towards(
                self.current_output[joint], desired[joint], maximum_step
            )

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.profile.active_joints)
        message.position = [
            self.current_output[joint] for joint in self.profile.active_joints
        ]
        self.publisher.publish(message)
        self._set_status(status)


def main(args=None):
    rclpy.init(args=args)
    node = LinkerHandRetargetingNode()
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
