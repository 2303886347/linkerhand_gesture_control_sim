"""订阅 ROS 2 图像，使用 MediaPipe 输出关键点和人体手部关节角。"""

import os
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from hand_pose_msgs.msg import HandLandmark, HandPose
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

from mediapipe_hand_pose.geometry import calculate_joint_angles
from mediapipe_hand_pose.one_euro_filter import OneEuroFilter


class MediaPipeHandPoseNode(Node):
    def __init__(self):
        super().__init__('mediapipe_hand_pose')

        self.declare_parameter('input_topic', '/usb_camera/image_raw')
        self.declare_parameter('pose_topic', '/mediapipe/hand_pose')
        self.declare_parameter('angles_topic', '/mediapipe/human_joint_angles')
        self.declare_parameter('debug_image_topic', '/mediapipe/debug_image')
        self.declare_parameter('target_hand', 'any')
        self.declare_parameter('input_mirrored', False)
        self.declare_parameter('model_complexity', 1)
        self.declare_parameter('max_num_hands', 2)
        self.declare_parameter('min_detection_confidence', 0.6)
        self.declare_parameter('min_tracking_confidence', 0.6)
        self.declare_parameter('max_processing_fps', 15.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('preview_window_name', 'MediaPipe Hand Pose')
        self.declare_parameter('mirror_preview', True)
        self.declare_parameter('use_one_euro_filter', True)
        self.declare_parameter('one_euro_min_cutoff', 0.8)
        self.declare_parameter('one_euro_beta', 0.3)
        self.declare_parameter('one_euro_derivative_cutoff', 1.0)
        self.declare_parameter('one_euro_reset_timeout', 0.5)

        input_topic = str(self.get_parameter('input_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        angles_topic = str(self.get_parameter('angles_topic').value)
        debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.target_hand = str(self.get_parameter('target_hand').value).lower()
        self.input_mirrored = bool(self.get_parameter('input_mirrored').value)
        self.max_processing_fps = float(self.get_parameter('max_processing_fps').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.show_preview = bool(self.get_parameter('show_preview').value)
        self.preview_window_name = str(
            self.get_parameter('preview_window_name').value
        )
        self.mirror_preview = bool(self.get_parameter('mirror_preview').value)
        self.use_one_euro_filter = bool(
            self.get_parameter('use_one_euro_filter').value
        )
        self.one_euro_reset_timeout = max(
            float(self.get_parameter('one_euro_reset_timeout').value), 0.0
        )

        filter_parameters = {
            'min_cutoff': float(
                self.get_parameter('one_euro_min_cutoff').value
            ),
            'beta': float(self.get_parameter('one_euro_beta').value),
            'derivative_cutoff': float(
                self.get_parameter('one_euro_derivative_cutoff').value
            ),
        }
        self.image_landmark_filter = OneEuroFilter(**filter_parameters)
        self.joint_angle_filter = OneEuroFilter(**filter_parameters)
        self.filtered_angle_names = None
        self.last_filtered_handedness = None
        self.last_detection_time = None

        if self.target_hand not in {'any', 'left', 'right'}:
            self.get_logger().warning(
                f'未知 target_hand={self.target_hand}，将使用 any。'
            )
            self.target_hand = 'any'

        if self.show_preview and not (
            os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')
        ):
            self.get_logger().warning('未检测到图形显示环境，已自动关闭预览窗口。')
            self.show_preview = False

        self.bridge = CvBridge()
        self.pose_publisher = self.create_publisher(HandPose, pose_topic, 10)
        self.angles_publisher = self.create_publisher(JointState, angles_topic, 10)
        self.debug_publisher = self.create_publisher(
            Image, debug_image_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, qos_profile_sensor_data
        )

        self.mp_hands = mp.solutions.hands
        self.drawing_utils = mp.solutions.drawing_utils
        self.drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=int(self.get_parameter('max_num_hands').value),
            model_complexity=int(self.get_parameter('model_complexity').value),
            min_detection_confidence=float(
                self.get_parameter('min_detection_confidence').value
            ),
            min_tracking_confidence=float(
                self.get_parameter('min_tracking_confidence').value
            ),
        )

        self.last_process_time = 0.0
        self.convert_failures = 0
        self.geometry_failures = 0
        self.geometry_fallbacks = 0
        self.get_logger().info(
            f'正在订阅 {input_topic}；目标手={self.target_hand}；'
            f'最大处理帧率={self.max_processing_fps:.1f} FPS；'
            f'One Euro 滤波={self.use_one_euro_filter}'
        )

    def _correct_handedness(self, raw_label):
        # MediaPipe Hands 假设输入是自拍镜像；普通相机原图需要交换左右标签。
        if self.input_mirrored:
            return raw_label
        return {'Left': 'Right', 'Right': 'Left'}.get(raw_label, raw_label)

    def _select_hand(self, results):
        if not results.multi_hand_landmarks or not results.multi_handedness:
            return None

        candidates = []
        for index, (landmarks, handedness) in enumerate(
            zip(results.multi_hand_landmarks, results.multi_handedness)
        ):
            classification = handedness.classification[0]
            label = self._correct_handedness(classification.label)
            if self.target_hand != 'any' and label.lower() != self.target_hand:
                continue

            world_landmarks = None
            if results.multi_hand_world_landmarks:
                world_landmarks = results.multi_hand_world_landmarks[index]
            candidates.append(
                (float(classification.score), label, landmarks, world_landmarks)
            )

        return max(candidates, key=lambda item: item[0]) if candidates else None

    @staticmethod
    def _to_landmark_messages(landmarks):
        if landmarks is None:
            return []

        messages = []
        for point in landmarks.landmark:
            message = HandLandmark()
            message.x = float(point.x)
            message.y = float(point.y)
            message.z = float(point.z)
            messages.append(message)
        return messages

    @staticmethod
    def _to_numpy(landmarks):
        return np.asarray(
            [[point.x, point.y, point.z] for point in landmarks.landmark],
            dtype=float,
        )

    @staticmethod
    def _calculate_angles_with_fallback(image_points, world_points=None):
        """优先使用 3D 关键点，退化时回退到仍可见的图像关键点。"""
        if world_points is None:
            return calculate_joint_angles(image_points), False

        try:
            return calculate_joint_angles(world_points), False
        except ValueError as world_error:
            try:
                return calculate_joint_angles(image_points), True
            except ValueError as image_error:
                raise ValueError(
                    f'3D 关键点失败：{world_error}；'
                    f'图像关键点也失败：{image_error}'
                ) from image_error

    def _publish_debug_frame(self, frame, header):
        if self.publish_debug_image:
            debug_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_message.header = header
            self.debug_publisher.publish(debug_message)

        if self.show_preview:
            cv2.imshow(self.preview_window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    def _prepare_debug_frame(self, frame, landmarks=None):
        """生成显示帧；镜像只作用于调试画面，不改变感知数据。"""
        if not self.mirror_preview:
            return frame.copy(), landmarks

        debug_frame = cv2.flip(frame, 1)
        if landmarks is None:
            return debug_frame, None

        debug_landmarks = type(landmarks)()
        debug_landmarks.CopyFrom(landmarks)
        for point in debug_landmarks.landmark:
            point.x = 1.0 - point.x
        return debug_frame, debug_landmarks

    @staticmethod
    def _replace_landmark_coordinates(landmarks, coordinates):
        filtered_landmarks = type(landmarks)()
        filtered_landmarks.CopyFrom(landmarks)
        for point, coordinate in zip(
            filtered_landmarks.landmark, coordinates
        ):
            point.x = float(coordinate[0])
            point.y = float(coordinate[1])
            point.z = float(coordinate[2])
        return filtered_landmarks

    def _reset_one_euro_filters(self):
        self.image_landmark_filter.reset()
        self.joint_angle_filter.reset()
        self.filtered_angle_names = None
        self.last_filtered_handedness = None

    def _filter_image_landmarks(self, landmarks, timestamp):
        if not self.use_one_euro_filter:
            return landmarks
        coordinates = self.image_landmark_filter.filter(
            self._to_numpy(landmarks), timestamp
        )
        return self._replace_landmark_coordinates(landmarks, coordinates)

    def _filter_joint_angles(self, angles, timestamp):
        if not angles or not self.use_one_euro_filter:
            return angles

        angle_names = tuple(angles.keys())
        if angle_names != self.filtered_angle_names:
            self.joint_angle_filter.reset()
            self.filtered_angle_names = angle_names

        filtered_values = self.joint_angle_filter.filter(
            np.asarray(list(angles.values()), dtype=float), timestamp
        )
        return dict(zip(angle_names, filtered_values.tolist()))

    @staticmethod
    def _draw_angle_summary(frame, angles):
        summary = [
            ('Thumb Abd', angles.get('thumb_cmc_abduction', 0.0)),
            ('Thumb CMC', angles.get('thumb_cmc_flexion', 0.0)),
            ('Thumb MCP', angles.get('thumb_mcp_flexion', 0.0)),
            ('Index', angles.get('index_pip_flexion', 0.0)),
            ('Middle', angles.get('middle_pip_flexion', 0.0)),
            ('Ring', angles.get('ring_pip_flexion', 0.0)),
            ('Pinky', angles.get('pinky_pip_flexion', 0.0)),
        ]
        for row, (name, angle) in enumerate(summary, start=1):
            cv2.putText(
                frame,
                f'{name}: {np.degrees(angle):5.1f} deg',
                (12, 28 + row * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (50, 230, 255),
                1,
                cv2.LINE_AA,
            )

    def image_callback(self, image_message):
        now = time.monotonic()
        if self.max_processing_fps > 0.0:
            minimum_interval = 1.0 / self.max_processing_fps
            if now - self.last_process_time < minimum_interval:
                return
        self.last_process_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.convert_failures += 1
            if self.convert_failures == 1 or self.convert_failures % 30 == 0:
                self.get_logger().error(f'ROS 图像转换失败：{error}')
            return

        start_time = time.perf_counter()
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb_frame.flags.writeable = False
        results = self.hands.process(rgb_frame)
        selected = self._select_hand(results)

        pose_message = HandPose()
        pose_message.header = image_message.header
        debug_frame, debug_landmarks = self._prepare_debug_frame(frame)

        if selected is None:
            if (
                self.last_detection_time is not None
                and now - self.last_detection_time
                >= self.one_euro_reset_timeout
            ):
                self._reset_one_euro_filters()
                self.last_detection_time = None
            pose_message.detected = False
            pose_message.processing_time_ms = float(
                (time.perf_counter() - start_time) * 1000.0
            )
            self.pose_publisher.publish(pose_message)
            cv2.putText(
                debug_frame,
                'No target hand detected',
                (12, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (40, 40, 230),
                2,
                cv2.LINE_AA,
            )
            self._publish_debug_frame(debug_frame, image_message.header)
            return

        confidence, handedness, landmarks, world_landmarks = selected
        if (
            self.last_filtered_handedness is not None
            and handedness != self.last_filtered_handedness
        ):
            self._reset_one_euro_filters()
        self.last_filtered_handedness = handedness
        self.last_detection_time = now

        filtered_landmarks = self._filter_image_landmarks(landmarks, now)
        debug_frame, debug_landmarks = self._prepare_debug_frame(
            frame, filtered_landmarks
        )
        try:
            world_points = (
                None
                if world_landmarks is None
                else self._to_numpy(world_landmarks)
            )
            angles, used_image_fallback = self._calculate_angles_with_fallback(
                self._to_numpy(filtered_landmarks), world_points
            )
            if used_image_fallback:
                self.geometry_fallbacks += 1
                if (
                    self.geometry_fallbacks == 1
                    or self.geometry_fallbacks % 30 == 0
                ):
                    self.get_logger().warning(
                        '3D 关键点暂时退化，已回退到图像关键点计算关节角'
                    )
            angles = self._filter_joint_angles(angles, now)
        except ValueError as error:
            self.geometry_failures += 1
            if self.geometry_failures == 1 or self.geometry_failures % 30 == 0:
                self.get_logger().warning(f'关节角计算失败：{error}')
            angles = {}

        pose_message.detected = True
        pose_message.handedness = handedness
        pose_message.confidence = confidence
        pose_message.landmarks = self._to_landmark_messages(filtered_landmarks)
        pose_message.world_landmarks = self._to_landmark_messages(world_landmarks)
        pose_message.joint_names = list(angles.keys())
        pose_message.joint_angles = list(angles.values())
        pose_message.processing_time_ms = float(
            (time.perf_counter() - start_time) * 1000.0
        )
        self.pose_publisher.publish(pose_message)

        if angles:
            joint_state = JointState()
            joint_state.header = image_message.header
            joint_state.name = list(angles.keys())
            joint_state.position = list(angles.values())
            self.angles_publisher.publish(joint_state)

        self.drawing_utils.draw_landmarks(
            debug_frame,
            debug_landmarks,
            self.mp_hands.HAND_CONNECTIONS,
            self.drawing_styles.get_default_hand_landmarks_style(),
            self.drawing_styles.get_default_hand_connections_style(),
        )
        cv2.putText(
            debug_frame,
            f'{handedness} {confidence:.2f}  {pose_message.processing_time_ms:.1f} ms',
            (12, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (60, 230, 60),
            2,
            cv2.LINE_AA,
        )
        self._draw_angle_summary(debug_frame, angles)
        self._publish_debug_frame(debug_frame, image_message.header)

    def destroy_node(self):
        if hasattr(self, 'hands'):
            # Ctrl+C 可能在 MediaPipe 图释放期间再次到达，保证退出流程继续执行。
            try:
                self.hands.close()
            except KeyboardInterrupt:
                pass
        if self.show_preview:
            try:
                cv2.destroyAllWindows()
            except KeyboardInterrupt:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeHandPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
