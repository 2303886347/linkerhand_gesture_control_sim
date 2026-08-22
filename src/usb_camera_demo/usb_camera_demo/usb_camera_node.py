"""从 USB 摄像头采集画面，并发布为 ROS 2 图像消息。"""

import os

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'usb_camera_optical_frame')
        self.declare_parameter('topic', '/usb_camera/image_raw')
        self.declare_parameter('show_preview', False)

        device = str(self.get_parameter('device').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        topic = str(self.get_parameter('topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.show_preview = bool(self.get_parameter('show_preview').value)

        # 无图形会话时主动关闭预览，避免 OpenCV 因无法连接显示器而退出。
        if self.show_preview and not (
            os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')
        ):
            self.get_logger().warning(
                '未检测到图形显示环境，已自动关闭预览窗口。'
            )
            self.show_preview = False

        source = int(device) if device.isdecimal() else device
        self.capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.capture.release()
            self.get_logger().fatal(
                f'无法打开摄像头 {device}，请检查设备路径和访问权限。'
            )
            raise RuntimeError(f'无法打开摄像头 {device}')

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.failed_reads = 0
        self.timer = self.create_timer(1.0 / max(fps, 1.0), self.publish_frame)

        actual_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'已打开 {device}：{actual_width}x{actual_height}，'
            f'{actual_fps:.1f} FPS；正在发布 {topic}'
        )

    def publish_frame(self):
        ok, frame = self.capture.read()
        if not ok:
            self.failed_reads += 1
            if self.failed_reads == 1 or self.failed_reads % 30 == 0:
                self.get_logger().warning('从摄像头读取画面失败。')
            return

        self.failed_reads = 0
        message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)

        if self.show_preview:
            cv2.imshow('USB Camera', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    def destroy_node(self):
        if hasattr(self, 'capture'):
            # 收到 Ctrl+C 时系统可能再次打断底层设备释放，因此保证退出流程可继续。
            try:
                self.capture.release()
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
    node = None
    exit_code = 0

    try:
        node = UsbCameraNode()
        rclpy.spin(node)
    except RuntimeError:
        exit_code = 1
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
