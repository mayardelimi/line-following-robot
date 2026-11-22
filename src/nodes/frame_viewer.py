#!/usr/bin/env python3
"""Live view and frame saving for either controller.

Shows two windows: the raw camera image and the controller's debug overlay.
Click a window, then press:
    s   save the current raw frame + overlay
    r   start / stop recording (one frame out of every `save_every`)

Files go to `save_dir` (default ~/line_follower_frames) as 00001_raw.png, 00001_overlay.png, ...
Without a display (SSH) the windows are skipped; recording still works:
    ros2 param set /frame_viewer record true
"""
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class FrameViewer(Node):
    def __init__(self):
        super().__init__('frame_viewer')

        self.declare_parameter('save_dir', os.path.expanduser('~/line_follower_frames'))
        self.declare_parameter('save_every', 10)
        self.declare_parameter('record', False)

        self.bridge = CvBridge()
        self.raw = None
        self.overlay = None
        self.frame_count = 0
        self.saved_count = 0
        self.has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        if not self.has_display:
            self.get_logger().warn('No display: windows disabled (use rqt_image_view, or record with the "record" parameter)')

        image_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, 'camera_sensor/image_raw', self.on_raw, image_qos)
        self.create_subscription(Image, 'debug/overlay', self.on_overlay, image_qos)
        self.create_timer(1.0 / 30.0, self.refresh)

    def on_raw(self, msg):
        self.raw = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.frame_count += 1
        if self.get_parameter('record').value and self.frame_count % self.get_parameter('save_every').value == 0:
            self.save()

    def on_overlay(self, msg):
        self.overlay = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def save(self):
        if self.raw is None:
            return
        folder = os.path.expanduser(self.get_parameter('save_dir').value)
        os.makedirs(folder, exist_ok=True)
        self.saved_count += 1
        base = os.path.join(folder, f'{self.saved_count:05d}')
        cv2.imwrite(base + '_raw.png', self.raw)
        if self.overlay is not None:
            cv2.imwrite(base + '_overlay.png', self.overlay)
        self.get_logger().info(f'Saved frame {self.saved_count} -> {folder}')

    def refresh(self):
        if not self.has_display:
            return
        if self.raw is not None:
            cv2.imshow('raw camera', cv2.resize(self.raw, (400, 300)))
        if self.overlay is not None:
            cv2.imshow('controller view', self.overlay)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self.save()
        elif key == ord('r'):
            recording = not self.get_parameter('record').value
            self.set_parameters([Parameter('record', Parameter.Type.BOOL, recording)])
            self.get_logger().info(f"Recording {'ON' if recording else 'OFF'}")


def main(args=None):
    rclpy.init(args=args)
    node = FrameViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
