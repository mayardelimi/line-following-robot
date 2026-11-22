#!/usr/bin/env python3
"""PID line follower.

camera_sensor/image_raw --> OpenCV finds the white line --> PID --> /cmd_vel

Pipeline per frame:
  1. resize to 320 px wide and keep the bottom `crop_height` rows (the floor in front of the robot)
  2. threshold the bright line, take the largest blob and its centroid (image moments)
  3. error = (centroid_x - image_centre) / half_width           in [-1, 1]
  4. angular.z = -PID(error);  linear.x shrinks when the error is large
  5. if the line disappears: keep the last command for `max_lost_frames`, then stop

All values below are ROS parameters, e.g.:  ros2 param set /pid_follower kp 1.5
(use decimals for float parameters: 1.5, not 1)
"""
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

PROCESSING_WIDTH = 320
SMOOTHING_FRAMES = 3


class PidFollower(Node):
    def __init__(self):
        super().__init__('pid_follower')

        # Error is normalised to [-1, 1], so kp must be around 1-3 to give a real turn rate.
        for name, default in (
            ('kp', 2.0),
            ('ki', 0.0),
            ('kd', 0.3),
            ('max_linear', 0.5),        # m/s on a straight line
            ('max_angular', 1.0),       # rad/s
            ('white_threshold', 180),   # line renders ~250, floor ~0
            ('crop_height', 100),       # rows of the 320-wide image used for detection
            ('min_area', 100),          # smallest blob (px^2) accepted as the line
            ('max_lost_frames', 15),    # ~0.5 s at 30 Hz
        ):
            self.declare_parameter(name, default)

        self.bridge = CvBridge()
        self.error_window = deque(maxlen=SMOOTHING_FRAMES)
        self.integral = 0.0
        self.last_error = 0.0
        self.lost_frames = 0
        self.last_cmd = Twist()

        image_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, 'camera_sensor/image_raw', self.on_image, image_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.overlay_pub = self.create_publisher(Image, 'debug/overlay', 1)
        self.mask_pub = self.create_publisher(Image, 'debug/binary', 1)

        self.get_logger().info('PID follower started')

    # ------------------------------------------------------------------ vision
    def param(self, name):
        return self.get_parameter(name).value

    def resize_and_crop(self, frame):
        height, width = frame.shape[:2]
        if width != PROCESSING_WIDTH:
            height = int(height * PROCESSING_WIDTH / width)
            frame = cv2.resize(frame, (PROCESSING_WIDTH, height))
        crop_start = max(0, height - int(self.param('crop_height')))
        return frame, crop_start

    def find_line(self, crop):
        """Return (centroid_x or None, binary mask)."""
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, int(self.param('white_threshold')), 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs = [c for c in contours if cv2.contourArea(c) > self.param('min_area')]
        if not blobs:
            return None, mask

        m = cv2.moments(max(blobs, key=cv2.contourArea))
        if m['m00'] == 0:
            return None, mask
        return int(m['m10'] / m['m00']), mask

    # ----------------------------------------------------------------- control
    def pid(self, error_px, width):
        """PID on the normalised, lightly smoothed error. Returns a Twist."""
        self.error_window.append(error_px / (width / 2.0))
        error = sum(self.error_window) / len(self.error_window)

        self.integral = float(np.clip(self.integral + error, -10.0, 10.0))   # anti-windup
        derivative = error - self.last_error
        self.last_error = error

        output = self.param('kp') * error + self.param('ki') * self.integral + self.param('kd') * derivative

        cmd = Twist()
        cmd.linear.x = float(self.param('max_linear') * max(0.3, 1.0 - 0.7 * abs(error)))
        limit = float(self.param('max_angular'))
        cmd.angular.z = float(np.clip(-output, -limit, limit))   # line on the right -> turn right (negative)
        return cmd

    # ---------------------------------------------------------------- callback
    def on_image(self, msg):
        try:
            frame, crop_start = self.resize_and_crop(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
            width = frame.shape[1]
            cx, mask = self.find_line(frame[crop_start:])

            if cx is None:
                self.lost_frames += 1
                cmd = self.last_cmd if self.lost_frames <= self.param('max_lost_frames') else Twist()
                error_px = None
            else:
                self.lost_frames = 0
                error_px = cx - width // 2
                cmd = self.last_cmd = self.pid(error_px, width)

            self.cmd_pub.publish(cmd)
            self.publish_debug(frame, mask, crop_start, cx, error_px, cmd)

            self.get_logger().info(
                f'error={error_px} v={cmd.linear.x:.2f} w={cmd.angular.z:.2f} lost={self.lost_frames}',
                throttle_duration_sec=1.0)

        except Exception as exc:   # never leave the robot driving on a failure
            self.get_logger().error(f'Image processing error: {exc}')
            self.cmd_pub.publish(Twist())

    def publish_debug(self, frame, mask, crop_start, cx, error_px, cmd):
        if self.overlay_pub.get_subscription_count() > 0:
            view = frame.copy()
            h, w = view.shape[:2]
            cv2.rectangle(view, (0, crop_start), (w - 1, h - 1), (255, 0, 0), 2)         # blue: region used
            cv2.line(view, (w // 2, crop_start), (w // 2, h), (255, 255, 0), 1)          # robot centre
            if cx is not None:
                cv2.circle(view, (cx, crop_start + (h - crop_start) // 2), 7, (0, 0, 255), -1)
            cv2.putText(view, f'PID err={error_px} v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}',
                        (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(view, 'bgr8'))
        if self.mask_pub.get_subscription_count() > 0:
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, 'mono8'))


def main(args=None):
    rclpy.init(args=args)
    node = PidFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())   # stop the robot on shutdown
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
