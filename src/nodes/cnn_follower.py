#!/usr/bin/env python3
"""CNN line follower (default controller).

camera_sensor/image_raw --> pretrained CNN (ONNX) --> (v, w) --> /cmd_vel

Model: JdeRobot "Follow-Line" imitation-learning network (unified master model, v1).
  input : float32 [1, 3, 240, 640]   RGB scaled to 0-1, bottom half of a 640x480 frame
  output: float32 [1, 2]             (v, w) in the units of the car it was trained on

Only `w` (turn rate) is used: it is mapped to the robot with  w_cmd = w_scale * w + w_bias,
and the forward speed is the fixed `cnn_speed`. The network cannot tell that the track has
ended, so a small OpenCV check (white pixels in the strip in front of the robot) stops the
robot when the line has been gone for `max_lost_frames` frames.

Tune at runtime, e.g.:  ros2 param set /cnn_follower w_scale 0.6
(use decimals for float parameters: 0.6, not 1)
"""
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except ImportError:                     # pragma: no cover
    get_package_share_directory = None
    PackageNotFoundError = Exception

try:
    import onnxruntime as ort
except ImportError:                     # pip install --user --break-system-packages onnxruntime
    ort = None

PACKAGE = 'line_maze_solver'
MODEL_FILE = 'follow-line-imitation-master-model-v1.onnx'
MODEL_SIZE = (640, 480)                 # (width, height) the network was trained on
MODEL_CROP_ROWS = 240                   # bottom rows of that frame fed to the network
WATCHDOG_WIDTH = 320


class CnnFollower(Node):
    def __init__(self):
        super().__init__('cnn_follower')

        for name, default in (
            ('model_path', ''),             # empty = ai_models/ inside the package
            ('cnn_speed', 0.4),             # m/s, fixed forward speed
            ('w_scale', 1.0),               # w_cmd = w_scale * w_model + w_bias
            ('w_bias', 0.0),
            ('max_angular', 1.0),           # rad/s
            ('stop_when_line_lost', True),  # OpenCV safety stop (see module docstring)
            ('white_threshold', 180),
            ('min_white_pixels', 100),      # in the 320-wide strip; below this = no line
            ('max_lost_frames', 15),        # ~0.5 s at 30 Hz
        ):
            self.declare_parameter(name, default)

        self.bridge = CvBridge()
        self.lost_frames = 0
        self.session = self.load_model()
        self.input_name = self.session.get_inputs()[0].name

        image_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, 'camera_sensor/image_raw', self.on_image, image_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.overlay_pub = self.create_publisher(Image, 'debug/overlay', 1)

        self.get_logger().info('CNN follower started')

    # ------------------------------------------------------------------- model
    def param(self, name):
        return self.get_parameter(name).value

    def find_model(self):
        candidates = []
        if self.param('model_path'):
            candidates.append(os.path.expanduser(self.param('model_path')))
        if get_package_share_directory is not None:
            try:
                candidates.append(os.path.join(get_package_share_directory(PACKAGE), 'ai_models', MODEL_FILE))
            except PackageNotFoundError:
                pass
        here = os.path.dirname(os.path.realpath(__file__))          # src/nodes (symlink-install)
        candidates.append(os.path.join(here, '..', '..', 'ai_models', MODEL_FILE))

        for path in candidates:
            if os.path.isfile(path):
                return os.path.normpath(path)
        raise RuntimeError('CNN model not found. Looked in:\n  ' + '\n  '.join(candidates))

    def load_model(self):
        if ort is None:
            raise RuntimeError('onnxruntime is not installed: '
                               'pip install --user --break-system-packages onnxruntime')
        path = self.find_model()
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        session = ort.InferenceSession(path, sess_options=options, providers=['CPUExecutionProvider'])
        self.get_logger().info(f'CNN model loaded: {path}')
        return session

    def infer(self, frame):
        """Same preprocessing as the JdeRobot exercise. Returns (v, w) in the model's own units."""
        if (frame.shape[1], frame.shape[0]) != MODEL_SIZE:
            frame = cv2.resize(frame, MODEL_SIZE)
        rgb = frame[-MODEL_CROP_ROWS:, :, ::-1].astype(np.float32) / 255.0
        batch = np.ascontiguousarray(rgb.transpose(2, 0, 1))[None]
        out = self.session.run(None, {self.input_name: batch})[0][0]
        return float(out[0]), float(out[1])

    # ------------------------------------------------------------ safety check
    def line_visible(self, frame):
        """True if enough bright pixels are in the strip just in front of the robot."""
        height = int(frame.shape[0] * WATCHDOG_WIDTH / frame.shape[1])
        strip = cv2.resize(frame, (WATCHDOG_WIDTH, height))[-100:]
        gray = cv2.cvtColor(strip, cv2.COLOR_BGR2GRAY)
        return np.count_nonzero(gray > self.param('white_threshold')) >= self.param('min_white_pixels')

    # ---------------------------------------------------------------- callback
    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            t0 = time.time()
            v_model, w_model = self.infer(frame)
            infer_ms = (time.time() - t0) * 1000.0

            limit = float(self.param('max_angular'))
            cmd = Twist()
            cmd.linear.x = float(self.param('cnn_speed'))
            cmd.angular.z = float(np.clip(self.param('w_scale') * w_model + self.param('w_bias'), -limit, limit))

            if self.param('stop_when_line_lost'):
                self.lost_frames = 0 if self.line_visible(frame) else self.lost_frames + 1
                if self.lost_frames > self.param('max_lost_frames'):
                    cmd = Twist()

            self.cmd_pub.publish(cmd)
            self.publish_overlay(frame, v_model, w_model, cmd)

            self.get_logger().info(
                f'model(v={v_model:.2f}, w={w_model:.2f}) -> cmd(v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}) '
                f'{infer_ms:.0f} ms lost={self.lost_frames}',
                throttle_duration_sec=1.0)

        except Exception as exc:   # never leave the robot driving on a failure
            self.get_logger().error(f'Inference error: {exc}')
            self.cmd_pub.publish(Twist())

    def publish_overlay(self, frame, v_model, w_model, cmd):
        if self.overlay_pub.get_subscription_count() == 0:
            return
        view = frame.copy()
        h, w = view.shape[:2]
        top = h - int(MODEL_CROP_ROWS * h / MODEL_SIZE[1])
        cv2.rectangle(view, (0, top), (w - 1, h - 1), (255, 0, 0), 3)                  # blue: CNN input region
        cv2.putText(view, f'CNN model(v={v_model:.2f} w={w_model:.2f})  cmd(v={cmd.linear.x:.2f} w={cmd.angular.z:.2f})',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(view, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CnnFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:        # model or onnxruntime missing: say so clearly and exit
        print(f'[cnn_follower] {exc}')
    finally:
        if node is not None:
            try:
                node.cmd_pub.publish(Twist())   # stop the robot on shutdown
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
