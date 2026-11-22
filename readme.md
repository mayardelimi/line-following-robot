# Line Maze Solver

A vision-based line-following robot for **ROS 2 + Gazebo (gz sim)**. A differential-drive robot with a
front camera follows a white line on a black floor. Two interchangeable controllers drive it:

| Controller | File | How it decides |
|---|---|---|
| **CNN** (default) | `src/nodes/cnn_follower.py` | Pretrained ONNX network: camera image -> `(v, w)` |
| **PID** | `src/nodes/pid_follower.py` | OpenCV finds the line, a PID turns the offset into a turn rate |

Both read `camera_sensor/image_raw` and publish `/cmd_vel`.

## Install

```bash
cd ~/AI/robotic/ros2_ws/src            # your workspace
# put this folder (Line_Maze_Solver) here, then:
pip install --user --break-system-packages onnxruntime     # needed by the CNN controller only
cd ~/AI/robotic/ros2_ws
colcon build --packages-select line_maze_solver --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch line_maze_solver maze_bot_launch.py                    # CNN controller (default)
ros2 launch line_maze_solver maze_bot_launch.py controller:=pid    # PID controller
ros2 launch line_maze_solver maze_bot_launch.py viewer:=false use_rviz:=false
```

Launch arguments: `controller:=cnn|pid`, `viewer:=true|false` (camera windows), `use_rviz:=true|false`.

## Tuning (no rebuild needed)

Every value is a ROS parameter; change it while the robot runs (use decimals for floats):

```bash
ros2 param set /pid_follower kp 1.5            # also ki, kd, max_linear, max_angular, white_threshold, crop_height ...
ros2 param set /cnn_follower w_scale 0.6       # also w_bias, cnn_speed, max_angular, stop_when_line_lost ...
ros2 param list /cnn_follower
```

**CNN notes.** The network was trained on an F1 car on a different track, so its turn rate `w` is in
other units: the robot uses `w_cmd = w_scale * w + w_bias` at a fixed `cnn_speed`. Start with the defaults
and adjust `w_scale` (too weak: raise it; wobbling: lower it; wrong direction: the sign is negative).
The node logs the raw model output and the command once per second. Because a CNN cannot tell that
the track has ended, the node stops the robot when no line has been visible for 15 frames
(`stop_when_line_lost`).

## See and save frames

`frame_viewer.py` opens two windows (raw camera, controller overlay). Click a window and press
**s** to save one frame or **r** to start/stop recording. Files go to `~/line_follower_frames/`.
Without a display: `ros2 param set /frame_viewer record true`.
The same images are on the topics `camera_sensor/image_raw`, `debug/overlay` (both controllers) and
`debug/binary` (PID): `ros2 run rqt_image_view rqt_image_view`.

## Project structure

```
Line_Maze_Solver/
├── ai_models/follow-line-imitation-master-model-v1.onnx   pretrained CNN (44 MB)
├── config/default.rviz
├── launch/
│   ├── maze_bot_launch.py            everything (choose controller:=cnn|pid)
│   ├── gazebo_launch.py              gz sim, robot spawn, ROS<->gz bridge
│   ├── robot_state_publisher_launch.py
│   └── rviz_launch.py
├── models/line_path_ground_plane/    track (path_2_4096.png in use, path_1_4096.png alternative)
├── src/nodes/
│   ├── cnn_follower.py
│   ├── pid_follower.py
│   └── frame_viewer.py
├── urdf/                             robot.urdf.xacro, camera.xacro
└── worlds/line_follower.world
```

Robot: differential drive, 0.15 x 0.15 x 0.05 m, camera 800x600 at 30 Hz.
Data flow: `Gazebo camera -> ros_gz_bridge -> controller -> /cmd_vel -> ros_gz_bridge -> DiffDrive plugin -> wheels`.

## Credits and licenses

* Simulation package based on [Line_Maze_Solver](https://github.com/rohitjangra7370/Line_Maze_Solver)
  by Rohit Jangra (Apache License 2.0, see `LICENSE`).
* CNN model: JdeRobot Follow-Line imitation-learning models (Google Summer of Code 2025,
  Md. Shariar Kabir, `JdeRobot/Follow-Line-Imitation-DL-Models-V1`). Check the model's license
  before redistributing the `.onnx` file.
