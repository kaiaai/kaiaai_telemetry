# [Kaia.ai](https://kaia.ai/) ROS2 telemetry package
[Kaia.ai](https://kaia.ai/) ROS2 kaia_telemetry package communicates with the Kaia.ai ESP32-based robot,
receives raw sensor telemetry data over WiFi using Micro-ROS and re-publishes the telemetry to standard ROS2 topics:
- robot laser scan data on the /scan topic
- robot wheels and servos position on the /joint_states topic
- robot wheel odometry on the /odom topic
- robot odometry transform on the /tf topic

The [telemetry message](https://github.com/kaiaai/kaia_msgs) is a ROS2 custom message designed to be as compact as possible in terms of its size in order to reduce communication latency and minimize dropped packets, thus keeping the robot's navigation responsive and agile.

## How to rebuild the package

Navigate to the development workspace, manually build, install and launch Kaia.ai
packages including kaiaai_telemetry
```
cd /ros_ws
colcon build
. install/setup.bash
ros2 launch kaiaai_bringup main.launch.py robot_model:=makerspet_loki
```

At the newly opened bash prompt, run the telemetry node that subscribes to the raw telemetry
data on /telemetry topic, converts the raw telemetry data to proper ROS2 messages re-publishes those on
/scan, /joint_states, /odom and /tf topics.
```
ros2 run kaiaai_telemetry telem
```
Open yet another bash prompt and inspect the raw telemetry data going "in" on the /telemetry topic
and the converted telemetry data published on the /scan, /joint_states, /odom and /tf topics, etc.:
```
ros2 topic list
ros2 topic echo /telemetry
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /joint_states
ros2 topic echo /tf
```

## Modding the default robot

The telemetry launch commands described above default to `makerspet_snoopy` robot, which defined in the
`makerspet_snoopy` robot description package. To create a new robot named `jack45_waldo`, start
by cloning an existing robot description package `makerspet_snoopy` into `jack45_waldo`
and proceed with modding `jack45_waldo` files. The file containing telemetry parameters
for `jack45_waldo` is `/ros_ws/src/jack45_waldo/config/telem.yaml`
```
cp -r /ros_ws/src/makerspet_loki /ros_ws/src/jack45_waldo
```
Modify files in `/ros_ws/src/jack45_waldo/` as needed, including `/ros_ws/src/jack45_waldo/config/telem.yaml`

Now you can run telemetry on `jack45_waldo` with `jack45_waldo`-specific telemetry parameters as follows:
```
ros2 run kaiaai_telemetry telem robot_model:=jack45_waldo
```
