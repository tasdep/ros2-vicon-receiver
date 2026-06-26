# vicon_velocity_estimator

Companion package for `vicon_receiver` that estimates linear and angular
velocity from a `geometry_msgs/msg/PoseStamped` Vicon pose topic.

```bash
ros2 run vicon_velocity_estimator pose-velocity-estimator --ros-args \
  -p pose_topic:=/vicon/tas/tas \
  -p velocity_topic:=/vicon/tas/velocity \
  -p method:=lowpass_fd \
  -p alpha:=0.35 \
  -p window_size:=7
```

Use this for plotting or dataset recording. The PX4 external-vision bridge uses
pose-only messages and sends velocity fields as `NaN`.
