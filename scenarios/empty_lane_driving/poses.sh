#! /usr/bin/bash
ros2 topic pub -t 5 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped 'header:
  frame_id: map
pose:
  pose:
    position:
      x: 3796.374267578125
      y: 73708.7265625
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.9740832158310118
      w: 0.22618993929066458'

ros2 topic pub -t 5 /planning/mission_planning/goal geometry_msgs/msg/PoseStamped 'header:
  frame_id: map
pose:
  position:
    x: 3784.696533203125
    y: 73756.2421875
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.25407197999986825
    w: 0.9671853126360773'

ros2 topic pub /autoware/engage autoware_auto_vehicle_msgs/msg/Engage 'engage: true'

