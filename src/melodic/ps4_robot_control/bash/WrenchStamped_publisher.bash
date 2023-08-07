rostopic pub /myrobot/left_cartesian_force_controller/target_wrench geometry_msgs/WrenchStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'left_soft_hand_tip'
wrench:
  force:
    x: 0.05
    y: 0.0
    z: 0.0
  torque:
    x: 0.0
    y: 0.0
    z: 0.0" -1


rostopic pub /target_wrench geometry_msgs/WrenchStamped "
header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'base_link'
wrench:
  force:
    x: 0.0
    y: 0.0
    z: 0.0
  torque:
    x: 0.0
    y: 0.0
    z: 0.0" -1


