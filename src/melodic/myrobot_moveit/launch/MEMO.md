# bringup.launch
## ノードの記述順
- planning_context.launch
- robot_state_publisher
- moveit_rviz.launch
- default_warehouse_db.launch
- joint_state_publisher
- move_group.launch
- gazebo.launch
- custom_denso_robot_control.launch
- multiple_rs_camera.launch
- planning.py
- grasp_detection_client.py
- octomap_handler.py
- record

## Argument
| Argument | デフォルト値 | 説明 |
| :--- | :---: | :--- |
|robot_name|myrobot|ロボット名|
|sim|true|シミュレータにするかどうか|
|record_all|false|rosbagを記録するかどうか|
|enable_auto_planning|true|要確認|
|use_camera|true|実機でリアルセンスを使用するかどうか|
|fps|1||
|wait_server|true||
|use_constraint|false||
|pre_move|true||
|grasp_only|false||
|manual_wait|false||
|used_camera|left_camera||
|sensors|[right_camera, body_camera]||
