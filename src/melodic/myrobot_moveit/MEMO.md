# launch
## bringup.launch
### ノードの記述順
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

### Argument
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
|grasp_only|false|コンテナでpickをするだけか(falseで台に置くところまでやる)|
|manual_wait|false||
|used_camera|left_camera||
|sensors|[right_camera, body_camera]||

# script
## add_object.py
3秒間だけPlanning Sceneにオブジェクトを追加する。

## grasp_detection_client.py

## manual_data_recoder.py

## octomap_handler.py

## pick_place_test.py
pick_place.pyがそのままだと動かなかったのでコピーして、改変。

## pick_place.py
MoveGroupCommanderのpick()とplace()を利用したデモ。
ref: [Pick and Place](https://ros-planning.github.io/moveit_tutorials/doc/pick_place/pick_place_tutorial.html),[MoveGroupCommander](https://robo-marc.github.io/moveit_documents/moveit_commander.html#movegroupcommander),[moveit_msgs](https://robo-marc.github.io/moveit_documents/ros_messages.html)

## planning.py

## pose_planner.py
エンドエフェクタの位置・姿勢を指定した経路計画

## robot_info.py
ロボットの情報を出力
- moveit_commander.RobotCommander()
- moveit_commander.MoveGroupCommander("<group_name>")

## test.py
detected_objectsトピックからの情報をもとにpick and placeする。
pick and placeの部分に関してはpick_place.pyと同じ。



## modules
### const.py
WHOLE_ROOT_PATH : /home/shin/catkin_ws

WHOLE_OUTPUTS_PATH : /home/shin/catkin_ws/outputs

PKG_ROOT_PATH : /home/shin/catkin_ws/src/myrobot_moveit

PKG_OUTPUTS_PATH : /home/shin/catkin_ws/src/myrobot_moveit/outputs