#!/usr/bin/env python2
# coding: UTF-8
import math
import sys

import moveit_commander as mc
import numpy as np
import rospy
import tf
from actionlib import SimpleActionClient
from controller_manager_msgs.srv import SwitchController
from detect.msg import VisualizeTargetAction, VisualizeTargetGoal, HandSpeedDirection
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist, Vector3
from grasp_detection_client import GraspDetectionClient
from container_position_client import ContainerPositionClient
from modules.colored_print import *
from modules.ros.utils import call
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import Grasp as BaseGrasp
from moveit_msgs.msg import OrientationConstraint, RobotState
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, Header
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint
from exclusion_list_client import ExclusionListClient

class MoveGroup(mc.MoveGroupCommander):
    def __init__(self, name, scaling_factor=1, planning_time=5):
        super(MoveGroup, self).__init__(name)
        self.set_planning_time(planning_time)
        self.set_max_velocity_scaling_factor(scaling_factor)

    # ジョイント名をキー, 関節角度値を値とした辞書
    def get_current_joint_dict(self):
        return dict(zip(self.get_active_joints(), self.get_current_joint_values()))

class MoveGroupHandler:
    def __init__(self, left_start_move_group, right_start_move_group, whole_move_group):
        self.left_start_move_group = left_start_move_group
        # self.left_eef_default_pose = left_start_move_group.get_current_pose().pose
        self.right_start_move_group = right_start_move_group
        # self.right_eef_default_pose = right_start_move_group.get_current_pose().pose
        self.whole_move_group = whole_move_group
        self.whole_name = whole_move_group.get_name()

        # left arm (arm_index: 0) からスタート
        self.set_current_move_group(0)
        # self.current_move_group = left_start_move_group
        # self.current_eef_default_pose = left_start_move_group.get_current_pose().pose

        self.is_in_peril = False
        self.sub = rospy.Subscriber("/is_in_peril", Bool, self.set_peril)

        self.mv_right_arm = MoveGroup("back_and_right_arm")

        self.hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
        self.hand_msg = Float64MultiArray()
        self.hand_msg.data = [0.0, 0.0]

        self.hand_right_pub = rospy.Publisher('/hand_right_ref_pressure', Float64, queue_size=1)
        self.hand_right_msg = Float64()
        self.hand_right_msg.data = 0.0
        
        self.hand_left_pub = rospy.Publisher('/hand_left_ref_pressure', Float64, queue_size=1)
        self.hand_left_msg = Float64()
        self.hand_left_msg.data = 0.0

        self.hand_emergency_enable_pub = rospy.Publisher('/hand_emergency_enable', Bool, queue_size=1)
        self.hand_emergency_enable_msg = Bool()
        self.hand_emergency_enable_msg.data = False

    def set_peril(self, msg):
        self.is_in_peril = msg.data

    def set_current_move_group(self, arm_index):
        if arm_index == 0:
            self.current_move_group = self.left_start_move_group
            # self.current_eef_default_pose = self.left_eef_default_pose
        elif arm_index == 1:
            self.current_move_group = self.right_start_move_group
            # self.current_eef_default_pose = self.right_eef_default_pose 

    def initialize_current_pose(self, wait=True):
        group_name = self.get_current_name()
        target_name = "{}_start".format(group_name)
        target_joint_dict = self.current_move_group.get_named_target_values(target_name)
        plan = self.current_move_group.plan(target_joint_dict)
        self.current_move_group.execute(plan, wait=wait)

    def initialize_whole_pose(self, wait=True,):
        # target_name = "{}_default".format(self.whole_name)
        target_name = "{}_start".format(self.whole_name)
        target_joint_dict = self.whole_move_group.get_named_target_values(target_name)
        plan = self.whole_move_group.plan(target_joint_dict)
        self.whole_move_group.execute(plan, wait=wait)


    # def plan(self, joints={}, is_degree=False, **kwargs):
    #     # if plan failed, switch move_group & plan again
    #     new_joints = joints
    #     new_joints.update(kwargs)
    #     if is_degree:
    #         new_joints = { k:np.radians(v)  for k,v in new_joints.items()}

    #     merged_joints = self.whole_move_group.get_current_joint_dict()
    #     merged_joints.update(new_joints)
        
    #     return self.whole_move_group.plan(merged_joints)

    def execute(self, plan, wait):
        res =  self.whole_move_group.execute(plan, wait=wait)
        self.whole_move_group.stop()
        self.whole_move_group.clear_pose_targets()
        return res

    def approach(self, target_pose, c_eef_step=0.001, c_jump_threshold=0.0):
        printb("approach planning start")

        hand_enable_pub = rospy.Publisher('/hand_enable', Bool, queue_size=1)
        hand_enable_msg = Bool()
        hand_enable_msg.data = True 
        hand_enable_pub.publish(hand_enable_msg)

        waypoints = [target_pose]
        plan, plan_score = self.current_move_group.compute_cartesian_path(waypoints, c_eef_step, c_jump_threshold)
        printc("plan_score : {}".format(plan_score))


        if plan_score > 0.9: # TODO
            printb("approach execution")
            if self.is_in_peril:
                return False
            self.execute(plan, wait=True)
            return True
        else:
            printr("approach execution failed...")
            return False


    def pick(self, arm_index, target_pose, access_distance, target_pressure, z_direction, down=True, c_eef_step=0.001, c_jump_threshold=0.0):
        printb("pick planning start")
        if arm_index == 0:
            arm = "left"
        elif arm_index == 1:
            arm = "right"

        printb("hand adjustment execution")
        if self.is_in_peril:
                return False
        # self.hand_msg.data[arm_index] = target_pressure # 右アームのみ
        # self.hand_pub.publish(self.hand_msg)

        if arm_index == 0:
            self.hand_left_msg.data = target_pressure
            self.hand_left_pub.publish(self.hand_left_msg)
        else:
            self.hand_right_msg.data = target_pressure
            self.hand_right_pub.publish(self.hand_right_msg)


        printc("target_pressure : {}".format(target_pressure))

        waypoints = [target_pose]
        plan, plan_score = self.current_move_group.compute_cartesian_path(waypoints, c_eef_step, c_jump_threshold)
        print("pose score", plan_score)
        if plan_score < 0.5:
            print("pick failed 1...")
            return False
        
        printb("pre pose execution")
        if self.is_in_peril:
                return False
        self.execute(plan, wait=True)

        rospy.sleep(0.1)

        # ハンドの曲げセンサによる例外処理有効化
        self.hand_emergency_enable_msg.data = True
        self.hand_emergency_enable_pub.publish(self.hand_emergency_enable_msg)

        # 先に位置姿勢と取得して、その後コントローラーを切り替える
        # さもないと指令加速度過大になりがち
        startup_pub = rospy.Publisher('/startup/{}'.format(arm), Empty, queue_size=1)
        empty_msg = Empty()
        startup_pub.publish(empty_msg)


        if self.is_in_peril:
                return False
        if down:
            try:
                call("/myrobot/{}_arm/controller_manager/switch_controller".format(arm), SwitchController,
                    start_controllers=["{}_cartesian_motion_controller".format(arm)],
                    stop_controllers=["{}_arm_controller".format(arm)],
                    strictness=1, start_asap=False, timeout=0.0)

            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)


            # 把持時のaccess_distanceとはハンドとキャベツの距離(m単位)
            # move_time = 1.35 - access_distance / 0.03 * 0.1
            # move_time = 1.35 - access_distance * 0.01 
            # move_time = 1.35 - access_distance * 0.01 / 2 
            # move_time = 1.35 - access_distance * 0.01 / 1.2 
            move_time = 1.2 - access_distance * 0.01 / 1.2 

            printc("access_distance : {}".format(access_distance))
            printc("move_time : {}".format(move_time))

            printb("down execution")
            if self.is_in_peril:
                    return False
            lower_speed_pub = rospy.Publisher('/target_hand_lower_speed', HandSpeedDirection, queue_size=1)
            lower_speed = HandSpeedDirection()
            lower_speed.speed = 0.1
            lower_speed.direction = Vector3(x = z_direction[0], y = z_direction[1] , z = z_direction[2])
            lower_speed_pub.publish(lower_speed)

            rospy.sleep(move_time)


            lower_speed.speed = 0
            lower_speed_pub.publish(lower_speed)

            rospy.sleep(move_time)

            printb("grab execution")
            if self.is_in_peril:
                    return False
            # self.hand_msg.data[arm_index] = 1.5
            # self.hand_pub.publish(self.hand_msg)
            if arm_index == 0:
                self.hand_left_msg.data = 1.5
                self.hand_left_pub.publish(self.hand_left_msg)
            else:
                self.hand_right_msg.data = 1.5
                self.hand_right_pub.publish(self.hand_right_msg)


            printb("grabed")

            rospy.sleep(1.0)

            printb("up execution")
            if self.is_in_peril:
                    return False
            lower_speed.speed = -0.1
            lower_speed_pub.publish(lower_speed)

            rospy.sleep(move_time)

            printb("up stop execution")
            if self.is_in_peril:
                    return False
            lower_speed.speed = 0
            lower_speed_pub.publish(lower_speed)

            rospy.sleep(move_time)


            try:
                call("/myrobot/{}_arm/controller_manager/switch_controller".format(arm), SwitchController,
                    start_controllers=["{}_arm_controller".format(arm)],
                    stop_controllers=["{}_cartesian_motion_controller".format(arm)],
                    strictness=1, start_asap=False, timeout=0.0)

            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)


        # self.hand_emergency_enable_msg.data = False
        # self.hand_emergency_enable_pub.publish(self.hand_emergency_enable_msg)

        # rospy.sleep(0.5)

        return True


    def place(self):

        target_joint_dict = self.whole_move_group.get_named_target_values("back_and_arms_place_down")
        plan = self.whole_move_group.plan(target_joint_dict)

        printb("pre place execution")
        if self.is_in_peril:
                return False
        self.execute(plan, wait=True)

        if self.is_in_peril:
                return False
        self.hand_msg.data = [0, 0]
        self.hand_pub.publish(self.hand_msg)

        rospy.sleep(1)

        hand_enable_pub = rospy.Publisher('/hand_enable', Bool, queue_size=1)
        hand_enable_msg = Bool()
        hand_enable_msg.data = False 
        hand_enable_pub.publish(hand_enable_msg)

        # rospy.sleep(2)


        target_joint_dict = self.whole_move_group.get_named_target_values("back_and_arms_place_up")
        plan = self.whole_move_group.plan(target_joint_dict)
        printb("post place execution")
        if self.is_in_peril:
                return False
        self.execute(plan, wait=True)

        return True

    def get_current_name(self):
        return self.current_move_group.get_name()



class ContactOrientationController:
    def __init__(self):
        self.angle = 15
        self.contact_angles = {
            0 : [np.radians(0), np.radians(0), math.pi],
            1 : [np.radians(-30), np.radians(0), math.pi + np.radians(-self.angle)], 
            2 : [np.radians(0), np.radians(self.angle), math.pi],
            3 : [np.radians(-15), np.radians(self.angle), math.pi + np.radians(-self.angle)],
            4 : [np.radians(30), np.radians(0), math.pi + np.radians(self.angle)],
            6 : [np.radians(15), np.radians(self.angle), math.pi + np.radians(self.angle)],
            8 : [np.radians(60), np.radians(-self.angle), math.pi],
            9 : [np.radians(-45), np.radians(-self.angle), math.pi + np.radians(-self.angle)],
            12 : [np.radians(45), np.radians(-self.angle), math.pi + np.radians(self.angle)]
        }
        self.z_direction = {}
        for k, v in self.contact_angles.items():
            self.z_direction[k] = self.compute_z_direction(v[2], v[1])

    def compute_z_direction(self, xt, yt):
        P = np.array([[1, 0, 0],
                      [0, np.cos(xt), -np.sin(xt)],
                      [0, np.sin(xt), np.cos(xt)]])
        Q = np.array([[np.cos(yt), 0, np.sin(yt)],
                      [0, 1, 0],
                      [-np.sin(yt), 0, np.cos(yt)]])

        z = np.array([0, 0, 1])
        return np.dot(P, np.dot(Q, z))



class Myrobot:
    def __init__(self, fps, left_image_topic, left_depth_topic, left_points_topic, 
                 right_image_topic, right_depth_topic, right_points_topic, wait = True):
        mc.roscpp_initialize(sys.argv)

        self.robot = mc.RobotCommander()
        self.scene_handler = mc.PlanningSceneInterface(synchronous=True)

        # left groups
        mv_group_left = MoveGroup("left_arm", scaling_factor=1, planning_time=10)
        # mv_body_to_left_arm = MoveGroup("body_and_left_arm", planning_time=10)
        # right groups
        mv_group_right = MoveGroup("right_arm", scaling_factor=1, planning_time=10)
        # mv_body_to_right_arm = MoveGroup("body_and_right_arm", planning_time=10)
        # whole group
        mv_group_while = MoveGroup("back_and_arms", scaling_factor=1, planning_time=10)

        printg("movegroup load : SUCCESS")

        self.mv_handler = MoveGroupHandler(mv_group_left, mv_group_right, mv_group_while)
        # self.mv_handler = MoveGroupHandler(mv_left_arm, mv_right_arm, mv_base_to_arms)
        # self.mv_handler = MoveGroupHandler(mv_left_arm, mv_right_arm,  mv_right_arm)

        self.arm_index = 0

        self.gd_cli = [
            # for left arm     
            GraspDetectionClient( 
                arm_index = 0,
                fps=fps,
                image_topic=left_image_topic, 
                depth_topic=left_depth_topic,
                points_topic=left_points_topic,
                wait=wait
            ),
            # for right arm
            GraspDetectionClient(
                 arm_index = 1, 
                fps=fps, 
                image_topic=right_image_topic, 
                depth_topic=right_depth_topic,
                points_topic=right_points_topic,
                wait=wait
            )
        ]

        self.cp_cli = ContainerPositionClient()

        self.el_cli = ExclusionListClient()

    def is_in_peril(self):
        return self.mv_handler.is_in_peril
    
    def set_robot_status_good(self):
        self.mv_handler.is_in_peril = False

    def set_arm_index(self, arm_index):
         self.arm_index = arm_index
         self.mv_handler.set_current_move_group(arm_index)

    # def _create_constraint(self, name, link_name, rpy, base_frame_id="base_link", xyz_tolerance=(0.05, 0.05, 3.6)):
    #     q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    #     constraint = Constraints(
    #         name=name,
    #         orientation_constraints = [OrientationConstraint(
    #             header=Header(frame_id=base_frame_id),
    #             link_name=link_name,
    #             orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
    #             # allow max rotations
    #             absolute_x_axis_tolerance = xyz_tolerance[0],
    #             absolute_y_axis_tolerance = xyz_tolerance[1],
    #             absolute_z_axis_tolerance = xyz_tolerance[2],
    #             weight = 1
    #         )]
    #     )
    #     return constraint

    def initialize_current_pose(self, wait=True):
        self.mv_handler.initialize_current_pose(wait=wait)

    def initialize_whole_pose(self):
        self.mv_handler.initialize_whole_pose()

    # def plan(self, joints={}, is_degree=False, **kwargs):
    #     return self.mv_handler.plan(joints, is_degree, **kwargs)

    # def execute(self, plan, wait=False):
    #     res =  self.mv_handler.execute(plan, wait)
    #     return res

    def approach(self, object_msg, c_eef_step=0.01, c_jump_threshold=0.0):
        approach_desired_distance = object_msg.length_to_center
        contact = object_msg.contact

        coc = ContactOrientationController()
        
        target_pose = object_msg.center_pose.pose
        # 位置に関して
        vec = coc.z_direction[contact] * approach_desired_distance * -1
        target_pose.position.x += vec[0]
        target_pose.position.y += vec[1]
        target_pose.position.z += vec[2]
        # 姿勢に関して
        rpy = coc.contact_angles[contact]
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2], axes="szyx")
        target_pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        res = self.mv_handler.approach(target_pose, c_eef_step, c_jump_threshold)
        return res
    
    def pick(self, res_msg, contact, down=True, c_eef_step=0.01, c_jump_threshold=0.0):
        # obj_position_point = target_pose.position

        # obj_position_vector = Vector3(obj_position_point.x, obj_position_point.y, obj_position_point.z)
        res_pose = res_msg.pose
        res_angle = res_msg.angle
        access_distance = res_msg.distance
        target_pressure = res_msg.pressure

        rpy=(math.pi, 0, np.radians(res_angle))
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        
        coc = ContactOrientationController()
        if contact != 0:
            rpy = coc.contact_angles[contact]
            q = quaternion_from_euler(rpy[0], rpy[1], rpy[2], axes="szyx")
            orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


        target_pose = Pose(position=res_pose.position, orientation=orientation)


        res = self.mv_handler.pick(self.arm_index, target_pose, access_distance, target_pressure, coc.z_direction[contact], down, c_eef_step, c_jump_threshold)
        return res
        

    def place(self):       
        # place_position = (1.1, -0.63, 0.78)
        place_position = (1.35, -0.63, 0.55)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = place_position
        res = self.mv_handler.place()
        return res        

    def detect(self):
        return self.gd_cli[self.arm_index].detect()
    
    def calcurate_insertion(self):
        return self.gd_cli[self.arm_index].calcurate_insertion()
    
    def add_exclusion_cabbage(self, u, v):
         return self.el_cli.add(self.arm_index, u, v)


    def clear_exclusion_cabbage(self):
         return self.el_cli.clear(self.arm_index)

    def set_container(self):

        res = self.cp_cli.get()

        container_pose = PoseStamped()
        container_pose.header.frame_id = "base_link"

        q = quaternion_from_euler(0.0, 0.0, -math.pi / 2)
        container_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        container_pose.pose.position.x = res.point.x
        container_pose.pose.position.y = res.point.y
        container_pose.pose.position.z = res.point.z
        container_name = 'container'

        container_filename = '/home/shin/catkin_ws/src/myrobot_description/urdf/container/close.dae'
        self.scene_handler.add_mesh(container_name, container_pose, container_filename, size=(0.001, 0.001, 0.001))

    def delete_container(self):
         self.scene_handler.remove_world_object('container')

    def info(self):
        print("-" * 30)
        print("/// robot commander ///")
        print("planinng frame: {}".format(self.robot.get_planning_frame()))
        print("group names: {}".format(self.robot.get_group_names()))
        print("/// scene interface ///")
        print("known objects: {}".format(self.scene_handler.get_known_object_names()))
        print("/// move_group commander ///")
        print("current group: {}".format(self.mv_handler.get_current_name()))
        print("end effector: {}".format(self.mv_handler.current_move_group.get_end_effector_link()))
        print("goal_joint_tolerance: {}".format(self.mv_handler.current_move_group.get_goal_joint_tolerance()))
        print("goal_orientation_tolerance: {}".format(self.mv_handler.current_move_group.get_goal_orientation_tolerance()))
        print("get_goal_position_tolerance: {}".format(self.mv_handler.current_move_group.get_goal_position_tolerance()))
        print("get_goal_tolerance: {}".format(self.mv_handler.current_move_group.get_goal_tolerance()))
        print("planning frame: {}".format(self.mv_handler.current_move_group.get_planning_frame()))
        print("planning time: {}".format(self.mv_handler.current_move_group.get_planning_time()))
        print("pose reference frame: {}".format(self.mv_handler.current_move_group.get_pose_reference_frame()))
        print("-" * 30)


if __name__ == "__main__":
    rospy.init_node("planning_node")

    # ref: http://zumashi.blogspot.com/2016/10/rosrun.html
    ns = rospy.get_param("robot_name", default="myrobot")
    fps = rospy.get_param("fps", default=1)
    left_image_topic = rospy.get_param("left_image_topic")
    left_depth_topic = rospy.get_param("left_depth_topic")
    left_points_topic = rospy.get_param("left_points_topic")
    right_image_topic = rospy.get_param("right_image_topic")
    right_depth_topic = rospy.get_param("right_depth_topic")
    right_points_topic = rospy.get_param("right_points_topic")
    down = rospy.get_param("down")
    # sensors = rospy.get_param("sensors", default=("left_camera", "right_camera", "body_camera"))

    wait = rospy.get_param("wait_server", default=True)

    rospy.loginfo("################################################")

    print("waiting for image topics")
    rospy.logerr("################################################")

    rospy.wait_for_message(left_image_topic, Image)
    rospy.wait_for_message(left_depth_topic, Image)
    rospy.wait_for_message(right_image_topic, Image)
    rospy.wait_for_message(right_depth_topic, Image)

    print("initializing instances...")
    myrobot = Myrobot(fps=fps, left_image_topic=left_image_topic, left_depth_topic=left_depth_topic, left_points_topic=left_points_topic, 
                      right_image_topic=right_image_topic, right_depth_topic=right_depth_topic, right_points_topic=right_points_topic, 
                      wait=wait)

    myrobot.info()
    # hand_radius_mmはdetect側のlaunchで設定しているのでwait後に読み込み
    hand_radius_mm = rospy.get_param("hand_radius_mm", default="157.5")
    collision_radius = hand_radius_mm / 1000 * 0.5

    print("initializing pose...")
    myrobot.initialize_whole_pose()

    print("stating detect flow...")

    # 最初のメッセージがなぜか無視されるので、ダミーで一回パブリッシュ
    hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
    hand_msg = Float64MultiArray()
    hand_msg.data = [0.0, 0.0]
    hand_pub.publish(hand_msg)
    # 同様の理由でここでもパブリッシュ
    startup_pub_left = rospy.Publisher('/startup/left', Empty, queue_size=1)
    empty_msg = Empty()
    startup_pub_left.publish(empty_msg)
    startup_pub_right = rospy.Publisher('/startup/right', Empty, queue_size=1)
    empty_msg = Empty()
    startup_pub_right.publish(empty_msg)
    lower_speed_pub = rospy.Publisher('/target_hand_lower_speed', HandSpeedDirection, queue_size=1)
    lower_speed = HandSpeedDirection()
    lower_speed.speed = 0
    lower_speed.direction = Vector3()
    lower_speed_pub.publish(lower_speed)
    hand_enable_pub = rospy.Publisher('/hand_enable', Bool, queue_size=1)
    hand_enable_msg = Bool()
    hand_enable_msg.data = True 
    hand_enable_pub.publish(hand_enable_msg)
    hand_emergency_enable_pub = rospy.Publisher('/hand_emergency_enable', Bool, queue_size=1)
    hand_emergency_enable_msg = Bool()
    hand_emergency_enable_msg.data = False
    hand_emergency_enable_pub.publish(hand_emergency_enable_msg)
    hand_right_pub = rospy.Publisher('/hand_right_ref_pressure', Float64, queue_size=1)
    hand_right_msg = Float64()
    hand_right_msg.data = 0.0
    hand_right_pub.publish(hand_right_msg)
    hand_left_pub = rospy.Publisher('/hand_left_ref_pressure', Float64, queue_size=1)
    hand_left_msg = Float64()
    hand_left_msg.data = 0.0
    hand_left_pub.publish(hand_left_msg)

    is_in_peril = False

    myrobot.initialize_whole_pose()

    hand_pub.publish(hand_msg)

    while not rospy.is_shutdown():
        printb("############ Loop start ############")

        myrobot.set_container()

        # left arm (arm_index: 0)
        myrobot.set_arm_index(0)

        while True:
            printp("=== left arm start ===")
            if myrobot.is_in_peril():
                printr("now robot is in peril...")
                rospy.sleep(1)
                continue


            rospy.sleep(0.1)

            obj = myrobot.detect()

            printg("obj_cetner : {}".format(obj.center.uv)) 
            printg("object score: {}".format(obj.score))
            printg("contact : {}".format(obj.contact))

            is_detect_successed = False
            
            # 右半分にあるキャベツは無視する
            printp("x value : {}".format(obj.center_pose.pose.position.x))
            if obj.center_pose.pose.position.y >= -0.03  and obj.center_pose.pose.position.x > 1.1:
                is_detect_successed = True
            # 角にあるキャベツは無視 / 一時的な対応にしたい
            if obj.contact in [3, 6, 12, 9]:
                is_detect_successed = False 

            printy("is_detect_successed : {}".format(is_detect_successed))
            # approach 
            is_approach_successed = False
            if is_detect_successed and not myrobot.is_in_peril():
                is_approach_successed = myrobot.approach(obj)
            printy("is_approach_successed : {}".format(is_approach_successed))

            # pick
            is_pick_successed = False
            if is_approach_successed and not myrobot.is_in_peril():
                res = myrobot.calcurate_insertion()
                if res.success and not myrobot.is_in_peril():
                    is_pick_successed = myrobot.pick(res, obj.contact, down=down)
                else:
                    printr("no good cabbage...")
            printy("is_pick_successed : {}".format(is_pick_successed))

            if myrobot.is_in_peril():
                is_pick_successed = False

            if is_pick_successed:
                printp("=== left arm end ===")
                break

            if not is_approach_successed or not is_pick_successed:
                obj_center = obj.center.uv
                myrobot.add_exclusion_cabbage(obj_center[0], obj_center[1])
            else:
                myrobot.add_exclusion_cabbage(-1, -1)

            myrobot.initialize_whole_pose()

        # myrobot.initialize_current_pose(wait=False)
        myrobot.initialize_current_pose()
        hand_emergency_enable_msg.data = False
        hand_emergency_enable_pub.publish(hand_emergency_enable_msg)
        myrobot.clear_exclusion_cabbage()

        # right arm (arm_index: 1)
        myrobot.set_arm_index(1)

        while True:
            printp("=== right arm start ===")
            if myrobot.is_in_peril():
                printr("now robot is in peril...")
                rospy.sleep(1)
                continue


            rospy.sleep(0.1)

            obj = myrobot.detect()

            printg("obj_cetner : {}".format(obj.center.uv)) 
            printg("object score: {}".format(obj.score))
            printg("contact : {}".format(obj.contact))

            is_detect_successed = False
            
            # 左半分にあるキャベツは無視する
            if obj.center_pose.pose.position.y <= 0.03 and obj.center_pose.pose.position.x > 1.1:
                is_detect_successed = True 
            # 角にあるキャベツは無視 / 一時的な対応にしたい
            if obj.contact in [3, 6, 12, 9]:
                is_detect_successed = False 

            # approach 
            is_approach_successed = False
            if is_detect_successed and not myrobot.is_in_peril():
                is_approach_successed = myrobot.approach(obj)
            printy("is_approach_successed : {}".format(is_approach_successed))

            # pick
            is_pick_successed = False
            if is_approach_successed and not myrobot.is_in_peril():
                res = myrobot.calcurate_insertion()
                if res.success and not myrobot.is_in_peril():
                    is_pick_successed = myrobot.pick(res, obj.contact, down=down)
                else:
                    printr("no good cabbage...")
            printy("is_pick_successed : {}".format(is_pick_successed))

            if myrobot.is_in_peril():
                is_pick_successed = False

            if is_pick_successed:
                printp("=== right arm end ===")
                break

            if not is_approach_successed or not is_pick_successed:
                obj_center = obj.center.uv
                myrobot.add_exclusion_cabbage(obj_center[0], obj_center[1])
            else:
                myrobot.add_exclusion_cabbage(-1, -1)
 
            myrobot.initialize_whole_pose()

        myrobot.initialize_current_pose()
        hand_emergency_enable_msg.data = False
        hand_emergency_enable_pub.publish(hand_emergency_enable_msg)
        myrobot.clear_exclusion_cabbage()

        myrobot.delete_container()

        is_place_successed = False
        if is_pick_successed and not myrobot.is_in_peril():
            # myrobot.initialize_current_pose(cartesian_mode=True) # こっちだとスコアが低くて実行されなかった
            # myrobot.initialize_whole_pose()
            is_place_successed = myrobot.place()
        printy("is_place_successed : {}".format(is_place_successed))


        if myrobot.is_in_peril():
            printr("robot has got some error in this loop")
            continue
        printg("will initialize")

        myrobot.set_robot_status_good()
        myrobot.initialize_whole_pose()

        hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
        hand_msg = Float64MultiArray()
        hand_msg.data = [0.0, 0.0]
        hand_pub.publish(hand_msg)

        printg("initialized!!")
