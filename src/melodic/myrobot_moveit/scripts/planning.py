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
    def __init__(self, name, planning_time=5):
        super(MoveGroup, self).__init__(name)
        self.set_planning_time(planning_time)

    # ジョイント名をキー, 関節角度値を値とした辞書
    def get_current_joint_dict(self):
        return dict(zip(self.get_active_joints(), self.get_current_joint_values()))

class MoveGroupHandler:
    def __init__(self, left_start_move_group, right_start_move_group, start_move_group, whole_move_group):
        self.left_start_move_group = left_start_move_group
        self.left_eef_default_pose = left_start_move_group.get_current_pose().pose
        self.right_start_move_group = right_start_move_group
        self.right_eef_default_pose = right_start_move_group.get_current_pose().pose
        self.start_move_group = start_move_group
        self.start_eef_default_pose = start_move_group.get_current_pose().pose
        self.whole_move_group = whole_move_group
        self.whole_name = whole_move_group.get_name()
        self.set_current_move_group(self.start_move_group, self.start_eef_default_pose)

        self.is_in_peril = False
        self.sub = rospy.Subscriber("/is_in_peril", Bool, self.set_peril)

        self.mv_right_arm = MoveGroup("back_and_right_arm")

    def set_peril(self, msg):
        self.is_in_peril = msg.data

    def set_current_move_group(self, move_group, default_eef_pose=None):
        self.current_move_group = move_group
        self.current_eef_default_pose = default_eef_pose
        rospy.loginfo("current move group is changed to '{}'".format(self.get_current_name()))

    def reset_move_group(self):
        # TODO: also reset joint values
        self.current_move_group = self.start_move_group
        self.current_eef_default_pose = self.start_eef_default_pose
        
    def initialize_current_pose(self, cartesian_mode=False, c_eef_step=0.01, c_jump_threshold=0.0, wait=True, plan_only=True):
        group_name = self.get_current_name()
        # target_name = "{}_default".format(group_name)
        target_name = "{}_start".format(group_name)
        target_joint_dict = self.current_move_group.get_named_target_values(target_name)
        if cartesian_mode:
            waypoints = [self.current_eef_default_pose]
            plan, _ = self.current_move_group.compute_cartesian_path(waypoints, c_eef_step, c_jump_threshold)
        else:
            plan = self.current_move_group.plan(target_joint_dict)
        if not plan_only:
            self.current_move_group.execute(plan, wait=wait)
        return plan

    def initialize_whole_pose(self, wait=True, plan_only=False):
        # target_name = "{}_default".format(self.whole_name)
        target_name = "{}_start".format(self.whole_name)
        target_joint_dict = self.whole_move_group.get_named_target_values(target_name)
        plan = self.whole_move_group.plan(target_joint_dict)
        if not plan_only:
            self.whole_move_group.execute(plan, wait=wait)
        return plan


    def plan(self, joints={}, is_degree=False, **kwargs):
        # if plan failed, switch move_group & plan again
        new_joints = joints
        new_joints.update(kwargs)
        if is_degree:
            new_joints = { k:np.radians(v)  for k,v in new_joints.items()}

        merged_joints = self.whole_move_group.get_current_joint_dict()
        merged_joints.update(new_joints)
        
        return self.whole_move_group.plan(merged_joints)

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

        # pre_pose = self.current_eef_default_pose
        # grasp_position = grasps[0].grasp_pose.pose.position # x, y are same among grasps
        # apploach_desired_distance = grasps[0].pre_grasp_approach.desired_distance
        # pre_pose.position.x = grasp_position.x
        # pre_pose.position.y = grasp_position.y
        # pre_pose.position.z =  grasp_position.z + apploach_desired_distance
        # pre_pose.orientation =  grasps[0].grasp_pose.pose.orientation

        # pre_pose = target_pose
        # pre_pose.position.z += approach_desired_distance
        # waypoints = [pre_pose]
        printr("target_pose : ")
        printr(target_pose)

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


    def pick(self, target_pose, access_distance, target_pressure, z_direction, c_eef_step=0.001, c_jump_threshold=0.0):
        printb("pick planning start")

        printb("hand adjustment execution")
        if self.is_in_peril:
                return False
        hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
        hand_msg = Float64MultiArray()
        hand_msg.data = [0.0, target_pressure] # 右アームのみ
        hand_pub.publish(hand_msg)

        printc("target_pressure : {}".format(target_pressure))


        # pre_pose = self.current_eef_default_pose
        # pre_pose = self.current_move_group.get_current_pose().pose
        # pre_pose.position = target_pose.position #TODO TMP
        # pre_pose.orientation =  target_pose.orientation
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

        print("##################################")

        # 先に位置姿勢と取得して、その後コントローラーを切り替える
        # さもないと指令加速度過大になりがち
        startup_pub = rospy.Publisher('/startup/right', Empty, queue_size=1)
        empty_msg = Empty()
        startup_pub.publish(empty_msg)


        if self.is_in_peril:
                return False
        try:
            call("/myrobot/right_arm/controller_manager/switch_controller", SwitchController,
                start_controllers=["right_cartesian_motion_controller"],
                stop_controllers=["right_arm_controller"],
                strictness=1, start_asap=False, timeout=0.0)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

            
        

        # 把持時のaccess_distanceとはハンドとキャベツの距離(m単位)
        # move_time = 1.35 - access_distance / 0.03 * 0.1
        move_time = 1.35 - access_distance * 0.01 

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
        hand_msg.data = [0.0, 1.6]
        hand_pub.publish(hand_msg)
        printb("grabed")

        rospy.sleep(1.5)

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

        rospy.sleep(move_time + 0.05)


        try:
            call("/myrobot/right_arm/controller_manager/switch_controller", SwitchController,
                start_controllers=["right_arm_controller"],
                stop_controllers=["right_cartesian_motion_controller"],
                strictness=1, start_asap=False, timeout=0.0)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        print("##################################")

        rospy.sleep(0.5)

        return True


    def place(self, object_name, target_pose, c_eef_step=0.001, c_jump_threshold=0.0):

        target_joint_dict = self.mv_right_arm.get_named_target_values("back_and_right_arm_place")
        plan = self.mv_right_arm.plan(target_joint_dict)



        printb("pre place execution")
        if self.is_in_peril:
                return False
        self.current_move_group.execute(plan, wait=True)

        if self.is_in_peril:
                return False
        hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
        hand_msg = Float64MultiArray()
        hand_msg.data = [0, 0]
        hand_pub.publish(hand_msg)

        hand_enable_pub = rospy.Publisher('/hand_enable', Bool, queue_size=1)
        hand_enable_msg = Bool()
        hand_enable_msg.data = False 
        hand_enable_pub.publish(hand_enable_msg)

        rospy.sleep(2)


        target_joint_dict = self.mv_right_arm.get_named_target_values("back_and_right_arm_start")
        plan = self.mv_right_arm.plan(target_joint_dict)
        printb("post place execution")
        if self.is_in_peril:
                return False
        self.current_move_group.execute(plan, wait=True)

        return True

    def get_current_name(self):
        return self.current_move_group.get_name()



class ContactOrientationController:
    def __init__(self):
        self.angle = 10
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
    def __init__(self, fps, image_topic, depth_topic, points_topic, raw_point_topics, wait = True, use_constraint = False, add_ground = False, used_camera = "left_camera"):
        mc.roscpp_initialize(sys.argv)

        self.robot = mc.RobotCommander()
        self.scene_handler = mc.PlanningSceneInterface(synchronous=True)
        # self.scene_handler = PlanningSceneHandler(raw_point_topics)

        # box_pose = PoseStamped()
        # box_pose.header.frame_id = "world"
        # box_pose.pose.position =Vector3(0, -1, 0.2)
        # q = quaternion_from_euler(0.0, 0.0, 0.0)
        # box_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        # support_surface_name = "table"
        # self.scene_handler.add_box(support_surface_name, box_pose, size=(0.5, 0.5, 0.4))
#         if add_ground:
#             plane_pose = PoseStamped()
#             plane_pose.header.frame_id = "world"
#             self.scene_handler.add_plane("ground plane", plane_pose)    

        # constraints
        # if use_constraint:
        #     constraint_rpy = (0, math.pi, 0) # TODO: compute z from finger property (now 45 for 4 fingers)
        #     constraint_xyz_tolerance = (0.05, 0.05, 3.6) # TODO: update this value
        #     # constraint_xyz_tolerance = (0.08726, 0.08726, 6.28318) # TODO: update this value
        #     left_hand_constraint = self._create_constraint("left_hand_constraint", link_name="left_soft_hand_base_link", 
        #                                                 rpy=constraint_rpy, xyz_tolerance=constraint_xyz_tolerance)
        #     right_hand_constraint = self._create_constraint("right_hand_constraint", link_name="right_soft_hand_base_link", 
        #                                                 rpy=constraint_rpy, xyz_tolerance=constraint_xyz_tolerance)
        # else:
        # left_hand_constraint = Constraints()
        # right_hand_constraint = Constraints()

        mv_base_to_left_arm = MoveGroup("base_and_left_arm", planning_time=10)
        mv_body_to_left_arm = MoveGroup("body_and_left_arm", planning_time=10)
        mv_left_arm = MoveGroup("left_arm", planning_time=10)
        # right groups
        mv_base_to_right_arm = MoveGroup("base_and_right_arm", planning_time=10)
        mv_body_to_right_arm = MoveGroup("body_and_right_arm", planning_time=10)
        mv_right_arm = MoveGroup("right_arm", planning_time=10)
        # whole group
        # TODO: constraintあてる
        mv_base_to_arms = MoveGroup("base_and_arms", planning_time=10)

        #############
        mv_back_and_arm = MoveGroup("back_and_right_arm", planning_time=10)
        #############
 
        printg("movegroup load : SUCCESS")


        # start_mv = mv_body_to_left_arm if used_camera == "left_camera" else mv_body_to_right_arm
        # start_mv = mv_body_to_left_arm if used_camera == "left_camera" else mv_body_to_right_arm
        start_mv = mv_left_arm if used_camera == "left_camera" else mv_right_arm
        # self.mv_handler = MoveGroupHandler(mv_base_to_left_arm, mv_base_to_right_arm, start_mv, mv_base_to_arms)
        # self.mv_handler = MoveGroupHandler(mv_left_arm, mv_right_arm, start_mv, mv_base_to_arms)
        self.mv_handler = MoveGroupHandler(mv_left_arm, mv_right_arm, start_mv, mv_right_arm)

        self.gd_cli = GraspDetectionClient( 
            fps=fps, 
            image_topic=image_topic, 
            depth_topic=depth_topic,
            points_topic=points_topic,
            wait=wait
        )

        self.cp_cli = ContainerPositionClient()

        self.el_cli = ExclusionListClient()

    def is_in_peril(self):
        return self.mv_handler.is_in_peril
    
    def set_robot_status_good(self):
        self.mv_handler.is_in_peril = False

    def _create_constraint(self, name, link_name, rpy, base_frame_id="base_link", xyz_tolerance=(0.05, 0.05, 3.6)):
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        constraint = Constraints(
            name=name,
            orientation_constraints = [OrientationConstraint(
                header=Header(frame_id=base_frame_id),
                link_name=link_name,
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
                # allow max rotations
                absolute_x_axis_tolerance = xyz_tolerance[0],
                absolute_y_axis_tolerance = xyz_tolerance[1],
                absolute_z_axis_tolerance = xyz_tolerance[2],
                weight = 1
            )]
        )
        return constraint

    def initialize_current_pose(self, cartesian_mode=False, c_eef_step=0.01, c_jump_threshold=0.0):
        self.mv_handler.initialize_current_pose(cartesian_mode, c_eef_step, c_jump_threshold)
        self.mv_handler.reset_move_group()

    def initialize_whole_pose(self):
        self.mv_handler.initialize_whole_pose()
        self.mv_handler.reset_move_group()

    # def get_around_octomap(self, values=[-30, 30, 0], sleep_time=0, is_degree=False, should_reset=True):
    #     if should_reset:
    #         self.scene_handler.clear_octomap()
    #     for value in values:
    #         plan = self.plan(joint_back=value, is_degree=is_degree)
    #         self.execute(plan, wait=True)
    #         rospy.sleep(sleep_time)
    #         self.scene_handler.update_octomap()
    #         rospy.sleep(sleep_time)

    def plan(self, joints={}, is_degree=False, **kwargs):
        return self.mv_handler.plan(joints, is_degree, **kwargs)

    def execute(self, plan, wait=False):
        res =  self.mv_handler.execute(plan, wait)
        return res

    def approach(self, object_msg, c_eef_step=0.01, c_jump_threshold=0.0):
        arm_index = 1
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
        return res, arm_index
    
    def pick(self, res_msg, contact, c_eef_step=0.01, c_jump_threshold=0.0):
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


        print("in PICK")
        print(target_pose.position)


        res = self.mv_handler.pick(target_pose, access_distance, target_pressure, coc.z_direction[contact], c_eef_step, c_jump_threshold)
        return res
        

    def place(self, arm_index, object_name, approach_desired_distance=0.1, approach_min_distance=0.05, retreat_desired_distance=0.01, retreat_min_distance=0.05):       
        # place_position = (1.1, -0.63, 0.78)
        place_position = (1.35, -0.63, 0.55)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = place_position
        res = self.mv_handler.place(object_name, pose)
        return res        

    def detect(self):
        return self.gd_cli.detect()
    
    def calcurate_insertion(self):
        return self.gd_cli.calcurate_insertion()

    def select_arm(self, y):
        # 0: left, 1: right
        arm_index =  0 if y > 0 else 1
        print("y: {}, arm_index: {}".format(y, arm_index))
        if arm_index == 0:
            new_move_group = self.mv_handler.left_start_move_group
            new_eef_default_pose = self.mv_handler.left_eef_default_pose
        else:
            new_move_group = self.mv_handler.right_start_move_group
            new_eef_default_pose = self.mv_handler.right_eef_default_pose

        self.mv_handler.set_current_move_group(new_move_group, new_eef_default_pose)
        return arm_index
    
    def set_container(self):
        # listener = tf.TransformListener()
        # (trans, rot) = listener.lookupTransform('/base_link', '/container_base', rospy.Time(0))

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
    image_topic = rospy.get_param("image_topic")
    depth_topic = rospy.get_param("depth_topic")
    points_topic = rospy.get_param("points_topic")
    sensors = rospy.get_param("sensors", default=("left_camera", "right_camera", "body_camera"))
    grasp_only = rospy.get_param("grasp_only", default="false")
    raw_point_topics = ["/{}/{}/depth/color/points".format(ns, sensor_name) for sensor_name in sensors]

    wait = rospy.get_param("wait_server", default=True)
    use_constraint = rospy.get_param("use_constraint", default=False)
    used_camera = rospy.get_param("used_camera", default="left_camera")

    rospy.loginfo("################################################")

    print("waiting for image topics")
    rospy.logerr("################################################")

    rospy.wait_for_message(image_topic, Image)
    rospy.wait_for_message(depth_topic, Image)

    print("initializing instances...")
    myrobot = Myrobot(fps=fps, image_topic=image_topic, depth_topic=depth_topic, points_topic=points_topic, 
                      raw_point_topics=raw_point_topics, wait=wait, use_constraint=use_constraint, add_ground=True,
                      used_camera=used_camera)
    myrobot.info()
    # hand_radius_mmはdetect側のlaunchで設定しているのでwait後に読み込み
    hand_radius_mm = rospy.get_param("hand_radius_mm", default="157.5")
    collision_radius = hand_radius_mm / 1000 * 0.5

    print("waiting for visualize server")
    vis_cli = SimpleActionClient("visualize_server_draw_target", VisualizeTargetAction)
    vis_cli.wait_for_server()

    print("initializing pose...")
    myrobot.initialize_whole_pose()
    print("getting around octomap...")
    ### myrobot.get_around_octomap(values=[-30, 30, 0], sleep_time=1.0, is_degree=True, should_reset=True)

    print("stating detect flow...")
    registered_objects = []


    # 最初のメッセージがなぜか無視されるので、ダミーで一回パブリッシュ
    hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
    hand_msg = Float64MultiArray()
    hand_msg.data = [0.0, 0.0]
    hand_pub.publish(hand_msg)
    # 同様の理由でここでもパブリッシュ
    startup_pub = rospy.Publisher('/startup/right', Empty, queue_size=1)
    empty_msg = Empty()
    startup_pub.publish(empty_msg)
    lower_speed_pub = rospy.Publisher('/target_hand_lower_speed', HandSpeedDirection, queue_size=1)
    lower_speed = HandSpeedDirection()
    lower_speed.speed = 0
    lower_speed.direction = Vector3()
    lower_speed_pub.publish(lower_speed)
    hand_enable_pub = rospy.Publisher('/hand_enable', Bool, queue_size=1)
    hand_enable_msg = Bool()
    hand_enable_msg.data = True 
    hand_enable_pub.publish(hand_enable_msg)

    is_in_peril = False

    while not rospy.is_shutdown():
        if myrobot.is_in_peril():
            printr("now robot is in peril...")
            rospy.sleep(1)
            continue
        rospy.logerr("loop start")

        myrobot.set_container()

        rospy.sleep(0.1)
        # TODO: 作業完了したかのフラグ作って基準状態以外では検出が走らないようにしたい
        # object = myrobot.detect()
        # print("objects: {}".format(len(objects)))
        # if len(objects) == 0:
        #     continue

        # scores = [obj.score for obj in objects]
        # print("scores : ", scores)
        # ordered_indexes = np.argsort(scores)
        # # ordered_indexes = np.argsort(scores)[::-1] # 降順
        # target_index = ordered_indexes[0]

        # obj = objects[target_index]
        obj = myrobot.detect()
        obj_name = "object_{}".format(len(registered_objects))
        # TMP: ズレの補正
        # obj.center_pose.pose.position.y -= 0.01

        # visualize target
        # vis_cli.send_goal(VisualizeTargetGoal(obj.index))
        obj_center = obj.center.uv
        printg("obj_cetner : {}".format(obj_center)) 
        # add object
        obj_pose = obj.center_pose
        # obj_pose.pose.position.z -= obj.length_to_center / 2
        obj_pose.pose.orientation = Quaternion()
        # obj.length_to_center = obj.length_to_center * 1.3
        insert_depth = obj.length_to_center
        # TMP: 検出対象の外接矩形の半径からradiusを求めていたが若干精度悪い気がするので一旦固定値にしている
        # myrobot.scene_handler.add_cylinder(obj_name, obj_pose, height=obj.length_to_center, radius=collision_radius * 0.6)
        ### myrobot.scene_handler.update_octomap()

        is_detect_successed = False
        printy(obj_pose.pose.position.y)
        
        # 右アームは左半分位あるキャベツを無視する
        if obj_pose.pose.position.y < 0:
            is_detect_successed = True 

        printy("contact : {}".format(obj.contact))

        # pick
        print("try to pick | score: {}".format(obj.score))
        # TODO: pull up arm index computation from pick
        is_approach_successed = False
        arm_index = -1
        if is_detect_successed and not myrobot.is_in_peril():
            is_approach_successed, arm_index = myrobot.approach(obj)
        printy("is_approach_successed : {}".format(is_approach_successed))

        is_pick_successed = False
        if is_approach_successed and not myrobot.is_in_peril():
            res = myrobot.calcurate_insertion()
            if res.success and not myrobot.is_in_peril():
                # is_pick_successed = myrobot.pick(res.pose, res.angle, res.distance, res.pressure)
                is_pick_successed = myrobot.pick(res, obj.contact)
            else:
                printr("no good cabbage...")
        printy("is_pick_successed : {}".format(is_pick_successed))
            
        print("pick result : {}".format(is_approach_successed))

        myrobot.delete_container()

        is_place_successed = False
        if is_pick_successed and not myrobot.is_in_peril():
            # myrobot.initialize_current_pose(cartesian_mode=True) # こっちだとスコアが低くて実行されなかった
            myrobot.initialize_whole_pose()
            is_place_successed = myrobot.place(arm_index, obj_name)
            print("place result : {}".format(is_place_successed))
        printy("is_place_successed : {}".format(is_place_successed))


        link = "left_soft_hand_tip" if arm_index == 0 else "right_soft_hand_tip"
        # myrobot.scene_handler.remove_attached_object(link) 


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


        if not is_approach_successed or not is_pick_successed:
            print(obj_center)
            myrobot.el_cli.add(obj_center[0], obj_center[1])

        printg("initialized!!")

        # myrobot.scene_handler.remove_world_object(obj_name)
        ### myrobot.scene_handler.update_octomap()
