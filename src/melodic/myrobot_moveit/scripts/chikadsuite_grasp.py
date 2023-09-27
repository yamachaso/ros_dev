#!/usr/bin/env python2
# coding: UTF-8

import sys
import math
import numpy as np
import rospy
from actionlib import SimpleActionClient
from std_msgs.msg import Header
import moveit_commander as mc
from moveit_msgs.msg import Grasp as BaseGrasp, Constraints, OrientationConstraint
from detect.msg import VisualizeTargetAction, VisualizeTargetGoal
from grasp_detection_client import GraspDetectionClient
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped, Pose
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

from octomap_handler import OctomapHandler
from std_msgs.msg import Float64MultiArray

from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class MoveGroup(mc.MoveGroupCommander):
    def __init__(self, name, parent=None, constraint=Constraints(), support_surface_name="", planning_time=5):
        super(MoveGroup, self).__init__(name)
        rospy.logerr("a")
        self.constraint = constraint
        self.parent = parent
        rospy.logerr("b")
        # self.next = mc.MoveGroupCommander(next_name)
        self.set_path_constraints(constraint)
        self.set_support_surface_name(support_surface_name)
        self.set_planning_time(planning_time)
        rospy.logerr("c")

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

    def approach(self, object_name, grasps, c_eef_step=0.001, c_jump_threshold=0.0, manual_wait=False):
        rospy.logwarn("approach_fucntion")
        pre_pose = self.current_eef_default_pose
        grasp_position = grasps[0].grasp_pose.pose.position # x, y are same among grasps
        apploach_desired_distance = grasps[0].pre_grasp_approach.desired_distance
        print("apploach_desired_distance : ", apploach_desired_distance)
        print(grasp_position)
        pre_pose.position.x = grasp_position.x
        pre_pose.position.y = grasp_position.y
        pre_pose.position.z =  grasp_position.z + apploach_desired_distance
        pre_pose.orientation =  grasps[0].grasp_pose.pose.orientation
        waypoints = [pre_pose]
        plan, plan_score = self.current_move_group.compute_cartesian_path(waypoints, c_eef_step, c_jump_threshold)
        print("plan_score", plan_score)
        if plan_score > 0.9: # TODO
            self.execute(plan, wait=True)

            # rospy.logerr("grab")
            # hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
            # hand_msg = Float64MultiArray()
            # hand_msg.data = [0.8, 0.8]
            # hand_pub.publish(hand_msg)
            # rospy.logerr("grabed")

            # self.current_move_group.attach_object(object_name)

            # hand_msg.data = [0.0, 0.0]
            # hand_pub.publish(hand_msg)

            return True
        else:
            print("approach failed...")
            return False

    def pick(self, object_name, target_pose, target_pressure, arm_index,c_eef_step=0.001, c_jump_threshold=0.0):
        rospy.logwarn("pick_fucntion")

        # print("\033[92m{}\033[0m".format("target_pressure"))
        # print("\033[92m{}\033[0m".format(target_pressure))

        hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
        hand_msg = Float64MultiArray()
        hand_msg.data = [target_pressure, target_pressure]
        hand_pub.publish(hand_msg)

        
        rospy.logerr(target_pressure)

        # pre_pose = self.current_eef_default_pose
        pre_pose = self.current_move_group.get_current_pose().pose
        pre_pose.position = target_pose.position #TODO TMP
        pre_pose.position.z -= 0.15
        pre_pose.orientation =  target_pose.orientation
        print("pre_pose : ", pre_pose)
        plan, plan_score = self.current_move_group.compute_cartesian_path([pre_pose], c_eef_step, c_jump_threshold)
        print("pre_pose position")
        print(pre_pose.position)
        print("pre_pose score", plan_score)
        if plan_score < 0.5:
            print("pick failed 1...")
            return False
        
        print("\033[92m{}\033[0m".format("pre pose"))
        self.execute(plan, wait=True)

        rospy.sleep(1)


        # これが大事？
        self.current_move_group.stop()
        self.current_move_group.clear_pose_targets()



        # pick_pose = self.current_move_group.get_current_pose()
        # print(self.current_move_group.get_end_effector_link())
        # end_effector_link = self.current_move_group.get_end_effector_link()
        # pick_pose.pose.position.z -= 0.1
        # print(pick_pose)
        # # move_group.set_position_target([1.5, -0.2, 0.2], end_effector_link="right_soft_hand_tip")


        # robot = mc.RobotCommander()
        # current_state = robot.get_current_state()
        # joint_state = JointState()
        # joint_state.name = current_state.joint_state.name
        # joint_state.position = current_state.joint_state.position

        # # RobotStateメッセージに新しいジョイント状態を設定
        # robot_state = RobotState()
        # robot_state.joint_state = joint_state

        # # set_start_stateを使用して新しいジョイント状態を設定
        # self.current_move_group.set_start_state(robot_state)

        # self.current_move_group.set_pose_target(pick_pose, end_effector_link=end_effector_link)

        # # 追記：実行可能かの確認
        # plan = self.current_move_group.plan()
        # if not plan.joint_trajectory.points:
        #     rospy.logerr("No motion plan found")

        # モーションプランの計画と実行
        # self.current_move_group.go(wait=True)

        # pick_pose.pose.position.z -= 0.2
        # print(pick_pose)
        # # move_group.set_position_target([1.5, -0.2, 0.2], end_effector_link="right_soft_hand_tip")
        # self.current_move_group.set_pose_target(pick_pose, end_effector_link=end_effector_link)
        # self.current_move_group.go(wait=True)

        # pick_pose.pose.position.z -= 0.0
        # print(pick_pose)
        # # move_group.set_position_target([1.5, -0.2, 0.2], end_effector_link="right_soft_hand_tip")
        # self.current_move_group.set_pose_target(pick_pose, end_effector_link=end_effector_link)
        # self.current_move_group.go(wait=True)

        # 後処理
        # self.current_move_group.stop()
        # self.current_move_group.clear_pose_targets()
    
            
###################################
        # pick_pose = pre_pose
        # print("pick_pose : ", pick_pose)
        # pick_pose.position = target_pose.position
        # pick_pose.position.z -= 0.1
        # # pick_pose.orientation =  target_pose.orientation
        # # self.current_move_group.set_pose_target(pick_pose)
        # self.current_move_group.set_num_planning_attempts(300)
        # print("pick_pose : ", pick_pose)
        # plan, plan_score = self.current_move_group.compute_cartesian_path([pick_pose], c_eef_step, c_jump_threshold)
        # print("pick score", plan_score)
        # self.current_move_group.set_num_planning_attempts(1)

        # if plan_score < 0.05:
        #     print("pick failed 2...")
        #     return False
        # print("\033[92m{}\033[0m".format("pick pose"))
        # self.execute(plan, wait=True)
        # print("\033[92m{}\033[0m".format("pick pose finished"))

###############################
        # move_group = mc.MoveGroupCommander("right_arm") # for open_manipulator
        # move_group.set_planner_id('RRTConnectkConfigDefault')
        # # ref: https://answers.ros.org/question/334902/moveit-control-gripper-instead-of-panda_link8-eff/
        # # move_group.set_end_effector_link("right_panda_hand_tip")
        # pick_pose = move_group.get_current_pose()
        # pick_pose.pose.position = target_pose.position
        # # pick_pose.pose.position.z -= 0.1
        # # move_group.set_position_target([1.5, -0.2, 0.2], end_effector_link="right_soft_hand_tip")
        # move_group.set_pose_target(pick_pose, end_effector_link="right_soft_hand_tip")

        # # 追記：実行可能かの確認
        # plan = move_group.plan()
        # # if not plan.joint_trajectory.points:
        # #     rospy.logerr("No motion plan found")

        # # モーションプランの計画と実行
        # move_group.go(wait=True)

        # move_group.stop()
        # move_group.clear_pose_targets()

        # print("\033[92m{}\033[0m".format("##########################################"))
        # pick_pose = move_group.get_current_pose()
        # pick_pose.pose.position.z -= 0.1
        # move_group.set_pose_target(pick_pose, end_effector_link="right_soft_hand_tip")
        # plan = move_group.plan()
        # move_group.go(wait=True)

        # # 後処理
        # move_group.stop()
        # move_group.clear_pose_targets()

        # print("\033[92m{}\033[0m".format("##########################################")

###############################



        # pick_pose = pre_pose
        # pick_pose.position = target_pose.position
        # pick_pose.position.z -= 0.1
        # end_effector_link = self.current_move_group.get_end_effector_link()
        # self.current_move_group.set_position_target(
        #     pick_pose.position, 
        #     end_effector_link=end_effector_link)
        # self.current_move_group.go(wait=True)

        # # 後処理
        # self.current_move_group.stop()
        # self.current_move_group.clear_pose_targets()



        rospy.logerr("grab")
        hand_msg.data = [1.2, 1.2]
        hand_pub.publish(hand_msg)
        rospy.logerr("grabed")

        rospy.sleep(3)

        self.current_move_group.attach_object(object_name)


        # post_pose = pre_pose
        post_pose = self.current_move_group.get_current_pose().pose
        print("post_pose : ", post_pose)
        post_pose.position.z += 0.2 #TODO ハードコード
        print("post_pose : ", post_pose)
        plan, plan_score = self.current_move_group.compute_cartesian_path([post_pose], c_eef_step, c_jump_threshold)
        if plan_score < 0.5:
            print("pick failed 3...")
            return False
        
        print("post_pose score", plan_score)
        print("\033[92m{}\033[0m".format("retreat!!"))
        self.execute(plan, wait=True)

        # hand_msg.data = [0.0, 0.0]
        # hand_pub.publish(hand_msg)

        return True


    def place(self, object_name, locations):
        self.initialize_current_pose(cartesian_mode=True, wait=True)
        self.set_current_move_group(self.current_move_group.parent, self.current_eef_default_pose) # tmp             
        return bool(self.current_move_group.place(object_name, locations))

    def get_current_name(self):
        return self.current_move_group.get_name()

class PlanningSceneHandler(mc.PlanningSceneInterface):
    def __init__(self, raw_point_topics):
        super(PlanningSceneHandler, self).__init__(synchronous=True)

        self.oh = OctomapHandler(raw_point_topics)

    def clear_octomap(self):
        self.oh.clear()

    def update_octomap(self):
        self.oh.update()

class Grasp(BaseGrasp):
    def __init__(self, grasp_quality, approach_desired_distance, approach_min_distance, 
                 retreat_desired_distance, retreat_min_distance, 
                 position=None, orientation=None, xyz=(0, 0, 0), rpy=(0, 0, 0), 
                 frame_id="base_link", finger_joints=[], allowed_touch_objects=[]):
        super(Grasp, self).__init__()
        # setting grasp-pose: this is for parent_link
        self.grasp_pose.header.frame_id = frame_id
        self.allowed_touch_objects = allowed_touch_objects
        self.grasp_quality = grasp_quality
        if position is None:
            position = Vector3(xyz[0], xyz[1], xyz[2])
        if orientation is None:
            q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.grasp_pose.pose.position = position
        self.grasp_pose.pose.orientation = orientation
        # setting pre-grasp approach
        self.pre_grasp_approach.direction.header.frame_id = frame_id
        self.pre_grasp_approach.direction.vector.z = -1
        self.pre_grasp_approach.min_distance = approach_min_distance
        self.pre_grasp_approach.desired_distance = approach_desired_distance
        # setting post-grasp retreat
        self.post_grasp_retreat.direction.header.frame_id = frame_id
        self.post_grasp_retreat.direction.vector.z = 1
        self.post_grasp_retreat.min_distance = retreat_min_distance
        self.post_grasp_retreat.desired_distance = retreat_desired_distance
        # setting posture of eef before grasp
        self.pre_grasp_posture.joint_names = finger_joints
        self.pre_grasp_posture.points = [JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Duration(2.0))]
        # self.grasp_posture.points = [JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Time(0)), JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Time(10))]
        # setting posture of eef during grasp
        self.grasp_posture.joint_names = finger_joints
        self.pre_grasp_posture.points = [JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Duration(2.0))]
        # self.grasp_posture.points = [JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Time(0)), JointTrajectoryPoint(positions=[0.0], time_from_start=rospy.Time(10))]



class Myrobot:
    def __init__(self, fps, image_topic, depth_topic, points_topic, raw_point_topics, wait = True, use_constraint = False, add_ground = True, used_camera = "left_camera"):
        mc.roscpp_initialize(sys.argv)

        self.robot = mc.RobotCommander()
        self.scene_handler = PlanningSceneHandler(raw_point_topics)

        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position =Vector3(0, -1, 0.2)
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        box_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        support_surface_name = "table"
        self.scene_handler.add_box(support_surface_name, box_pose, size=(0.5, 0.5, 0.4))
        if add_ground:
            plane_pose = PoseStamped()
            plane_pose.header.frame_id = "world"
            self.scene_handler.add_plane("ground plane", plane_pose)    

        # constraints
        if use_constraint:
            constraint_rpy = (0, math.pi, 0) # TODO: compute z from finger property (now 45 for 4 fingers)
            constraint_xyz_tolerance = (0.05, 0.05, 3.6) # TODO: update this value
            # constraint_xyz_tolerance = (0.08726, 0.08726, 6.28318) # TODO: update this value
            left_hand_constraint = self._create_constraint("left_hand_constraint", link_name="left_soft_hand_base_link", 
                                                        rpy=constraint_rpy, xyz_tolerance=constraint_xyz_tolerance)
            right_hand_constraint = self._create_constraint("right_hand_constraint", link_name="right_soft_hand_base_link", 
                                                        rpy=constraint_rpy, xyz_tolerance=constraint_xyz_tolerance)
        else:
            left_hand_constraint = Constraints()
            right_hand_constraint = Constraints()
        rospy.logerr("4")
        # 実機だとここで止まってしまうことがある。
        # myrobot_moveit/scripts/moveit_test.pyを一度実行するとなぜか直る
        # left groups
        # test_group = mc.MoveGroupCommander("right_arm")
        mv_base_to_left_arm = MoveGroup("base_and_left_arm", constraint=left_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        rospy.logerr("SUCCESS")
        mv_body_to_left_arm = MoveGroup("body_and_left_arm", parent=mv_base_to_left_arm, constraint=left_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        mv_left_arm = MoveGroup("left_arm", parent=mv_body_to_left_arm, constraint=left_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        # right groups
        mv_base_to_right_arm = MoveGroup("base_and_right_arm", constraint=right_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        mv_body_to_right_arm = MoveGroup("body_and_right_arm", parent=mv_base_to_right_arm, constraint=right_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        mv_right_arm = MoveGroup("right_arm", parent=mv_body_to_right_arm, constraint=right_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        # whole group
        # TODO: constraintあてる
        mv_base_to_arms = MoveGroup("base_and_arms", support_surface_name=support_surface_name, planning_time=10)


        # TMP
        # mv_right_arm.set_goal_joint_tolerance(0.07)
        # mv_right_arm.set_goal_orientation_tolerance(0.07)
        # mv_right_arm.set_goal_position_tolerance(0.001)
        # TMP end

        # start_mv = mv_body_to_left_arm if used_camera == "left_camera" else mv_body_to_right_arm
        # start_mv = mv_body_to_left_arm if used_camera == "left_camera" else mv_body_to_right_arm
        start_mv = mv_left_arm if used_camera == "left_camera" else mv_right_arm
        # self.mv_handler = MoveGroupHandler(mv_base_to_left_arm, mv_base_to_right_arm, start_mv, mv_base_to_arms)
        self.mv_handler = MoveGroupHandler(mv_left_arm, mv_right_arm, start_mv, mv_base_to_arms)

        self.gd_cli = GraspDetectionClient( 
            fps=fps, 
            image_topic=image_topic, 
            depth_topic=depth_topic,
            points_topic=points_topic,
            wait=wait
        )

        rospy.logerr("7")

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

    def get_around_octomap(self, values=[-30, 30, 0], sleep_time=0, is_degree=False, should_reset=True):
        if should_reset:
            self.scene_handler.clear_octomap()
        for value in values:
            plan = self.plan(joint_back=value, is_degree=is_degree)
            self.execute(plan, wait=True)
            rospy.sleep(sleep_time)
            self.scene_handler.update_octomap()
            rospy.sleep(sleep_time)

    def plan(self, joints={}, is_degree=False, **kwargs):
        return self.mv_handler.plan(joints, is_degree, **kwargs)

    def execute(self, plan, wait=False):
        res =  self.mv_handler.execute(plan, wait)
        return res

    def approach(self, object_name, object_msg, 
             c_eef_step=0.01, c_jump_threshold=0.0,
             grasp_quality=1., approach_desired_distance=0.1, approach_min_distance=0.05, retreat_desired_distance=0.1, retreat_min_distance=0.05, manual_wait=False):
        obj_position_point = object_msg.center_pose.pose.position
        # z = max(obj_position_point.z - object_msg.length_to_center / 2, 0.01)
        obj_position_vector = Vector3(obj_position_point.x, obj_position_point.y, obj_position_point.z) # キャベツの表面の位置
        # TODO: change grsp frame_id from "base_link" to each hand frame
        # arm_index = self.select_arm(obj_position_vector.y) # TODO: 一時的にコメントアウト
        # arm_index = 1
        arm_index = 0
        # if arm_index == 0:
            # return False, 0
        # if arm_index == 1:
        #     return False, 1
        # TMP: active jointでないといけない & 実機で存在しない関節を指定するとエラー & 空だとretreatが機能しない -> アームの関節を指定
        # finger_joints = ["left_finger_1_joint"] if arm_index == 0 else ["right_finger_1_joint"] 
        finger_joints = ["left_joint_6"] if arm_index == 0 else ["right_joint_6"] 
        grasps = [Grasp(
            position=obj_position_vector,
            rpy=(math.pi, 0, np.radians(object_msg.angle)),
            grasp_quality=grasp_quality,
            approach_desired_distance=approach_desired_distance,
            approach_min_distance=approach_min_distance,
            retreat_desired_distance=retreat_desired_distance,
            retreat_min_distance=retreat_min_distance,
            finger_joints=finger_joints,
            allowed_touch_objects=[object_name]
        )]
        res = self.mv_handler.approach(object_name, grasps, c_eef_step, c_jump_threshold, manual_wait)
        return res, arm_index
    
    def pick(self, object_name, target_pose, target_angle, target_pressure, arm_index, c_eef_step=0.01, c_jump_threshold=0.0):
        # obj_position_point = target_pose.position

        # obj_position_vector = Vector3(obj_position_point.x, obj_position_point.y, obj_position_point.z)

        rpy=(math.pi, 0, np.radians(target_angle))
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        pose = Pose(position=target_pose.position, orientation=orientation)
        
        print("in PICK")
        print(target_pose.position)


        res = self.mv_handler.pick(object_name, pose, target_pressure, arm_index, c_eef_step, c_jump_threshold)
        return res
        

    def place(self, arm_index, object_name, approach_desired_distance=0.1, approach_min_distance=0.05, retreat_desired_distance=0.01, retreat_min_distance=0.05):       
        place_position = (0, -1, 0.4)
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
    manual_wait = rospy.get_param("manual_wait", default=False)
    used_camera = rospy.get_param("used_camera", default="left_camera")

    rospy.loginfo("################################################")

    print("waiting for image topics")
    rospy.logerr("################################################")

    rospy.wait_for_message(image_topic, Image)
    rospy.wait_for_message(depth_topic, Image)
    rospy.logerr("#fefefefeffefeff")

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


    hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
    hand_msg = Float64MultiArray()
    hand_msg.data = [0.0, 0.0]
    hand_pub.publish(hand_msg)

    while not rospy.is_shutdown():
        rospy.logerr("loop start")
        rospy.sleep(0.1)
        # TODO: 作業完了したかのフラグ作って基準状態以外では検出が走らないようにしたい
        objects = myrobot.detect()
        print("objects: {}".format(len(objects)))
        if len(objects) == 0:
            continue

        scores = [obj.score for obj in objects]
        print("scores : ", scores)
        ordered_indexes = np.argsort(scores)
        # ordered_indexes = np.argsort(scores)[::-1] # 降順
        target_index = ordered_indexes[0]

        rospy.logerr("a1")
        obj = objects[target_index]
        obj_name = "object_{}".format(len(registered_objects))
        # TMP: ズレの補正
        obj.center_pose.pose.position.y -= 0.01

        # visualize target
        vis_cli.send_goal(VisualizeTargetGoal(obj.index))
        
        # add object
        obj_pose = obj.center_pose
        # obj_pose.pose.position.z -= obj.length_to_center / 2
        obj_pose.pose.orientation = Quaternion()
        # obj.length_to_center = obj.length_to_center * 1.3
        insert_depth = obj.length_to_center
        # TMP: 検出対象の外接矩形の半径からradiusを求めていたが若干精度悪い気がするので一旦固定値にしている
        myrobot.scene_handler.add_cylinder(obj_name, obj_pose, height=obj.length_to_center, radius=collision_radius * 0.6)
        ### myrobot.scene_handler.update_octomap()

        
        # pick
        print("try to pick {}-th object | score: {}".format(target_index, obj.score))
        # TODO: pull up arm index computation from pick
        is_approach_successed, arm_index = myrobot.approach(obj_name, obj,
                    grasp_quality=obj.score,
                    approach_desired_distance=insert_depth * 1.0, ## 重要
                    retreat_desired_distance=insert_depth * 2,
                    approach_min_distance=insert_depth * 1.2,
                    retreat_min_distance= insert_depth * 1.2,
                    manual_wait=manual_wait
        )
        print("is_approach_successed : \033[92m{}\033[0m".format(is_approach_successed))

        if is_approach_successed:
            print("\033[92m{}\033[0m".format("aaaa"))
            res = myrobot.calcurate_insertion()
            
            if res.success:
                myrobot.pick(obj_name, res.pose, res.angle, res.pressure, arm_index, c_eef_step=0.01, c_jump_threshold=0.0)
            else:
                print("\033[92m{}\033[0m".format("no good cabbage..."))
            print("\033[92m{}\033[0m".format("cccc"))
            


        hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
        hand_msg = Float64MultiArray()
        hand_msg.data = [0.0, 0.0]
        hand_pub.publish(hand_msg)

        print("pick result for the {}-th object: {}".format(target_index, is_approach_successed))
        is_place_successed = False
        if not grasp_only and is_approach_successed:
            myrobot.initialize_current_pose(cartesian_mode=True)
            print("try toc place {}-th object".format(target_index))
            is_place_successed = myrobot.place(arm_index, obj_name)
            print("place result for the {}-th object: {}".format(target_index, is_place_successed))
            # print("will initialize")
            # myrobot.initialize_whole_pose()
            # break
        # else:
            # print("will initialize")
            # myrobot.initialize_current_pose(cartesian_mode=True)

        link = "left_soft_hand_tip" if arm_index == 0 else "right_soft_hand_tip"
        myrobot.scene_handler.remove_attached_object(link) 
        # print("will initialize")
        print("\033[92m{}\033[0m".format("will initialize"))

        myrobot.initialize_whole_pose()
        if is_place_successed:
            break

        print("\033[92m{}\033[0m".format("initialized!!"))


        myrobot.scene_handler.remove_world_object(obj_name)
        ### myrobot.scene_handler.update_octomap()