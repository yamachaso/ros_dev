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

    def pick(self, object_name, grasps, c_eef_step=0.001, c_jump_threshold=0.0, manual_wait=False):
        rospy.logwarn("premove: true")
        pre_pose = self.current_eef_default_pose
        grasp_position = grasps[0].grasp_pose.pose.position # x, y are same among grasps
        apploach_desired_distance = grasps[0].pre_grasp_approach.desired_distance
        pre_pose.position.x = grasp_position.x
        pre_pose.position.y = grasp_position.y
        pre_pose.position.z =  grasp_position.z + apploach_desired_distance
        pre_pose.orientation =  grasps[0].grasp_pose.pose.orientation
        waypoints = [pre_pose]
        plan, plan_score = self.current_move_group.compute_cartesian_path(waypoints, c_eef_step, c_jump_threshold)
        if plan_score == 1:
            self.execute(plan, wait=True)
            post_pose = pre_pose
            post_pose.position.z -= apploach_desired_distance
            post_plan, post_plan_score = self.current_move_group.compute_cartesian_path([post_pose], c_eef_step, c_jump_threshold)
            print("post plan score:", post_plan_score)
            if manual_wait:
                # rospy.sleep(5)
                print("-" * 40)
                print("please input to approach")
                print("-" * 40)
                raw_input()
            else:
                rospy.sleep(5)
            self.execute(post_plan, wait=True)

            rospy.logerr("grab")
            hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
            hand_msg = Float64MultiArray()
            hand_msg.data = [0.8, 0.8]
            hand_pub.publish(hand_msg)
            rospy.logerr("grabed")

            self.current_move_group.attach_object(object_name)
            retreat_plan, retreat_plan_score = self.current_move_group.compute_cartesian_path([pre_pose], c_eef_step, c_jump_threshold)
            print("retreat plan score:", retreat_plan_score)
            if manual_wait:
                # rospy.sleep(5)
                print("-" * 40)
                print("please input to retreat")
                print("-" * 40)
                raw_input()
            else:
                rospy.sleep(5)
            self.execute(retreat_plan, wait=True)
            if manual_wait:
                print("-" * 40)
                print("please input to place")
                print("-" * 40)
                raw_input()
            else:
                rospy.sleep(5)

            hand_msg.data = [0.0, 0.0]
            hand_pub.publish(hand_msg)

            return True
        else:
            print("picking failed...")
            return False


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
        rospy.logerr("4.5")
        mv_base_to_left_arm = MoveGroup("base_and_left_arm", constraint=left_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        rospy.logerr("5")
        mv_body_to_left_arm = MoveGroup("body_and_left_arm", parent=mv_base_to_left_arm, constraint=left_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        rospy.logerr("6")
        mv_left_arm = MoveGroup("left_arm", parent=mv_body_to_left_arm, constraint=left_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        # right groups
        rospy.logerr("7")
        mv_base_to_right_arm = MoveGroup("base_and_right_arm", constraint=right_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        rospy.logerr("8")
        mv_body_to_right_arm = MoveGroup("body_and_right_arm", parent=mv_base_to_right_arm, constraint=right_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        rospy.logerr("9")
        mv_right_arm = MoveGroup("right_arm", parent=mv_body_to_right_arm, constraint=right_hand_constraint, support_surface_name=support_surface_name, planning_time=10)
        # whole group
        # TODO: constraintあてる
        rospy.logerr("10")
        mv_base_to_arms = MoveGroup("base_and_arms", support_surface_name=support_surface_name, planning_time=10)

        rospy.logerr("11")

        # start_mv = mv_body_to_left_arm if used_camera == "left_camera" else mv_body_to_right_arm
        # start_mv = mv_body_to_left_arm if used_camera == "left_camera" else mv_body_to_right_arm
        start_mv = mv_left_arm if used_camera == "left_camera" else mv_right_arm
        # self.mv_handler = MoveGroupHandler(mv_body_to_left_arm, mv_body_to_right_arm, start_mv, mv_base_to_arms)
        self.mv_handler = MoveGroupHandler(mv_left_arm, mv_right_arm, start_mv, mv_base_to_arms)

        rospy.logerr("6")

        self.gd_cli = GraspDetectionClient( 
            fps=fps, 
            image_topic=image_topic, 
            depth_topic=depth_topic,
            points_topic=points_topic,
            wait=wait
        )

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

    def pick(self, object_name, object_msg, 
             c_eef_step=0.01, c_jump_threshold=0.0,
             grasp_quality=1., approach_desired_distance=0.1, approach_min_distance=0.05, retreat_desired_distance=0.1, retreat_min_distance=0.05, manual_wait=False):
        obj_position_point = object_msg.center_pose.pose.position
        z = max(obj_position_point.z - object_msg.length_to_center / 2, 0.01)
        print("z: {}".format(z))
        obj_position_vector = Vector3(obj_position_point.x, obj_position_point.y, z)
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
        res = self.mv_handler.pick(object_name, grasps, c_eef_step, c_jump_threshold, manual_wait)
        return res, arm_index

    def place(self, arm_index, object_name, approach_desired_distance=0.1, approach_min_distance=0.05, retreat_desired_distance=0.01, retreat_min_distance=0.05):       
        place_position = (0, -1, 0.4)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = place_position
        res = self.mv_handler.place(object_name, pose)
        return res        

    def detect(self):
        return self.gd_cli.detect()

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
        rospy.logerr("loop wawa")
        rospy.sleep(0.1)
        # TODO: 作業完了したかのフラグ作って基準状態以外では検出が走らないようにしたい
        objects = myrobot.detect()
        print("objects: {}".format(len(objects)))
        if len(objects) == 0:
            continue

        scores = [obj.score for obj in objects]
        ordered_indexes = np.argsort(scores)[::-1] # 降順
        for target_index in ordered_indexes:
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
            is_pick_successed, arm_index = myrobot.pick(obj_name, obj,
                        grasp_quality=obj.score,
                        approach_desired_distance=insert_depth * 1.3,
                        retreat_desired_distance=insert_depth * 2,
                        approach_min_distance=insert_depth * 1.2,
                        retreat_min_distance= insert_depth * 1.2,
                        manual_wait=manual_wait
            )
            print("is_pick_successed : \033[92m{}\033[0m".format(is_pick_successed))

            hand_pub = rospy.Publisher('/hand_ref_pressure', Float64MultiArray, queue_size=1)
            hand_msg = Float64MultiArray()
            hand_msg.data = [0.0, 0.0]
            hand_pub.publish(hand_msg)

            print("pick result for the {}-th object: {}".format(target_index, is_pick_successed))
            is_place_successed = False
            if not grasp_only and is_pick_successed:
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
            print("will initialize")
            myrobot.initialize_whole_pose()
            if is_place_successed:
                break

        myrobot.scene_handler.remove_world_object(obj_name)
        ### myrobot.scene_handler.update_octomap()