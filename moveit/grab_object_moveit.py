#!/usr/bin/env python  
import rospy
import math
import tf
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Point,
    Quaternion,
    Vector3,
)
from visualization_msgs.msg import (
    Marker
)
import ipdb
import copy
import util

import sys,os
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import  PlanningScene, ObjectColor
import numpy as np
import pandas as pd
from birl_baxter_dmp.dmp_generalize import dmp_imitate
from trac_ik_baxter.srv import *
import pydmps
import pydmps.dmp_discrete
from birl_baxter_motion_generation.train_demontration_for_w import get_parameter_w
from birl_baxter_motion_generation.add_models import Addmodels

DIR_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = DIR_PATH + "/baxter_motion_sets/move_cart.txt"
TRAIN_LEN = 50
TAU = 100.0/TRAIN_LEN # set tau to 1 will give you 100 points
LIMB_NAME = "left"
GROUP_NAME_ARM = 'left_arm'
N_Demo=7   # number of DOFs
N_BFS=500  #number of basis function
CAMERA_FLAGE = True
REFERENCE_FRAME = 'base'

def get_base_camera_pose(flag):
    base_req = get_poseRequest()
    base_pose_client = rospy.ServiceProxy('get_pose_from_base_camera', get_pose, persistent=True)
    rospy.wait_for_service('get_pose_from_base_camera')
    rospy.loginfo("Geting pose service is ready")
    base_req.flag = flag
    marker_pose = base_pose_client(base_req)
    return [marker_pose.pose.position.x, marker_pose.pose.position.y, marker_pose.pose.position.z,
            marker_pose.pose.orientation.x, marker_pose.pose.orientation.y,
            marker_pose.pose.orientation.z, marker_pose.pose.orientation.w]

def istargetchanged():    
    return True

    
class MoveItP_n_P:   
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)   
        rospy.init_node('dmp_moveit_test_node',anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        
        # publish a string of traj for visuallizing
        marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
     
        dmp_weight,demo_start_pose,demo_end_pose = get_parameter_w(limb_name=LIMB_NAME,data_path=DATA_PATH,train_len=TRAIN_LEN)
        reference_frame = REFERENCE_FRAME
        print "add models"        
        #Addmodels(scene,reference_frame)
#        ipdb.set_trace()   
        print "============ Moving to the beginning of traj"
        traj_start = geometry_msgs.msg.Pose()    
        traj_start.position.x = demo_start_pose[0]
        traj_start.position.y = demo_start_pose[1]
        traj_start.position.z = demo_start_pose[2]
        traj_start.orientation.x = demo_start_pose[3]
        traj_start.orientation.y = demo_start_pose[4]
        traj_start.orientation.z = demo_start_pose[5]
        traj_start.orientation.w = demo_start_pose[6]
        
        group.set_start_state_to_current_state()
        group.set_pose_target(demo_start_pose)
        ipdb.set_trace()
        #ask RVIZ to visualize a plan
        plan_to_start = group.plan()
        print "============ Waiting while RVIZ displays plan_to_start..."
        rospy.sleep(2)  
        group.go(wait=True)
        group.clear_pose_targets()
        
        end_effector_link = group.get_end_effector_link()
        print("end_effector_link",end_effector_link)
        group.set_goal_position_tolerance(0.01)
        group.set_goal_orientation_tolerance(0.05)
        group.allow_replanning(True)
        
        group.set_pose_reference_frame(reference_frame)
        group.set_planning_time(5)
        
        current_pose = group.get_current_pose(end_effector_link).pose
        current_pose = [current_pose.position.x,current_pose.position.y,current_pose.position.z,
                        current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,
                        current_pose.orientation.w]
        #target_pose = get_base_camera_pose(flag=camera_flag)
        target_pose = demo_end_pose
        
        dmp_traj = dmp_imitate(starting_pose=current_pose,ending_pose=target_pose,weight_mat=dmp_weight,tau=TAU)
        rospy.loginfo("get dmp generalization traj")
        
        list_of_poses = []
        ipdb.set_trace()
        for idx in range(TRAIN_LEN):
            traj_pose = Pose()
            traj_pose.position.x = dmp_traj[idx, 0]
            traj_pose.position.y = dmp_traj[idx, 1]
            traj_pose.position.z = dmp_traj[idx, 2]
            traj_pose.orientation.x = dmp_traj[idx, 3]
            traj_pose.orientation.y = dmp_traj[idx, 4]
            traj_pose.orientation.z = dmp_traj[idx, 5]
            traj_pose.orientation.w = dmp_traj[idx, 6]
            # visualize the pose in rviz using marker
            alpha = float(idx) / TRAIN_LEN * 0.5 + 0.5
            rgba_tuple = ((0.5 * idx), 0.5, (0.5 * idx), alpha)
            util.send_traj_point_marker(marker_pub=marker_pub, pose=traj_pose, id=idx, rgba_tuple=rgba_tuple)
            rospy.loginfo("add one pose")
            list_of_poses.append(copy.deepcopy(traj_pose))
        
        rospy.loginfo("============ Generating dmp trajectory plan")
        (plan, fraction) = group.compute_cartesian_path(
            list_of_poses,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        print "============ Waiting while RVIZ displays dmp plans.."    
        group.plan()
        rospy.sleep(2)
        rospy.loginfo("============ Enter \"ok\" to execute plan, \"no\" to cancel execute plan ")
        s = raw_input()
        if s == 'ok':
            rospy.loginfo("============ Gonna execute plan")
            group.execute(plan)
            rospy.loginfo("============ Done")
        if s == 'no':
            rospy.loginfo("============ already cancel")
                # Exit MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script        
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItP_n_P()
    except KeyboardInterrupt:
        raise
