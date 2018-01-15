#!/usr/bin/env python  
import rospy
import math
import tf
from geometry_msgs.msg import (
    Pose,
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
import numpy as np
import pandas as pd
from birl_baxter_dmp.dmp_generalize import dmp_imitate
from trac_ik_baxter.srv import *
import pydmps
import pydmps.dmp_discrete
from birl_baxter_motion_generation.train_demontration_for_w import get_parameter_w

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = dir_path + "/baxter_motion_sets/move_cart.txt"
train_len = 50
tau = 100.0/train_len # set tau to 1 will give you 100 points
limb_name = "left"
plan_arm = "left_arm"
n_dmps=7   # number of DOFs
n_bfs=500  #number of basis function
camera_flag = True

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
    

def main():   
    rospy.init_node('dmp_moveit_test_node',anonymous=True)    
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory,
        queue_size= 20
    )
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
    rospy.loginfo("initialization is finished")
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(plan_arm)
    
    dmp_weight,demo_start_pose,demo_end_pose = get_parameter_w(limb_name=limb_name,data_path=data_path,train_len=train_len)
    
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
    plan_to_start = group.plan()
    print "============ Waiting while RVIZ displays plan_to_start..."
    rospy.sleep(2)  
    group.go(wait=True)
    group.clear_pose_targets()
    
    end_effector_link = group.get_end_effector_link()
    print("end_effector_link",end_effector_link)
    current_pose = group.get_current_pose(end_effector_link).pose
    current_pose = [current_pose.position.x,current_pose.position.y,current_pose.position.z,
                    current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,
                    current_pose.orientation.w]
    #target_pose = get_base_camera_pose(flag=camera_flag)
    target_pose = demo_end_pose
    
    dmp_traj = dmp_imitate(starting_pose=current_pose,ending_pose=target_pose,weight_mat=dmp_weight,tau=tau)
    rospy.loginfo("get dmp generalization traj")
    
    list_of_poses = []
    ipdb.set_trace()
    for idx in range(train_len):
        traj_pose = Pose()
        traj_pose.position.x = dmp_traj[idx, 0]
        traj_pose.position.y = dmp_traj[idx, 1]
        traj_pose.position.z = dmp_traj[idx, 2]
        traj_pose.orientation.x = dmp_traj[idx, 3]
        traj_pose.orientation.y = dmp_traj[idx, 4]
        traj_pose.orientation.z = dmp_traj[idx, 5]
        traj_pose.orientation.w = dmp_traj[idx, 6]
        # visualize the pose in rviz using marker
        alpha = float(idx) / train_len * 0.5 + 0.5
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


if __name__ == '__main__':
    sys.exit(main())

