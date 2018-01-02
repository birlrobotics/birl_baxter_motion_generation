#!/usr/bin/env python  
import rospy
import math
import tf
from arm_move import srv_action_client
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
import baxter_interface
from baxter_interface import CHECK_VERSION
import copy
from optparse import OptionParser
import util

import sys,os
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import pandas as pd
from birl_baxter_dmp.dmp_train import train
from birl_baxter_dmp.dmp_generalize import dmp_imitate
from trac_ik_baxter.srv import *
import pydmps
import pydmps.dmp_discrete

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "test1.txt")
dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "test1.txt")
train_len = 100
tau = 100.0/train_len # set tau to 1 will give you 100 points
limb_name = "left"

def move_to_start_pose(start_pose,limb_name):
    if limb_name == "left":
        name = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
        position = start_pose

    if limb_name == "right":
        name = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
        position = start_pose


    limb = baxter_interface.Limb(limb_name)

    
    d = dict(zip(name, position))

    traj = srv_action_client.Trajectory(limb_name)
    traj.clear(limb_name)
    traj.stop()
    traj.add_point([d[k] for k in limb.joint_names()], 4)
    traj.start()
    rospy.loginfo("stabalize baxter")
    traj.wait(1)
    rospy.sleep(5)


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


def main():
    global data_path, train_len, tau, limb_name

    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('grab_object_moveit')

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("left_arm")

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory,
        queue_size= 20
    )

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)


    # limb = baxter_interface.Limb(limb_name)
    train_set = pd.read_csv(data_path)  # using pandas read data
    resample_t = np.linspace(train_set.values[0, 0], train_set.values[-1, 0], train_len)  # resampling the time

    postion_x = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 1])
    postion_y = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 2])
    postion_z = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 3])
    orientation_x = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 4])
    orientation_y = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 5])
    orientation_z = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 6])
    orientation_w = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 7])

    traj = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * train_len
    for i in range(train_len):
        traj[i] = [postion_x[i], postion_y[i], postion_z[i], orientation_x[i], orientation_y[i], orientation_z[i],
                   orientation_w[i]]

    train_set_T = np.array(
        [np.array([postion_x, postion_y, postion_z, orientation_x, orientation_y, orientation_z, orientation_w]).T])
    param_w, base_function = train(train_set_T)
    # object_vision = get_base_camera_pose(flag=True)
    # end_pose = limb.endpoint_pose()
    # get current angles
    # endpoint_pose = [end_pose['position'].x,
    #                  end_pose['position'].y,
    #                  end_pose['position'].z,
    #                  end_pose['orientation'].x,
    #                  end_pose['orientation'].y,
    #                  end_pose['orientation'].z,
    #                  end_pose['orientation'].w]

    start_point = traj[0]
    #    start_point = endpoint_pose
    #    print start_point
    #    ending_point = object_vision
    ending_point = traj[-1]
    #    print ending_point
    dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=7, n_bfs=500, w=param_w)
    for i in range(7):
        dmp.y0[i] = start_point[i]  # set the initial state
        dmp.goal[i] = ending_point[i]  # set the ending goal

    y_track, dy_track, ddy_track = dmp.rollout(tau=tau)

#    rospy.loginfo("move_to_start_pose... ")
#    move_to_start_pose(start_pose = y_track[0],limb_name = limb_name)
#    rospy.loginfo("Done... ")

    
#    step_length = 0.01
    list_of_poses = []
#    list_of_poses.append(group.get_current_pose().pose)
    for idx in range(train_len):
        traj_pose = Pose()
        traj_pose.position.x = y_track[idx, 0]
        traj_pose.position.y = y_track[idx, 1]
        traj_pose.position.z = y_track[idx, 2]
        traj_pose.orientation.x = y_track[idx, 3]
        traj_pose.orientation.y = y_track[idx, 4]
        traj_pose.orientation.z = y_track[idx, 5]
        traj_pose.orientation.w = y_track[idx, 6]
        # visualize the pose in rviz using marker
        alpha = float(idx) / train_len * 0.5 + 0.5
        rgba_tuple = ((0.5 * idx), 0.5, (0.5 * idx), alpha)
        util.send_traj_point_marker(marker_pub=marker_pub, pose=traj_pose, id=idx, rgba_tuple=rgba_tuple)
        rospy.loginfo("add one pose")
        list_of_poses.append(copy.deepcopy(traj_pose))
    
    rospy.loginfo("============ Generating plan")
    group.set_max_acceleration_scaling_factor(0.01)
    group.set_max_velocity_scaling_factor(0.01)
    (plan, fraction) = group.compute_cartesian_path(
        list_of_poses,  # waypoints to follow
        0.01,  # eef_step
        0.0)  # jump_threshold
    print "============ Waiting while RVIZ displays plan.."
    
#    group.plan()
#    ipdb.set_trace()
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    rospy.loginfo("============ Visulaize plan")
    display_trajectory_publisher.publish(display_trajectory)
    rospy.sleep(10)
#    ipdb.set_trace()

    rospy.loginfo("============ Enter \"ok\" to execute plan")
    s = raw_input()
    if s == 'ok':
        rospy.loginfo("============ Gonna execute plan")
    group.execute(plan)
    rospy.loginfo("============ Done")
#    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())

