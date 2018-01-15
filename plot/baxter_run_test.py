#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
Created on Mon Nov  6 16:48:02 2017

@author: tony

"""
import rospy
import pandas as pd
import numpy as np
from birl_baxter_dmp.dmp_train import train
from birl_baxter_dmp.dmp_generalize import dmp_imitate
import pydmps
import pydmps.dmp_discrete
import os,sys
from trac_ik_baxter.srv import *
import baxter_interface
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import ipdb, copy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)
dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "test1.txt")
train_len = 500
tau = 100.0/train_len # set tau to 1 will give you 100 points
limb_name = "left" 

''' Design Params '''
#n=5 # Factor for tau and num waypoints


def clean_line(line, names):
    """
    Cleans a single line of recorded joint positions

    @param line: the line described in a list to process
    @param names: joint name keys
    """
    #convert the line of strings to a float or None
    #line = [try_float(x) for x in line.rstrip().split(' ')]
    #zip the values with the joint names
    combined = zip(names[0:], line[0:])
    #take out any tuples that have a none value
    cleaned = [x for x in combined if x[1] is not None]
    #convert it to a dictionary with only valid commands
    command = dict(cleaned)
    left_command = dict((key, command[key]) for key in command.keys()
                         if key[:-2] == 'left')
    return left_command

def get_base_camera_pose(flag):      
        base_req = get_poseRequest()
        base_pose_client = rospy.ServiceProxy('get_pose_from_base_camera', get_pose, persistent=True)
        rospy.wait_for_service('get_pose_from_base_camera')
        rospy.loginfo("Geting pose service is ready")
        base_req.flag = flag
        marker_pose = base_pose_client(base_req)
        return [marker_pose.pose.position.x, marker_pose.pose.position.y,marker_pose.pose.position.z,
                marker_pose.pose.orientation.x,marker_pose.pose.orientation.y,
                marker_pose.pose.orientation.z,marker_pose.pose.orientation.w]
        

def trac_ik_request(pose_sets, limb_name ):
        global train_len 
        dim = pose_sets.shape[1]
        if limb_name == "left": 
            home_angles = [-0.978679742670894,
                        -1.0089758632316308,
                        0.07094661143970038,
                        1.650179832567734,
                        -0.12156797743991904,
                        0.8095583608065271,
                        0.055223308363874894]
        
        if limb_name == "right":    
            home_angles = [ 0.6615292147755847,
                        -0.8828059434280556,
                         0.26691265709206197,
                         1.3449176557785365,
                         -0.15838351634916897,
                         1.0032234352770606,
                        -0.028378644575880154]
            
        ns = "trac_ik_" + limb_name
        ik_req = GetConstrainedPositionIKRequest()
        req_joint_state = JointState() 
        joint_names = [ limb_name+"_s0",
                    limb_name+"_s1",
                    limb_name+"_e0",
                    limb_name+"_e1",
                    limb_name+"_w0",
                    limb_name+"_w1",
                    limb_name+"_w2"
                  ]   
                  
        req_joint_state.name = joint_names
        trac_iksvc = rospy.ServiceProxy(ns, GetConstrainedPositionIK,persistent=True)
        rospy.wait_for_service(ns, 5.0)
        
        for i in range(train_len):            
            cart_pose = PoseStamped()
            cart_pose.header.frame_id = 'base'
            cart_pose.header.stamp = rospy.Time.now()
            cart_pose.header.seq = i
            
            req_joint_state.position = home_angles
            ik_req.seed_angles.append(req_joint_state)
            ik_req.num_steps = 100
            ik_req.end_tolerance = 0.01     
            
            cart_pose.pose.position.x = float(pose_sets[i][0])
            cart_pose.pose.position.y = float(pose_sets[i][1])
            cart_pose.pose.position.z = float(pose_sets[i][2]) 
            cart_pose.pose.orientation.x = float(pose_sets[i][3])
            cart_pose.pose.orientation.y = float(pose_sets[i][4])
            cart_pose.pose.orientation.z = float(pose_sets[i][5])
            cart_pose.pose.orientation.w = float(pose_sets[i][6])       
            ik_req.pose_stamp.append(cart_pose)                              
                 
        try:
            resp = trac_iksvc(ik_req)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        joint_state_list = []
        for i in range(train_len):
            if resp.isValid[i] != False:
                limb_joints = dict(zip(resp.joints[i].name, resp.joints[i].position))
                joint_state_list.append(limb_joints)
        rospy.loginfo("get ik done")
        return joint_state_list

def main():  
    rospy.init_node("trac_ik_dmp_cart_node", anonymous=True)     
    # verify robot is enabled
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()  
    

    # this file is recoded by baxter 
    # run this command to record data "rosrun baxter_examples joint_recorder.py -f path/file_name
    global data_path,train_len,tau,limb_name
    limb = baxter_interface.Limb(limb_name)
    train_set = pd.read_csv(data_path)  #using pandas read data
    resample_t = np.linspace(train_set.values[0, 0], train_set.values[-1, 0], train_len) # resampling the time

    postion_x = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 1])
    postion_y = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 2])
    postion_z = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 3])
    orientation_x = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 4])
    orientation_y = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 5])
    orientation_z = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 6])
    orientation_w = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 7])

    traj = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0]]* train_len
    for i in range(train_len):
        traj[i] = [postion_x[i],postion_y[i],postion_z[i],orientation_x[i],orientation_y[i],orientation_z[i],orientation_w[i]]
    
    train_set_T = np.array([np.array([postion_x,postion_y,postion_z,orientation_x,orientation_y,orientation_z,orientation_w]).T])
    param_w, base_function = train(train_set_T)
    
    object_vision = get_base_camera_pose(flag = True)
    end_pose = limb.endpoint_pose() 
    # get current angles
    endpoint_pose = [      end_pose['position'].x,
                           end_pose['position'].y,
                           end_pose['position'].z,
                           end_pose['orientation'].x,                                            
                           end_pose['orientation'].y,
                           end_pose['orientation'].z,
                           end_pose['orientation'].w ]  
#    ipdb.set_trace()
    start_point = traj[0]
#    start_point = endpoint_pose  
#    print start_point
#    ending_point = object_vision
    ending_point = traj[-1]
#    print ending_point
    dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=7, n_bfs=500, w=param_w)
    for i in range(7):
        dmp.y0[i] = start_point[i]  #set the initial state
        dmp.goal[i] = ending_point[i] # set the ending goal

    y_track, dy_track, ddy_track = dmp.rollout(tau=tau)# ?
    
#    diff = []
#    abs_len = np.sqrt((np.square(y_track[-1,0] -y_track[0,0])+np.square(y_track[-1,1] -y_track[0,1])+np.square(y_track[-1,2] -y_track[0,2])))   
#    for i in range(1,train_len):
#        dif = np.sqrt((np.square(y_track[i,0] -y_track[i-1,0])+np.square(y_track[i,1] -y_track[i-1,1])+np.square(y_track[i,2] -y_track[i-1,2])))        
#        diff.append(dif)
#    diff_np = np.array(diff)
#    diff_sum = diff_np.sum()
#    ipdb.set_trace()
#    num_point = diff_sum/0.001
#    ipdb.set_trace()
#    ipdb.set_trace()
    time_dmp = np.linspace(0, train_set.values[-1, 0], train_len) 
#    ipdb.set_trace()
    cmd = trac_ik_request(pose_sets=y_track, limb_name=limb_name)  
    print("moving to start position")
    limb.move_to_joint_positions(cmd[0])
    print("Done")
    rate = rospy.Rate(100)
    
    for idx in range(train_len):
        if rospy.is_shutdown():
            print("\n Aborting - ROS shutdown")
            return False
#        limb.set_joint_positions(cmd[idx])
        rate.sleep()
#    start_time = rospy.get_time()
#    for idx in range(train_len):
#        while (rospy.get_time() - start_time) < time_dmp[idx]:       
#            limb.set_joint_positions(cmd[idx])
#            if rospy.is_shutdown():
#                print("\n Aborting - ROS shutdown")
#                return False            
#            rate.sleep()
#    lines = np.column_stack((time_dmp,y_track))
#    keys = ['times','right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
#    keys = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']  
#    rate = rospy.Rate(1000)
#    pose = Pose()
#    for idx in range(1):
#        pose.position.x = y_track[idx,0]
#        pose.position.y = y_track[idx,1]
#        pose.position.z = y_track[idx,2]
#        pose.orientation.x = y_track[idx,3]
#        pose.orientation.y = y_track[idx,4]
#        pose.orientation.z = y_track[idx,5]
#        pose.orientation.w = y_track[idx,6]
#        lcmd_start= trac_ik_request(pose=pose)   
#    
#    
#    
#
#    start_time = rospy.get_time()
##    ipdb.set_trace()
#    for idx in range(1,len(time_dmp)):
#        pose.position.x = y_track[idx,0]
#        pose.position.y = y_track[idx,1]
#        pose.position.z = y_track[idx,2]
#        pose.orientation.x = y_track[idx,3]
#        pose.orientation.y = y_track[idx,4]
#        pose.orientation.z = y_track[idx,5]
#        pose.orientation.w = y_track[idx,6]
#        lcmd = trac_ik_request(pose=pose)                  
#        #command this set of commands until the next frame
#        print ((rospy.get_time() - start_time) < time_dmp[idx])
#        while (rospy.get_time() - start_time) < time_dmp[idx]:
#            limb.set_joint_positions(cmd[i])
#            if rospy.is_shutdown():
#                print("\n Aborting - ROS shutdown")
#                return False            
#            rate.sleep()
#    
#   {'right_s0': 1.3483103233007299, 'right_s1': -1.1026646285700645, 
#   'right_w0': 0.1503306903853853, 'right_w1': 1.5856406978589865, 'right_w2': 0.2999005810979858, 'right_e0': -0.34786733022148175, 'right_e1': 1.0831079510624184 
#1.3475303474795948, -1.100049758731219, -0.3475166430296643, 1.0789709540159511, 0.15063928197300847, 1.5889827180212805, 0.30286672811813503
#    y_track = []
#    start_time = rospy.get_time()
#    for t in range(dmp.timesteps):
#        y, _, _ = dmp.step()
#        y_track.append(np.copy(y))
#        rcmd = make_command(y_track)  
#        while (rospy.get_time() - start_time) < time_dmp[t]:
#            limb.set_joint_positions(rcmd)
#        y_track = []
#        dmp.goal = ending_point_vision
#        print dmp.goal
        
#######################################  For plotting  
    #creat fig
   
if __name__ == '__main__':
     sys.exit(main())
