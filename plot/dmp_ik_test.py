#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
Created on Mon Nov  6 16:48:02 2017

@author: tony

"""
import rospy
import pandas as pd
import numpy as np
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
import dmp_ability_test

    
        

def trac_ik_request(pose_sets, limb_name): 
        train_len = pose_sets.shape[0]
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
        joint_cmd_list = []
        joint_list = []
        for i in range(train_len):
            if resp.isValid[i] != False:
                limb_joints = dict(zip(resp.joints[i].name, resp.joints[i].position))
                joint_cmd_list.append(limb_joints)
                joint_list.append(resp.joints[i].position)
        rospy.loginfo("get ik done")
        return joint_cmd_list,joint_list

def main():  
    rospy.init_node("trac_ik_dmp_cart_node", anonymous=True)     
    dmp_traj,limb_name = dmp_ability_test.main()
    cmd,joint_list = trac_ik_request(pose_sets=dmp_traj[:,1:], limb_name=limb_name) 
    col = [None]*7
#    ipdb.set_trace()
    col[0] = np.array(joint_list)[:,0]
    col[1] = np.array(joint_list)[:,1]
    col[2] = np.array(joint_list)[:,2]
    col[3] = np.array(joint_list)[:,3]
    col[4] = np.array(joint_list)[:,4]
    col[5] = np.array(joint_list)[:,5]
    col[6] = np.array(joint_list)[:,6]

#######################################  For plotting         
    plt.figure(1)  
    plt.subplot(711)      
    plt.plot(dmp_traj[:,0], col[0],label='DMP imitation', lw=1)
    plt.xlabel("time")
    plt.ylabel("right_s0")
    plt.grid(True)
    plt.legend(loc='upper right')

    plt.subplot(712)
      
    plt.plot(dmp_traj[:,0], col[1],label='DMP imitation', lw=1)
    plt.ylabel("right_s1")
    plt.grid(True)

    plt.subplot(713)
      
    plt.plot(dmp_traj[:,0], col[2],label='DMP imitation', lw=1)
    plt.ylabel("right_e0")
    plt.grid(True)

    plt.subplot(714)
      
    plt.plot(dmp_traj[:,0], col[3],label='DMP imitation', lw=1)
    plt.ylabel("right_e1")
    plt.grid(True)

    plt.subplot(715)
    
    plt.plot(dmp_traj[:,0], col[4],label='DMP imitation', lw=1)
    plt.ylabel("right_w0")
    plt.grid(True)

    plt.subplot(716)
      
    plt.plot(dmp_traj[:,0], col[5],label='DMP imitation', lw=1)
    plt.ylabel("right_w1")
    plt.grid(True)

    plt.subplot(717)
      
    plt.plot(dmp_traj[:,0], col[6],label='DMP imitation', lw=1)
    plt.ylabel("right_w2")
    plt.grid(True)   
#    plt.show()
    return col
   
if __name__ == '__main__':
     sys.exit(main())
