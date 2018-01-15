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
#    print("Getting robot state... ")
#    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
#    init_state = rs.state().enabled
#    print("Enabling robot... ")
#    rs.enable()  
    

    # this file is recoded by baxter 
    # run this command to record data "rosrun baxter_examples joint_recorder.py -f path/file_name
    global data_path,train_len,tau,limb_name
#    limb = baxter_interface.Limb(limb_name)
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
      
    #object_vision = get_base_camera_pose(flag = True)
#    end_pose = limb.endpoint_pose() 
#    # get current angles
#    endpoint_pose = [      end_pose['position'].x,
#                           end_pose['position'].y,
#                           end_pose['position'].z,
#                           end_pose['orientation'].x,                                            
#                           end_pose['orientation'].y,
#                           end_pose['orientation'].z,
#                           end_pose['orientation'].w ]  
#    ipdb.set_trace()
    start_point = traj[0]
#    start_point = endpoint_pose  
#    print start_point
#    ending_point = object_vision
    ending_point = traj[-1]
#    print ending_point
#    dmp_0 = pydmps.dmp_discrete.DMPs_discrete(n_dmps=7, n_bfs=500, w=param_w)

    
#    ipdb.set_trace()
    num = 10
    y0 = [0.04,0.05,0.99]
    y1 = [0.05,0.05,0.99]
    y2 = [0.06,0.05,0.99]
    y3 = [0.02,0.1,0.99]
    y4 = [0.02,0.2,0.99]
    y5 = [-0.02,-0.05,-0.99]
    y6 = [-0.02,-0.05,-1.2]
    y7 = [0.02,0.05,1.3]
    y8 = [0.1,0.05,0.99]
    y9 = [0.15,0.05,0.99]

    list_of_dmp = [None]*num
    list_of_track = [None]*num
    for i in range(num):
        list_of_dmp[i] = pydmps.dmp_discrete.DMPs_discrete(n_dmps=7, n_bfs=500, w=param_w)
        for j in range(7):
            list_of_dmp[i].y0[j] = start_point[j]  #set the initial state
            list_of_dmp[i].goal[j] = ending_point[j]+(0.01*i) # set the ending goal
            start_point[j] = list_of_dmp[i].y0[j] +0.01
            ending_point[j] = list_of_dmp[i].goal[j]
        list_of_track[i], _, _ = list_of_dmp[i].rollout(tau=tau)# ?
         



      
    fig=plt.figure()
    ax = Axes3D(fig)    
    plt.xlabel('X')
    plt.ylabel('Y')
    ax.plot(postion_x,postion_y,postion_z,linewidth=4,alpha=0.3)
    for idx in range(num):
        ax.plot(list_of_track[idx][:,0],list_of_track[idx][:,1],list_of_track[idx][:,2],"--")            
    plt.draw()
    plt.show()
    

#######################################  For plotting  
    #creat fig
#    fig=plt.figure()
#    ax = Axes3D(fig)    
#    plt.xlabel('X')
#    plt.ylabel('Y')
#    
#    #plot traj fig 
#    ax.plot(postion_x,postion_y,postion_z,linewidth=4,alpha=0.3)       
#    #Plot plan fig  
#    for j in enumerate():
#        ax.plot(y_track[:,0],y_track[:,1],y_track[:,2],"--")
#    #Plot plan fig    
#    #show the plot
#    plt.draw()
#    plt.show()

#######################################  For plotting        

#######################################  saving data to a file 
#  
#    
#    data_path = os.path.join(dir_path, "./plot/baxter_dmp_runing.txt")
#    WriteFileDir = data_path   ## the path of generated dmp traj
#    plan_len = len(y_track[:,0])
#    f = open(WriteFileDir,'w')
#    f.write('time,')
#    f.write('right_s0,')
#    f.write('right_s1,')
#    f.write('right_e0,')
#    f.write('right_e1,')
#    f.write('right_w0,')
#    f.write('right_w1,')
#    f.write('right_w2\n')
#        
#    for i in range (plan_len):
#        f.write("%f," % (time_dmp[i],))
#        f.write(str(y_track[:, 0][i])+','+str(y_track[:, 1][i])+','+str(y_track[:, 2][i])+','
#        +str(y_track[:, 3][i])+','+str(y_track[:, 4][i])+','+str(y_track[:, 5][i])+','+str(y_track[:, 6][i])
#        +'\n')        
#    f.close()
   
if __name__ == '__main__':
     sys.exit(main())
