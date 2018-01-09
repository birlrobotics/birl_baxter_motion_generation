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

''' Design Params '''
dir_path = os.path.dirname(os.path.realpath(__file__))
data_path_re = "/data/test1.txt"
data_path = dir_path + data_path_re
train_len = 500
tau = 100.0/train_len # set tau to 1 will give you 100 points
limb_name = "left" 




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
        

def main():  
    rospy.init_node("dmp_ability_test", anonymous=True)     

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
    
#    object_vision = get_base_camera_pose(flag = True)
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
    time_dmp = np.linspace(0, train_set.values[-1, 0], train_len) 
    dmp_data_w_time = np.column_stack((time_dmp,y_track))
    
#    ipdb.set_trace() 
        
#######################################  For plotting  
    #creat fig
    fig=plt.figure(1)
    ax = Axes3D(fig)    
    plt.xlabel('X')
    plt.ylabel('Y')
    
    #plot traj fig 
    ax.plot(postion_x,postion_y,postion_z,linewidth=2,alpha=0.3)       
    #Plot plan fig    
    ax.plot(y_track[:,0],y_track[:,1],y_track[:,2],linewidth=2,alpha=0.3)
    #Plot plan fig    
    #show the plot
    plt.draw()
    plt.show() # uncomment to plot
    return dmp_data_w_time,limb_name

   
if __name__ == '__main__':
     sys.exit(main())
