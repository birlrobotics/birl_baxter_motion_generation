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
        

def trac_ik_request(pose):
        limb_name = "left"
        limb = baxter_interface.Limb(limb_name)
        ns = "trac_ik_left"
        trac_iksvc = rospy.ServiceProxy(ns, GetConstrainedPositionIK,persistent=True)
        rospy.wait_for_service(ns, 5.0)
#            rospy.loginfo("Ik solver is ready ")

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ik_req = GetConstrainedPositionIKRequest()
        ik_req.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        current_joint_angles = limb.joint_angles()
        # get current angles
        joint_names = [ limb_name+"_s0",
                        limb_name+"_s1",
                        limb_name+"_e0",
                        limb_name+"_e1",
                        limb_name+"_w0",
                        limb_name+"_w1",
                        limb_name+"_w2"
                      ]
        
        current_joint_states = [
            current_joint_angles[limb_name+"_s0"],
            current_joint_angles[limb_name+"_s1"],
            current_joint_angles[limb_name+"_e0"],
            current_joint_angles[limb_name+"_e1"],
            current_joint_angles[limb_name+"_w0"],
            current_joint_angles[limb_name+"_w1"],
            current_joint_angles[limb_name+"_w2"]
                                ]
        
        req_joint_state = JointState()         
        req_joint_state.name = joint_names
        req_joint_state.position = current_joint_states
        # populate the request message
        ik_req.seed_angles.append(req_joint_state)  
        ik_req.num_steps = 10
        ik_req.end_tolerance = 0.001       
        try:
            resp = trac_iksvc(ik_req)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        if resp.isValid[0] != False:
#            print("the solution is valid")
            #ipdb.set_trace()
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
#            print("IK Joint Solution:\n{0}".format(limb_joints))
#            print("------   ------------")
            return limb_joints
        else:

            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

def main():
    limb_name = "left"   
    rospy.init_node("trac_ik_dmp_node", anonymous=True) 
    limb = baxter_interface.Limb(limb_name)
    # verify robot is enabled
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()  
    

    # this file is recoded by baxter 
    # run this command to record data "rosrun baxter_examples joint_recorder.py -f path/file_name
    global data_path
    train_set = pd.read_csv(data_path)  #using pandas read data
    train_len = len(train_set)  # scaling factor
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
    current_joint_angles = limb.joint_angles()
    # get current angles
    joint_names = [ limb_name+"_s0",
                    limb_name+"_s1",
                    limb_name+"_e0",
                    limb_name+"_e1",
                    limb_name+"_w0",
                    limb_name+"_w1",
                    limb_name+"_w2"
                  ]
    
    current_joint_states = [
        current_joint_angles[limb_name+"_s0"],
        current_joint_angles[limb_name+"_s1"],
        current_joint_angles[limb_name+"_e0"],
        current_joint_angles[limb_name+"_e1"],
        current_joint_angles[limb_name+"_w0"],
        current_joint_angles[limb_name+"_w1"],
        current_joint_angles[limb_name+"_w2"]
                            ]
    
#    ipdb.set_trace()
    start_point = traj[0]
#    start_point = current_joint_states 
    print start_point
#    ending_point = traj[-1]
    ending_point = object_vision
    print ending_point
    dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=7, n_bfs=500, w=param_w)
    for i in range(7):
        dmp.y0[i] = start_point[i]  #set the initial state
        dmp.goal[i] = ending_point[i] # set the ending goal

    y_track, dy_track, ddy_track = dmp.rollout(tau=1)# ?
#    ipdb.set_trace()
    time_dmp = np.linspace(0, train_set.values[-1, 0], 100) #?
#    lines = np.column_stack((time_dmp,y_track))
#    keys = ['times','right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    keys = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']  
    rate = rospy.Rate(1000)
    pose = Pose()
    for idx in range(1):
        pose.position.x = y_track[idx,0]
        pose.position.y = y_track[idx,1]
        pose.position.z = y_track[idx,2]
        pose.orientation.x = y_track[idx,3]
        pose.orientation.y = y_track[idx,4]
        pose.orientation.z = y_track[idx,5]
        pose.orientation.w = y_track[idx,6]
        lcmd_start= trac_ik_request(pose=pose)   
    print("moving to start position")
    limb.move_to_joint_positions(lcmd_start)
    print("Done")

    start_time = rospy.get_time()
#    ipdb.set_trace()
    for idx in range(1,len(time_dmp)):
        pose.position.x = y_track[idx,0]
        pose.position.y = y_track[idx,1]
        pose.position.z = y_track[idx,2]
        pose.orientation.x = y_track[idx,3]
        pose.orientation.y = y_track[idx,4]
        pose.orientation.z = y_track[idx,5]
        pose.orientation.w = y_track[idx,6]
        lcmd = trac_ik_request(pose=pose)                  
        #command this set of commands until the next frame
        print ((rospy.get_time() - start_time) < time_dmp[idx])
        while (rospy.get_time() - start_time) < time_dmp[idx]:
            limb.set_joint_positions(lcmd)
            if rospy.is_shutdown():
                print("\n Aborting - ROS shutdown")
                return False            
            rate.sleep()
    
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
#    fig=plt.figure()
#    ax = Axes3D(fig)    
#    plt.xlabel('X')
#    plt.ylabel('Y')
#    
#    #plot traj fig 
#    ax.plot(Colum0_Traj,Colum1_Traj,Colum2_Traj,linewidth=4,alpha=0.3)       
#    #Plot plan fig    
#    ax.plot(y_track[:,0],y_track[:,1],y_track[:,2],"--")
#    #Plot plan fig    
#    #show the plot
#    plt.draw()
#    plt.show()
    plt.figure(1)  
    plt.subplot(711)
    plt.plot(resample_t, postion_x,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 0],label='DMP imitation', lw=1)
    plt.xlabel("time")
    plt.ylabel("right_s0")
    plt.grid(True)
    plt.legend(loc='upper right')

    plt.subplot(712)
    plt.plot(resample_t, postion_y,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 1],label='DMP imitation', lw=1)
    plt.ylabel("right_s1")
    plt.grid(True)

    plt.subplot(713)
    plt.plot(resample_t, postion_z,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 2],label='DMP imitation', lw=1)
    plt.ylabel("right_e0")
    plt.grid(True)

    plt.subplot(714)
    plt.plot(resample_t, orientation_x,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 3],label='DMP imitation', lw=1)
    plt.ylabel("right_e1")
    plt.grid(True)

    plt.subplot(715)
    plt.plot(resample_t, orientation_y,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 4],label='DMP imitation', lw=1)
    plt.ylabel("right_w0")
    plt.grid(True)

    plt.subplot(716)
    plt.plot(resample_t, orientation_z,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 5],label='DMP imitation', lw=1)
    plt.ylabel("right_w1")
    plt.grid(True)

    plt.subplot(717)
    plt.plot(resample_t, orientation_w,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 6],label='DMP imitation', lw=1)
    plt.ylabel("right_w2")
    plt.grid(True)   
    plt.show()

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
