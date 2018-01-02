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

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "./plot/horizontal.txt")

flag = True

''' Design Params '''
#n=5 # Factor for tau and num waypoints

#
#def make_command(line):
#    """
#    Cleans a single line of recorded joint positions
#
#    @param line: the line described in a list to process
#    @param names: joint name keys
#    """
#    
#    joint_cmd_names = [
#            'right_s0',
#            'right_s1',
#            'right_e0',
#            'right_e1',
#            'right_w0',
#            'right_w1',
#            'right_w2',
#        ]
#    data_line =[line[0][0],line[0][1],line[0][2],line[0][3],line[0][4],line[0][5],line[0][6]]
#    command = dict(zip(joint_cmd_names, data_line))
#    return command


def clean_line(line, names):
    """
    Cleans a single line of recorded joint positions

    @param line: the line described in a list to process
    @param names: joint name keys
    """
    #convert the line of strings to a float or None
    #line = [try_float(x) for x in line.rstrip().split(' ')]
    #zip the values with the joint names
    raw_ = line
    combined = zip(names[1:], line[1:])
    #take out any tuples that have a none value
    cleaned = [x for x in combined if x[1] is not None]
    #convert it to a dictionary with only valid commands
    command = dict(cleaned)
    right_command = dict((key, command[key]) for key in command.keys()
                         if key[:-2] == 'right_')
    return (command, right_command, raw_)



def main():
    limb_name = "right"
    rospy.init_node("trac_ik_dmp_node", anonymous=True) 
    # verify robot is enabled
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()  
    limb = baxter_interface.Limb(limb_name)
    current_joint_angles = limb.joint_angles()
    
    joint_names = [     limb_name+"_s0",
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
    #### ik service
    joint_client = rospy.ServiceProxy('get_joint_from_trac_ik', get_joint)
    rospy.wait_for_service('get_joint_from_trac_ik')
    rospy.loginfo("Geting joint service is ready ")  
    #####
    # this file is recoded by baxter 
    # run this command to record data "rosrun baxter_examples joint_recorder.py -f path/file_name
    global data_path
    train_set = pd.read_csv(data_path)  #using pandas read data
    train_len = len(train_set)  # the lengh of data
    resample_t = np.linspace(0, train_set.values[-1, 0], train_len) # resampling the time
    joint0_data = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 9])
    joint1_data = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 10])
    joint2_data = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 11])
    joint3_data = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 12])
    joint4_data = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 13])
    joint5_data = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 14])
    joint6_data = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 15])

    traj = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0]]* train_len
    for i in range(train_len):
        traj[i] = [joint0_data[i],joint1_data[i],joint2_data[i],joint3_data[i],joint4_data[i],joint5_data[i],joint6_data[i]]
    train_set = np.array([np.array([joint0_data,joint1_data,joint2_data,joint3_data,joint4_data,joint5_data,joint6_data]).T])
    param_w, base_function = train(train_set)
    
    global flag
    joint_req = get_jointRequest()
    joint_req.flag = flag
    joint_state = joint_client(joint_req)
    rospy.loginfo("raw_jointstate %s" %joint_state)
    ending_point_vision = [0,0,0,0,0,0,0]
    for i in range(7):
            ending_point_vision[i] = joint_state.joint_state.position[i]
    rospy.loginfo("ending_point_vision %s" %ending_point_vision)
    flag = False
    
    start_point = current_joint_states  
#    start_point = traj[0]  
    
    ending_point_degree = np.array(traj[-1])/np.pi*180
    ending_point_degree_ = np.array(ending_point_vision)/np.pi*180
    print("original joint states is")
    print ending_point_degree
    print "vision_based joint states"
    print ending_point_degree_
    ending_point = ending_point_vision
    
#    ending_point_hard_code = [ 0.6250971710633061, -0.8429224429430349, 0.9058156552463368, 1.3211409535663126,-0.5840631849873713, 1.3326458094754532, 0.7911505913519021]
    ending_point = traj[-1]
#    y_track = dmp_imitate(starting_pose=start_point, ending_pose=ending_point, weight_mat=param_w, tau=1, n_dmps=7 )
    dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=7, n_bfs=500, w=param_w)
    for i in range(7):
        dmp.y0[i] = start_point[i]  #set the initial state
        dmp.goal[i] = ending_point[i] # set the ending goal
    ipdb.set_trace()
    y_track, dy_track, ddy_track = dmp.rollout(tau=0.2)# ?
    time_dmp = np.linspace(0.2, 6.3, 500) #?
    lines = np.column_stack((time_dmp,y_track))
    keys = ['times','right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    _cmd, rcmd_start, _raw = clean_line(lines[0], keys)
    
    rate = rospy.Rate(1000)
    print("moving to start position")
    limb.move_to_joint_positions(rcmd_start)
    i = 0
    start_time = rospy.get_time()
    for values in (lines[0:]): 
        i += 1   
        sys.stdout.write("\r Record %d of %d" %
                             (i, len(lines) - 1))
        sys.stdout.flush()
        cmd, rcmd, values = clean_line(values, keys)
        #command this set of commands until the next frame
        while (rospy.get_time() - start_time) < values[0]:
            if rospy.is_shutdown():
                print("\n Aborting - ROS shutdown")
                return False
            if len(rcmd):
                
#                pass
                limb.set_joint_positions(rcmd)
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
    plt.plot(resample_t, joint0_data,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 0],label='DMP imitation', lw=1)
    plt.xlabel("time")
    plt.ylabel("right_s0")
    plt.grid(True)
    plt.legend(loc='upper right')

    plt.subplot(712)
    plt.plot(resample_t, joint1_data,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 1],label='DMP imitation', lw=1)
    plt.ylabel("right_s1")
    plt.grid(True)

    plt.subplot(713)
    plt.plot(resample_t, joint2_data,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 2],label='DMP imitation', lw=1)
    plt.ylabel("right_e0")
    plt.grid(True)

    plt.subplot(714)
    plt.plot(resample_t, joint3_data,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 3],label='DMP imitation', lw=1)
    plt.ylabel("right_e1")
    plt.grid(True)

    plt.subplot(715)
    plt.plot(resample_t, joint4_data,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 4],label='DMP imitation', lw=1)
    plt.ylabel("right_w0")
    plt.grid(True)

    plt.subplot(716)
    plt.plot(resample_t, joint5_data,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 5],label='DMP imitation', lw=1)
    plt.ylabel("right_w1")
    plt.grid(True)

    plt.subplot(717)
    plt.plot(resample_t, joint6_data,linewidth=1, linestyle="-", label="demestration")       
    plt.plot(time_dmp, y_track[:, 6],label='DMP imitation', lw=1)
    plt.ylabel("right_w2")
    plt.grid(True)   
    plt.show()

#######################################  For plotting        

#######################################  saving data to a file 
#  
#    
    data_path = os.path.join(dir_path, "./plot/baxter_dmp_runing.txt")
    WriteFileDir = data_path   ## the path of generated dmp traj
    plan_len = len(y_track[:,0])
    f = open(WriteFileDir,'w')
    f.write('time,')
    f.write('right_s0,')
    f.write('right_s1,')
    f.write('right_e0,')
    f.write('right_e1,')
    f.write('right_w0,')
    f.write('right_w1,')
    f.write('right_w2\n')
        
    for i in range (plan_len):
        f.write("%f," % (time_dmp[i],))
        f.write(str(y_track[:, 0][i])+','+str(y_track[:, 1][i])+','+str(y_track[:, 2][i])+','
        +str(y_track[:, 3][i])+','+str(y_track[:, 4][i])+','+str(y_track[:, 5][i])+','+str(y_track[:, 6][i])
        +'\n')        
    f.close()
   
if __name__ == '__main__':
     sys.exit(main())
