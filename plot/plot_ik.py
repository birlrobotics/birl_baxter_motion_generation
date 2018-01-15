#!/usr/bin/env python

import rospy
from trac_ik_baxter.srv import *
from trac_ik_baxter.msg import *
import baxter_interface
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
import ipdb
import os
import pandas as pd
import numpy as np
from std_msgs.msg import (
    Header,
    Empty,
)

def main():
    limb_name = "left" # change it if you use different arms
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

    
    train_len = 50    #set it 100 just for short the number to easy computer IK
    rospy.init_node("trac_ik_service", anonymous=True)         
#    limb = baxter_interface.Limb(limb_name)
    ik_request = GetConstrainedPositionIKRequest()
    ns = "trac_ik_" + limb_name
    iksvc = rospy.ServiceProxy(ns, GetConstrainedPositionIK)
    rospy.wait_for_service(ns, 5.0)
    rospy.loginfo("ready get ik solution")
    
    dir_path = os.path.dirname(os.path.realpath(__file__))
    data_path = os.path.join(dir_path, "test2.csv")
    train_set = pd.read_csv(data_path)  #using pandas read data

    # if you use left arm, the seq should be 1-7, for the right arm, 8-14
    resample_t = np.linspace(0, train_set.values[-1, 0], train_len) # resampling the time
    position_x = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 1])
    position_y = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 2])
    position_z = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 3])
    orientation_x = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 4])
    orientation_y = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 5])
    orientation_z = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 6])
    orientation_w = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 7])
#    ipdb.set_trace()
    
    ############ get curretnt joint states
        
    joint_names = [ limb_name+"_s0",
                    limb_name+"_s1",
                    limb_name+"_e0",
                    limb_name+"_e1",
                    limb_name+"_w0",
                    limb_name+"_w1",
                    limb_name+"_w2"
                  ]
    
    
    #################
    req_joint_state = JointState()          
    req_joint_state.name = joint_names
    
    # populate the request message
    
    for i in range(train_len):
        marker_pose = PoseStamped()
        marker_pose.header.frame_id = 'base'
        marker_pose.header.stamp = rospy.Time.now()
        marker_pose.header.seq = i
        
        marker_pose.pose.position.x = float(position_x[i])
        marker_pose.pose.position.y = float(position_y[i])
        marker_pose.pose.position.z = float(position_z[i]) 
        marker_pose.pose.orientation.x = float(orientation_x[i])
        marker_pose.pose.orientation.y = float(orientation_y[i])
        marker_pose.pose.orientation.z = float(orientation_z[i])
        marker_pose.pose.orientation.w = float(orientation_w[i])

        ik_request.pose_stamp.append(marker_pose)
        ik_request.seed_angles.append(req_joint_state)
        req_joint_state.position = home_angles
        ik_request.num_steps = 100
        ik_request.end_tolerance = 0.01
    resp = iksvc(ik_request)
    ipdb.set_trace()
    print resp
#    if resp.isValid[0] != False:
#        print("the solution is valid")
#        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
#        print("IK Joint Solution:\n{0}".format(limb_joints))
#        print("------------------")
#        return limb_joints
#    else:
#        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
#        return False
    
    
    #print resp
    #ipdb.set_trace()
    #joint_command = dict(zip(resp.joints[0].name, resp.joints[0].position))
    #return joint_command
        
    
#    traj = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0]]* train_len
#    for i in range(train_len):
#        traj[i] = [position_x[i],position_y[i],position_z[i],orientation_x[i],orientation_y[i],orientation_z[i],orientation_w[i]]
    
    
#    resp1 = trac_ik_solver.get_ik(marker_pose)
    #print resp1



if __name__ == '__main__':
    sys.exit(main())