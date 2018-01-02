#!/usr/bin/env python

# license removed for brevity
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from trac_ik_baxter.msg import *
import baxter_get_ik_srv as ik_solver
import copy
import ipdb
from geometry_msgs.msg import PoseStamped

'''
IK Solver.
Currently follows hard-coded structure. 
Uses Trac-IK numerical solver. Set seed to current pose.

'''

goal=[]


def callback(data):
    #ipdb.set_trace()
    print('Getting robots joint angle data')
    global goal
    goal = [data.position[0],
            data.position[1],
            data.position[2],
            data.position[3],
            data.position[4],
            data.position[5],
            data.position[6]]
def main():
    print("Initializing node..")
    rospy.init_node("test_accuracy_node")
    
    print("Getting robot state... ")    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    print("Enabling robot... ")
    rs.enable()
    
    right = baxter_interface.Gripper('right', CHECK_VERSION)
    
    rospy.Subscriber("marker_pose", joint_q, callback)
    right = baxter_interface.Limb('right')
    global goal
    joint_cmd_names = [
            'right_s0',
            'right_s1',
            'right_e0',
            'right_e1',
            'right_w0',
            'right_w1',
            'right_w2',
        ]
        
    hover_command = dict(zip(joint_cmd_names, goal))  
    print("moving to hover position: ")
    right.move_to_joint_positions(hover_command)
    #ipdb.set_trace()
    
    hover_pose = right.endpoint_pose()
    pick_pose = copy.copy(hover_pose)
    pick_pose_position = pick_pose['position']
    pick_pose_orientation = pick_pose['orientation']
   
    pick_pose = PoseStamped()
    pick_pose.pose.position.x = pick_pose_position[0]
    pick_pose.pose.position.y = pick_pose_position[1]
    pick_pose.pose.position.z = pick_pose_position[2] + 0.1
    pick_pose.pose.orientation.x = pick_pose_orientation[0]
    pick_pose.pose.orientation.y = pick_pose_orientation[1]
    pick_pose.pose.orientation.z = pick_pose_orientation[2]
    pick_pose.pose.orientation.w = pick_pose_orientation[3]

    ' Returns Joint Solutions'
    pick_joint= ik_solver.get_ik(pick_pose)
   
    pick_command = [pick_joint.joints[0].position[0], pick_joint.joints[0].position[1],pick_joint.joints[0].position[2],
    pick_joint.joints[0].position[3], pick_joint.joints[0].position[4], pick_joint.joints[0].position[5], pick_joint.joints[0].position[6]]
    
    pick_command = dict(zip(joint_cmd_names, pick_command)) 
    print("moving to pick position")
    right.move_to_joint_positions(pick_command)


if __name__ == '__main__':
    sys.exit(main())
