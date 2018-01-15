#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import struct
import sys
import copy

import rospy
import rospkg

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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
from trac_ik_baxter.srv import *
from sensor_msgs.msg import JointState
import ipdb
import copy

import pandas as pd

import numpy as np
from birl_baxter_dmp import dmp_generalize
from birl_baxter_dmp import dmp_train
from birl_baxter_dmp import dmp_train_data_from_baxter
import pydmps

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        self._flag = False
        #ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        #self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        #rospy.wait_for_service(ns, 5.0)

        ns = "trac_ik_" + self._limb_name
        self._trac_iksvc = rospy.ServiceProxy(ns, GetConstrainedPositionIK)
        rospy.wait_for_service(ns, 5.0)
        rospy.loginfo("Ik solver is ready ")

        self._base_req = get_poseRequest()
        self._r_hand_req = get_poseRequest()
        self._base_pose_client = rospy.ServiceProxy('get_pose_from_base_camera', get_pose)
        self._r_hand_pose_client = rospy.ServiceProxy('get_pose_from_r_hand_camera', get_pose)
        rospy.wait_for_service('get_pose_from_base_camera')
         
        rospy.loginfo("Geting pose service is ready ")

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
    
    def get_base_camera_pose(self):
        rospy.loginfo("getting pose from base camera")
        self._base_req.flag = self._flag
        resp = self._base_pose_client(self._base_req)
        return resp.pose
    
    def get_r_camera_pose(self):
        rospy.loginfo("getting pose from right camera ")
        self._r_hand_req = self._flag
        resp = self._r_hand_pose_client(self._r_hand_req)
        return resp.pose


    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")
        
    def trac_ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ik_req = GetConstrainedPositionIKRequest()
        ik_req.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        current_joint_angles = self._limb.joint_angles()
        # get current angles
        joint_names = [ self._limb_name+"_s0",
                        self._limb_name+"_s1",
                        self._limb_name+"_e0",
                        self._limb_name+"_e1",
                        self._limb_name+"_w0",
                        self._limb_name+"_w1",
                        self._limb_name+"_w2"
                      ]
        
        current_joint_states = [
            current_joint_angles[self._limb_name+"_s0"],
            current_joint_angles[self._limb_name+"_s1"],
            current_joint_angles[self._limb_name+"_e0"],
            current_joint_angles[self._limb_name+"_e1"],
            current_joint_angles[self._limb_name+"_w0"],
            current_joint_angles[self._limb_name+"_w1"],
            current_joint_angles[self._limb_name+"_w2"]
                                ]
        
        req_joint_state = JointState()         
        req_joint_state.name = joint_names
        req_joint_state.position = current_joint_states
        # populate the request message
        ik_req.seed_angles.append(req_joint_state)  
        ik_req.num_steps = 1
        #ik_req.end_tolerance = 0.0001
        
        try:
            resp = self._trac_iksvc(ik_req)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        if resp.isValid[0] != False:
            print("the solution is valid")
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print("IK Joint Solution:\n{0}".format(limb_joints))
            print("------------------")
            return limb_joints
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False


    def ik_request(self, pose):
        ipdb.set_trace()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.trac_ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x 
        ik_pose.position.y = current_pose['position'].y 
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z 
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.trac_ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.trac_ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self):
        self._flag = True
        #ipdb.set_trace()
        hover_pose = self.get_base_camera_pose()
        #static_gripper_pose = copy.copy(pose)
        # open the gripper
        self._approach(hover_pose)
        rospy.loginfo("get to hover position")
        rospy.sleep(1.0)
        self.gripper_open()
        pick_pose = self.get_base_camera_pose()  
        self._servo_to_pose(pick_pose)
        rospy.loginfo("get to pick position")
        # close gripper
        #self.gripper_close()
        # retract to clear object
        #self._retract()

def make_command(line):
    """
    Cleans a single line of recorded joint positions

    @param line: the line described in a list to process
    @param names: joint name keys
    """
    
    joint_cmd_names = [
            'right_s0',
            'right_s1',
            'right_e0',
            'right_e1',
            'right_w0',
            'right_w1',
            'right_w2',
        ]
    data_line =[line[0][0],line[0][1],line[0][2],line[0][3],line[0][4],line[0][5],line[0][6]]
    command = dict(zip(joint_cmd_names, data_line))
    return command

def get_goal_from_marker_pose():
    pnp._flag = True
    marker_pose = pnp.get_base_camera_pose()
    joint_state = pnp.trac_ik_request(marker_pose)
    ending_point = [ joint_state['right_s0'] ,
        joint_state['right_s1'],
        joint_state['right_e0'],
        joint_state['right_e1'],
        joint_state['right_w0'],
        joint_state['right_w1'],
        joint_state['right_w2']
      ]
    return ending_point
   

def main():
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("pick_and_place_dmp_demo")

    limb = 'right'
    
    hover_distance = 0.15 # meters
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_w0': -0.19059711289476267,
                             'right_w1': 1.6682041068253874,
                             'right_w2': 0.2116893487281871, 
                             'right_e0': 0.6300826086239337,
                             'right_e1': 1.0082088728376881,
                             'right_s0': 0.20363594959178868, 
                             'right_s1': -1.3173060015965992}
        
    pnp = PickAndPlace(limb, hover_distance)
    # Move to the desired starting angles
    rospy.loginfo("Moving to starting joint angle")
    ipdb.set_trace()
    pnp.move_to_start(starting_joint_angles)
    
    current_joint_angle = pnp._limb.joint_angles()
    
    start_point  = [
    current_joint_angle[pnp._limb_name+"_s0"],
    current_joint_angle[pnp._limb_name+"_s1"],
    current_joint_angle[pnp._limb_name+"_e0"],
    current_joint_angle[pnp._limb_name+"_e1"],
    current_joint_angle[pnp._limb_name+"_w0"],
    current_joint_angle[pnp._limb_name+"_w1"],
    current_joint_angle[pnp._limb_name+"_w2"]
                        ]
    
    weight = dmp_train_data_from_baxter.main()
    #ipdb.set_trace()
    dmp = pydmps.dmp_discrete.DMPs_discrete(dmps=7, bfs=100, w=weight)
    y_track, dy_track, ddy_track = dmp.rollout(tau=1)
    rcmd_start = make_command(y_track)
    pnp._guarded_move_to_joint_position(rcmd_start)
    print("move to dmp start")
    time_dmp = np.linspace(0.2,6,100)
    y_track = []
    right = baxter_interface.Limb('right')
    start_time = rospy.get_time()
    for t in range(dmp.timesteps):
        y, _, _ = dmp.step()
        y_track.append(np.copy(y))
        rcmd = make_command(y_track)  
        while (rospy.get_time() - start_time) < time_dmp[t]:
            right.set_joint_positions(rcmd)
        y_track = []       
        dmp.goal = get_goal_from_marker_pose()
    
    
    
    

if __name__ == '__main__':
    sys.exit(main())
