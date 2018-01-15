#!/usr/bin/python
import numpy as np
import PyKDL
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
import dmp_ik_test
import sys,ipdb

def main():
#    ipdb.set_trace()
    data = dmp_ik_test.main()
    train_len = len(data)
    end_frame = PyKDL.Frame()
    end_frame_dmp = PyKDL.Frame()
    baxter = URDF.from_parameter_server(key='robot_description')
    base_link = baxter.get_root()
    limb = "left"
    tip_link = limb + '_gripper'
    kdl_tree = kdl_tree_from_urdf_model(baxter)
    arm_chain = kdl_tree.getChain(base_link,tip_link)
    fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(arm_chain)
    Jnt_list = PyKDL.JntArray(7)
    Jnt_list_dmp = PyKDL.JntArray(7)
    pose = []
    pose_dmp = []
    for i in range(train_len):
        Jnt_list[0] = data[0][i]
        Jnt_list[1] = data[1][i]
        Jnt_list[2] = data[2][i]
        Jnt_list[3] = data[3][i]
        Jnt_list[4] = data[4][i]
        Jnt_list[5] = data[5][i]
        Jnt_list[6] = data[6][i]
        fk_p_kdl.JntToCart(Jnt_list,end_frame)
        pos = end_frame.p
        rot = PyKDL.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        pose.append(np.array([pos[0], pos[1], pos[2],
                             rot[0], rot[1], rot[2], rot[3]]))
    
    xs = []
    ys = []
    zs = []
    for j in range(train_len):
        position_x = pose[j][0]
        position_y = pose[j][1]
        position_z = pose[j][2]
        xs.append(position_x)
        ys.append(position_y)
        zs.append(position_z)
    
    
    fig=plt.figure(2)
    ax = Axes3D(fig)
    plt.xlabel('X')
    plt.ylabel('Y')
    # plot traj fig
    ax.plot(xs, ys, zs, linewidth=4, alpha=0.3)
    # Plot plan fig
    # Plot plan fig
    
    # show the plot
    plt.draw()
    plt.show()
if __name__ == '__main__':
     sys.exit(main())