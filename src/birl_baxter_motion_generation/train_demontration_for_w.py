#!/usr/bin/env python 
import numpy as np
import pandas as pd
import os
from birl_baxter_dmp.dmp_train import train

def get_parameter_w(limb_name,data_path,train_len):
    train_set = pd.read_csv(data_path)  # using pandas read data
    resample_t = np.linspace(train_set.values[0, 0], train_set.values[-1, 0], train_len)  # resampling the time
    if limb_name == "left":
        postion_x = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 1])
        postion_y = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 2])
        postion_z = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 3])
        orientation_x = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 4])
        orientation_y = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 5])
        orientation_z = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 6])
        orientation_w = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 7])
    if limb_name == "right":
        postion_x = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 8])
        postion_y = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 9])
        postion_z = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 10])
        orientation_x = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 11])
        orientation_y = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 12])
        orientation_z = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 13])
        orientation_w = np.interp(resample_t, train_set.values[:, 0], train_set.values[:, 14])
        
    traj = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * train_len
    for i in range(train_len):
        traj[i] = [postion_x[i], postion_y[i], postion_z[i], orientation_x[i], orientation_y[i], orientation_z[i],
                   orientation_w[i]]
    start_of_demo = traj[0]
    end_of_demo = traj[-1]
    train_set_T = np.array(
        [np.array([postion_x, postion_y, postion_z, orientation_x, orientation_y, orientation_z, orientation_w]).T])
    dmp_w, base_function = train(train_set_T)
    return dmp_w,start_of_demo,end_of_demo 