#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 27 19:58:04 2021

@author: abdullahk047
"""

import pandas as pd
from os.path import expanduser as ospath
import os
import math
import matplotlib.pyplot as plt
import numpy as np
import random

from pathlib import Path



def closest_waypoint(pos, waypoints):
    waypoints = np.asarray(waypoints)
    distances = np.sum((waypoints - pos)**2, axis=1)
    return np.argmin(distances)

def cte(pos, vel_vect, wp_idx, k, waypoints):
    next_wp_idx = wp_idx + 1
    if next_wp_idx >= k:
        next_wp_idx = k - 1

    a = np.array(waypoints[next_wp_idx]).flatten() - np.array(waypoints[wp_idx]).flatten()
    
    b = np.array(pos).flatten() - np.array(waypoints[wp_idx]).flatten()
    
    ex = np.linalg.norm(a)
    a_n = a / ex
    b_n = b / np.linalg.norm(b)
    phi = np.arccos(np.dot(a_n, b_n))
    
    cte = np.linalg.norm(b) * math.sin(phi)
    
    theta = np.arccos(np.dot(vel_vect, a)/(
        np.linalg.norm(vel_vect)*np.linalg.norm(a)))
    
    return cte, theta

waypoints_list = []

k = 40
x0 = 0
y0 = -20
r = 20
for i in range(k):
    x = x0 + r * math.cos(2 * math.pi * i / k)
    y = y0 + r * math.sin(2 * math.pi * i / k)
    waypoints_list.append((x,y))

waypoints_list = waypoints_list[k//4+1:] + waypoints_list[:k//4+1]

waypoints_list.reverse()

waypoints_list = waypoints_list[0:11]

first_tuple_elements = [a_tuple[0] for a_tuple in waypoints_list]
second_tuple_elements = [a_tuple[1] for a_tuple in waypoints_list]

plt.scatter(first_tuple_elements, second_tuple_elements, color = 'red', 
            label = 'Waypoint')

initial_ys = np.load("initial_ys2.npy")
initial_yaws = np.load("initial_yaws.npy")

datas = []    
pathlist = Path('/Users/abdullahk047/Desktop/data2/').rglob('*.txt')
for path in pathlist:
     # because path is object not string
     path_in_str = str(path)
     datas.append(pd.read_csv(path_in_str, sep='\t'))


for i in range(10):
        plt.plot(datas[i]['POS_X'], datas[i]['POS_Y'])

plt.scatter(np.zeros(10), initial_ys, color = 'green', 
            label = 'initial position')

plt.legend(fontsize = 16)
plt.xlabel('pos_x', fontsize = 16)
plt.ylabel('pos_y', fontsize = 16)
plt.title("Trajectory Generated for 10 random initial positions", 
          fontsize = 16)
plt.show()

for i in range(10):
    plt.plot(datas[i]['POS_X'], datas[i]['POS_Y'], 
                 label= "traj " + str(i))

    plt.scatter(0, initial_ys[i], color = 'green', 
            label = 'initial position')
    plt.scatter(first_tuple_elements, second_tuple_elements, color = 'red', 
            label = 'Waypoint')
    plt.xlabel('pos_x', fontsize = 16)
    plt.ylabel('pos_y', fontsize = 16)
    plt.title("Trajectory Generated", fontsize = 16)
    plt.show()

    ctes = []
    theta_e = []
    for idx, row in datas[i].iterrows():
        pos = (row['POS_X'], row['POS_Y'])
        vel_vect = (row['v_X'], row['v_Y'])
        closest_wp = closest_waypoint(pos, waypoints_list)
        ctec, theta = cte(pos, vel_vect, closest_wp, 11, waypoints_list)
        ctes.append(ctec)
        theta_e.append(theta)
        
    datas[i]['ctes'] = ctes
    datas[i]['Theta_e'] = theta_e
    datas[i]['TimeStamp'] = (datas[i]['TimeStamp'] - datas[i]['TimeStamp'][0]) / 1000.0
    plt.plot(datas[i]['TimeStamp'], datas[i]['ctes'])
    plt.xlabel('Time (s)', fontsize = 16)
    plt.ylabel('CTE (m)', fontsize = 16)
    plt.title("Cross-track Error vs Time", fontsize = 16)
    plt.show()
    
    plt.plot(datas[i]['TimeStamp'], datas[i]['Theta_e'])
    plt.xlabel('Time (s)', fontsize = 16)
    plt.ylabel('Theta_e (rad)', fontsize = 16)
    plt.title("Error Angle vs Time", fontsize = 16)
    plt.show()
    
    datas[i].to_csv(ospath('~/Desktop/data/' + str(i) + '.txt'),
                    sep = '\t', index = False)
    #datas[i].to_pickle(ospath('~/Desktop/data/' + str(i) + '.pkl'))
    
    
