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
      
    #print('pos = {}, nearestwp = {}, thiswp = {}, nextwp = {}'.format(
        #pos,wp_idx,waypoints[wp_idx], waypoints[next_wp_idx]))
    a = np.array(waypoints[next_wp_idx]).flatten() - np.array(waypoints[wp_idx]).flatten()
    
    #print('pos = {}, nearestwp = {}, thiswp = {}, nextwp = {}, a = {}'.format(
        #pos,wp_idx,waypoints[wp_idx], waypoints[next_wp_idx], a))
    
    b = np.array(pos).flatten() - np.array(waypoints[wp_idx]).flatten()
    
    ex = np.linalg.norm(a)
    #print(waypoints)
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
        plt.plot(datas[i]['POS_X'], datas[i]['POS_Y'])#, 
#                 label= "traj " + str(i))
        #plt.arrow(0, initial_ys[i], 
        #  math.cos(initial_yaws[i]-0.5*math.pi), math.sin(initial_yaws[i]-0.5*math.pi), 
        #  shape = 'full', head_width = 0.5, 
        #  label = 'initial orientation')

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
    #print(df['TimeStamp'][0])
    datas[i]['TimeStamp'] = (datas[i]['TimeStamp'] - datas[i]['TimeStamp'][0]) / 1000.0
    print("hereeee")
    #print(datas[i])
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
    
    datas[i].to_pickle(ospath('~/Desktop/data/' + str(i) + '.pkl'))

#print(waypoints_list)



"""
waypoints_list = [(0,0), (10,0), (20,0), (30,0), (40,0), 
                  (45,-5), 
                  (50,-10), (50,-20), (50,-30), (50,-40), 
                  (45,-45), 
                  (40,-50), (30,-50), (20,-50), (10,-50), 
                  (5,-45),
                  (0,-40), (0,-30), (0,-20), (0,-10)]
"""

"""
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

initial_y = random.uniform(-19, 19)

first_tuple_elements = [a_tuple[0] for a_tuple in waypoints_list]
second_tuple_elements = [a_tuple[1] for a_tuple in waypoints_list]

plt.figure()
plt.plot(df['POS_X'], df['POS_Y'], label = 'quadrotor real path')
plt.scatter(first_tuple_elements, second_tuple_elements, color = 'red', 
            label = 'Waypoint')
plt.plot(0, initial_y, 'go', label = 'initial position')
plt.plot(0, random.uniform(-19, 19), 'go')
plt.plot(0, random.uniform(-19, 19), 'go')
plt.plot(0, random.uniform(-19, 19), 'go')
plt.plot(0, random.uniform(-19, 19), 'go')
plt.plot(0, random.uniform(-19, 19), 'go')
plt.plot(0, random.uniform(-19, 19), 'go')
plt.plot(0, random.uniform(-19, 19), 'go')
plt.arrow(0, initial_y, -1, -1, shape = 'full', head_width = 1, 
          label = 'initial orientation')


plt.xlabel('pos_x')
plt.ylabel('pos_y')
plt.legend(loc = 'lower left')
plt.show()


ctes = []
theta_e = []
for idx, row in df.iterrows():
    pos = (row['POS_X'], row['POS_Y'])
    vel_vect = (row['v_X'], row['v_Y'])
    closest_wp = closest_waypoint(pos, waypoints_list)
    ctec, theta = cte(pos, vel_vect, closest_wp, k, waypoints_list)
    ctes.append(ctec)
    theta_e.append(theta)
    
df['ctes'] = ctes
df['Theta_e'] = theta_e
#print(df['TimeStamp'][0])
df['TimeStamp'] = (df['TimeStamp'] - df['TimeStamp'][0]) / 1000.0

plt.plot(df['TimeStamp'], df['ctes'])
plt.xlabel('Time (s)')
plt.ylabel('CTE (m)')
plt.show()

plt.plot(df['TimeStamp'], df['Theta_e'])
plt.xlabel('Time (s)')
plt.ylabel('Theta_e (rad)')

#df.to_excel(ospath('~/Desktop/airsimdatawctes7-6-21,2.xlsx'))
    
    """

    
    
    
    
    
    
    ##################################
    

    
    
