#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul  5 16:32:46 2021

@author: abdullahk047
"""

import pandas as pd
from os.path import expanduser as ospath
import math
import matplotlib.pyplot as plt
import numpy as np


def closest_waypoint(pos, waypoints):
    waypoints = np.asarray(waypoints)
    distances = np.sum((waypoints - pos)**2, axis=1)
    return np.argmin(distances)

def cte(pos, wp_idx, k, waypoints):
    next_wp_idx = wp_idx + 1
    if next_wp_idx >= k:
        next_wp_idx = 0
      
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
    
    return cte

df = pd.read_excel(ospath('~/Desktop/airsim7-5-21.xlsx'), 
                   sheet_name = 'Sheet9')
print(df)

#print(waypoints_list)

waypoints_list = [(0,0), (10,0), (20,0), (30,0), (40,0), 
                  (45,-5), 
                  (50,-10), (50,-20), (50,-30), (50,-40), 
                  (45,-45), 
                  (40,-50), (30,-50), (20,-50), (10,-50), 
                  (5,-45),
                  (0,-40), (0,-30), (0,-20), (0,-10)]

k = 20

first_tuple_elements = [a_tuple[0] for a_tuple in waypoints_list]
second_tuple_elements = [a_tuple[1] for a_tuple in waypoints_list]

plt.plot(first_tuple_elements, second_tuple_elements)

ctes = []
for idx, row in df.iterrows():
    pos = (row['POS_X'], row['POS_Y'])
    closest_wp = closest_waypoint(pos, waypoints_list)
    ctec = cte(pos, closest_wp, k, waypoints_list)
    ctes.append(ctec)
    
df['ctes'] = ctes
df.to_excel(ospath('~/Desktop/airsimdatawctes.xlsx'))
