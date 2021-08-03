#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul  5 13:55:51 2021

@author: abdullahk047
"""

import setup_path
import airsim

import sys
import time

import math
import numpy as np

print("""This script is designed to fly in a circular path in Blocks environment""")

client = airsim.MultirotorClient()


radius = 20
k = 40
x0 = 0
y0 = -20
waypoints = []
waypoints_list = []

z = -30

for i in range(k):
    wp_x = x0 + radius * math.cos(2 * math.pi * i / k)
    wp_y = y0 + radius * math.sin(2 * math.pi * i / k)
    waypoints.append(airsim.Vector3r(wp_x, wp_y, z))
    waypoints_list.append((wp_x, wp_y))
    
waypoints_list = waypoints_list[k//4+1:] + waypoints_list[:k//4+1]
waypoints = waypoints[k//4+1:] + waypoints[:k//4+1]

waypoints_list.reverse()
waypoints.reverse()

waypoints_list = waypoints_list[0:k//4+1]
waypoints = waypoints[0:k//4+1]

initial_ys = np.load("initial_ys.npy")
initial_yaws = np.load("initial_yaws.npy")

for i in range(10):
    client.confirmConnection()
    client.enableApiControl(True)

    print("arming the drone...")
    client.armDisarm(True)

    state = client.getMultirotorState()
    if state.landed_state == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync().join()
    else:
        client.hoverAsync().join()

    time.sleep(1)

    state = client.getMultirotorState()
    if state.landed_state == airsim.LandedState.Landed:
        print("take off failed...")
        sys.exit(1)

    # AirSim uses NED coordinates so negative axis is up.
    # z of -5 is 5 meters above the original launch point.
    
    print("make sure we are hovering at {} meters...".format(-z))
    client.moveToZAsync(z, 1).join()

    
    for j in range(3):
        client.moveToPositionAsync(0,initial_ys[i],z, 1, 
                           yaw_mode = airsim.YawMode(False,0)).join()
        client.rotateToYawAsync(initial_yaws[i])
        time.sleep(1)
        


    print("flying on path...")
    
    client.startRecording()
    
    result = client.moveOnPathAsync(waypoints,
                        5, 120,
                        airsim.DrivetrainType.ForwardOnly, 
                        airsim.YawMode(False,0), 20, 1).join()

    client.stopRecording()

    # drone will over-shoot so we bring it back to the start point before landing.
    client.moveToPositionAsync(0,0,z,1).join()
    print("landing...")
    client.landAsync().join()
    print("disarming...")
    client.armDisarm(False)
    client.enableApiControl(False)
    print("done.")