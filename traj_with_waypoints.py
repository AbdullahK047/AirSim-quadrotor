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

print("""This script is designed to fly in a square path in Blocks environment""")

client = airsim.MultirotorClient()
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
z = -30
print("make sure we are hovering at {} meters...".format(-z))
client.moveToZAsync(z, 1).join()


    
client.moveToPositionAsync(0,0,z, 1, 
                           yaw_mode = airsim.YawMode(False,0)).join()

print("at 0,0,-30")

waypoints = [airsim.Vector3r(50,0,z), airsim.Vector3r(50,-50,z), airsim.Vector3r(0,-50,z), airsim.Vector3r(0,0,z)]

print("flying on path...")
client.startRecording()

result = client.moveOnPathAsync(waypoints, velocity = 6, drivetrain = airsim.DrivetrainType.ForwardOnly, 
yaw_mode = airsim.YawMode(False,0), lookahead = -1, adaptive_lookahead = 0).join()

client.stopRecording()
print("circle complete")

# drone will over-shoot so we bring it back to the start point before landing.
client.moveToPositionAsync(0,0,z,1).join()
print("landing...")
client.landAsync().join()
print("disarming...")
client.armDisarm(False)
client.enableApiControl(False)
print("done.")