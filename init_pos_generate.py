#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 27 00:56:02 2021

@author: abdullahk047
"""
import random
import numpy as np
import math
import matplotlib.pyplot as plt


initial_ys = np.zeros(10)

for i in range(10):
    initial_ys[i] = random.uniform(-5, 5)
    #x.append(0)
    
print(initial_ys)

plt.scatter([0,0,0,0,0,0,0,0,0,0], initial_ys)

np.save("initial_ys2", initial_ys)

"""
initial_yaws = np.zeros(10)
#x = []
print(initial_yaws)

for i in range(10):
    initial_yaws[i] = random.uniform(0, 2*math.pi)
    #x.append(0)
    
print(initial_yaws)

np.save("initial_yaws", initial_yaws)


    
plt.scatter(x, initial_ys)

initial_ys = np.load("initial_ys.npy")
initial_yaws = np.load("initial_yaws.npy")

#plt.scatter([0,0,0,0,0,0,0,0,0,0], initial_ys)
plt.scatter([0,0,0,0,0,0,0,0,0,0], initial_yaws, c="red")
"""