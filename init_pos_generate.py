#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 27 00:56:02 2021

@author: abdullahk047
"""
import random
import numpy as np
import math


n = 100000
initial_ys = np.zeros(n)

for i in range(n):
    initial_ys[i] = random.uniform(-5, 5)

np.save("initial_ys", initial_ys)


initial_yaws = np.zeros(n)
for i in range(n):
    initial_yaws[i] = random.uniform(0, 2*math.pi)

np.save("initial_yaws", initial_yaws)