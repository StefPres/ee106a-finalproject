#!/usr/bin/env python

import numpy as np

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def direction_between(v1,v2):
    # returns the direction in radians between the two vectors in a right-hand-ruled frame, where the two vectors form a plane
    xproduct = np.cross(v1,v2)
    dir = -1 if np.sign(xproduct[2]) < 0 else 1
    return dir * angle_between(v1,v2)
