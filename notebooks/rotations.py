
import numpy as np
rads = np.pi / 180.0
1
def rotationX(theta_degrees):    
    theta = theta_degrees * rads
    matrix = np.array([[1,0,0],
                       [0, np.cos(theta), np.sin(theta) ],
                       [0, -np.sin(theta), np.cos(theta)]])
    return matrix


def rotationY(theta_degrees):
    theta = theta_degrees * rads
    matrix = np.array([[np.cos(theta),0,-np.sin(theta)],
                       [0, 1, 0],
                       [np.sin(theta),0, np.cos(theta)]])
    return matrix

def rotationZ(theta_degrees):    
    theta = theta_degrees * rads
    matrix = np.array([[ np.cos(theta), np.sin(theta), 0],
                       [-np.sin(theta), np.cos(theta), 0],
                       [0,0,1]])
    return matrix
