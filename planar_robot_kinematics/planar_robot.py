# -*- coding: utf-8 -*-
"""
Created on Mon Aug 19 10:50:56 2019

@author: giuse
"""
from numpy import *
from math import atan2

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
def rad2deg(a):
    if type(a)==int or type(a)==float: return a*(180/pi)
    if type(a)==list:
        for i in range(0, len(a)): a[i] *= (180/pi)
        return a

def deg2rad(a):
    if type(a)==int or type(a)==float: return a*(pi/180)
    if type(a)==list:
        for i in range(0, len(a)): a[i] *= (pi/180)
        return a

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
def denavit_hartemberg(DH_table_row):
    a = DH_table_row[0]     # displacement along x axis
    alpha = DH_table_row[2] # rotation along x axis
    d = DH_table_row[2]     # displacement along z axis
    theta = DH_table_row[3] # rotation along z axis
    # Denavit-Hartemberg transformation matrix
    DH = array([ [cos(theta), -sin(theta), 0, a*cos(theta)],
                 [sin(theta), cos(theta),  0, a*sin(theta)],
                 [0,          0,           1, d           ],
                 [0,          0,           0, 1           ],
               ])
    return DH

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
class planar_robot:
    
    def __init__(self, link):
        self.n = len(link)  # joint number
        self.l = link       # link length
        self.q = []         # joint angle
        for i in range(0,len(link)):
            self.q.append(0.)
        self.TOE = []       # Origin to end-effector transformation matrix
        
    def get_end_effector_pos(self):
        return [self.TOE[0][3], self.TOE[1][3], self.TOE[2][3]]

    def direct_kinematics(self, q):
        # params error check
        if len(q) != self.n:
            return -1
        else: self.q= q     
        a =     self.l          # displacement along x axis
        alpha = zeros(self.n)   # rotation along x axis
        d =     zeros(self.n)   # displacement along z axis
        theta = deg2rad(q)      # rotation along z axis
        
        # build DH table [a, alpha, d, theta]
        DH_table = []
        for row in range(0,self.n): DH_table.append( [a[row], alpha[row], d[row], theta[row]] )
    
        # Compute Transformation matrices between consecutive frames
        A = []
        for row in range(0,self.n): A.append(eye(4))
        for i in range(0,self.n):
            A[i] = denavit_hartemberg(DH_table[i])
        
        # Compute transformation matrix from O to e-e
        TOE = I = eye(4)    #from origin to end effector
        for i in range(0,self.n) : TOE = TOE @ A[i]
        
        for i in range(0,4):
            for j in range(0,4):
                TOE[i][j] = round(TOE[i][j], 3)
        
        self.TOE = TOE
    
#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    def RR_inverse_kinematics(self, p):
        px = p[0]
        py = p[1]
        #pz = p[2]
        
        if (px*px + py*py) >= (self.l[0]*self.l[0] + self.l[1]*self.l[1] + 2*self.l[0]*self.l[1]):
            return -1
        
        c2 = (px*px + py*py - self.l[0]*self.l[0] - self.l[1]*self.l[1]) / (2*self.l[0]*self.l[1])
        s2 = sqrt(1 - c2*c2)
        q2 = atan2(s2, c2)
        q1 = atan2(py, px) - atan2(self.l[1]*s2, self.l[0]+self.l[1]+c2)
        
        return [round(rad2deg(q1),3), round(rad2deg(q2),3)]