# -*- coding: utf-8 -*-
"""
Created on Fri Aug 16 17:04:20 2019

@author: Giuseppe Sensoli Arra'
"""

from planar_robot import planar_robot
from planar_robot import rad2deg


# MAIN ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

robot = planar_robot([10., 10.])

# direct kinematics----------------------
robot.direct_kinematics( [45, 0] )
if type(robot.TOE)==int and robot.TOE==-1:
    print("ERROR: link-joint dimensions does not match!")
else:
    print("TOE:")
    for row in robot.TOE: print(row)

# inverse kinematics---------------------
ee_pos = robot.get_end_effector_pos()
print("\nactual joint angles: q =", rad2deg(robot.q))
print("e-e pos: p =", ee_pos)
joint = robot.RR_inverse_kinematics(ee_pos)
if type(joint)==int and joint_==-1:
    print("ERROR: point", ee_pos, "unreachable because out of Workspace 1")
else:
    print("inverse kinematics output: q =", joint)
    

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::