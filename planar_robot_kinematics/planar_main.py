# -*- coding: utf-8 -*-
"""
Created on Fri Aug 16 17:04:20 2019

@author: Giuseppe Sensolini Arra'
"""

from planar_robot import planar_robot
from planar_robot import rad2deg


# MAIN ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

robot = planar_robot([0.6, 0.5])
joint = [-79.6111, 90.0]
# direct kinematics----------------------
print("\nDIRECT KINEMATICS:\n{ joints =", joint, "}")
robot.direct_kinematics( joint )
if type(robot.TOE)==int and robot.TOE==-1:
    print("ERROR: link-joint dimensions does not match!")
else:
    print("direct kinematics output:\nTOE =")
    for row in robot.TOE: print(row)

# inverse kinematics---------------------
ee_pos = robot.get_end_effector_pos()
print("\nINVERSE KINEMATICS: \n{ end_effector_pos(x,y,z) =", ee_pos,"}")
joint = robot.RR_inverse_kinematics(ee_pos)
if type(joint)==int and joint_==-1:
    print("ERROR: point", ee_pos, "unreachable because out of Workspace 1")
else:
    print("inverse kinematics output:\nq_a =", joint[0], "\nq_b =", joint[1])
    

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
