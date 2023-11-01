'''
THIS IS WHERE THE UR3 BURGER ASSSEMBLY WILL BE DONE BEFORE PUSHING IT INTO MAIN.PY
'''


import roboticstoolbox as rtb
from spatialmath import SE3
from ir_support import UR5
from math import pi, ceil
from spatialmath.base import *
from roboticstoolbox import models, jtraj, trapezoidal
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy import linalg


class AssemblyRobot:

    def __init__(self):
        self.robot = UR5()
        self.currentPoint = [0,0,0]

    def setPose(self, position):
        self.robot.base = position

    def move(self, mode, targetPoint=[0,0,0], targetRPY=[0,0,0], t=3):
                                             # Total time (s)
        delta_t = 0.02                             # Control frequency
        steps = int(t/delta_t)                     # No. of steps for simulation
        delta = 2*pi/steps                         # Small angle change
        epsilon = 0.1                              # Threshold value for manipulability/Damped Least Squares
        W = np.diag([1, 1, 1, 1, 1, 1])      # Weighting matrix for the velocity vector

        # 1.2) Allocate array data
        m = np.zeros([steps,1])                    # Array for Measure of Manipulability
        q_matrix = np.zeros([steps,6])             # Array for joint anglesR
        qdot = np.zeros([steps,6])                 # Array for joint velocities
        theta = np.zeros([3,steps])                # Array for roll-pitch-yaw angles
        x = np.zeros([3,steps])                    # Array for x-y-z trajectory

        xPos, yPos, zPos = self.robot.base.A[:3,3]

        
        #relativeStartPoint = [0,0,0]

        fkine = self.robot.fkine(self.robot.q).A
        relX, relY, relZ = fkine[:3,-1] #xyz pos

        roll, pitch, yaw = tr2rpy(fkine[:3,:3], order='xyz')  #RPY values as a rotation matrix

        if(mode=='circle'): 
            circleRadius = 0.05
            circleStep = (circleRadius*2)/(steps/2) #((2*pi * circleRadius)/steps)
            circlePoint = x = np.zeros([3,steps])

            i = 0
            for circleHalf in range(0,2):
                for x1 in np.arange(-circleRadius, circleRadius, circleStep):
                    if circleHalf == 0:
                        y1 = np.sqrt(circleRadius**2 - x1**2)
                    else:
                        x1 = -x1
                        y1 = -np.sqrt(circleRadius**2 - x1**2)
                    
                    # x[0,i] = xPos + x1 #+ relX
                    # x[1,i] = yPos + y1 #+ relY
                    # x[2,i] = zPos      #+    relZ
                    x[0,i] = x1 + relX + circleRadius
                    x[1,i] = y1 + relY
                    x[2,i] = relZ

                    theta[0,i] = roll                         # Roll angle 
                    theta[1,i] = pitch                         # Pitch angle
                    theta[2,i] = yaw                         # Yaw angle

                    i+=1

        if(mode=='point'):
            relativeStartPoint = self.robot.fkine(self.robot.q).A[:3,-1]#[0,0.7,0.6]
            relativeTargetPoint = targetPoint#[0.2,0.5,0.2]#[0,0.7,0.3]

            # 1.3) Set up trajectory, initial pose
            s = trapezoidal(0,1,steps).q               # Trapezoidal trajectory scalar
            for i in range(steps):
                x[0,i] = (1-s[i])*(relX) + s[i]*(xPos+relativeTargetPoint[0])    # Points in x
                x[1,i] = (1-s[i])*(relY) + s[i]*(yPos+relativeTargetPoint[1])    # Points in y
                x[2,i] = (1-s[i])*(relZ) + s[i]*(zPos+relativeTargetPoint[2])     # Points in z
                #x[2,i] = (1-s[i])*0.4 + s[i]*0.6
                theta[0,i] = (1-s[i])*(roll) + s[i]*(targetRPY[0])                         # Roll angle 
                theta[1,i] = (1-s[i])*(pitch) + s[i]*(targetRPY[1])                # Pitch angle
                theta[2,i] = (1-s[i])*(yaw) + s[i]*(targetRPY[2])                        # Yaw angle

        
        T = transl(x[:,0]) @ rpy2tr(theta[0,0], theta[1,0], theta[2,0])       # Create transformation of first point and angle     
        q0 = np.zeros([1,6])                                                  # Initial guess for joint angles
        #q_matrix[0,:] = self.robot.ikine_LM(T, q0).q                                # Solve joint angles to achieve first waypoint
        q_matrix[0,:] = self.robot.ikine_LM(T, self.robot.q, joint_limits=True).q 
        qlim = np.transpose(self.robot.qlim)
        
        # 1.4) Track the trajectory with RMRC
        for i in range(steps-1):
            T = self.robot.fkine(q_matrix[i,:]).A                                   # Get forward transformation at current joint state
            delta_x = x[:,i+1] - T[:3,3]                                      # Get position error from next waypoint
            Rd = rpy2r(theta[0,i+1], theta[1,i+1], theta[2,i+1])              # Get next RPY angles, convert to rotation matrix
            Ra = T[:3,:3]                                                     # Current end-effector rotation matrix
            Rdot = (1/delta_t)*(Rd - Ra)                                      # Calculate rotation matrix error
            S = Rdot @ Ra.T                                                   # Skew symmetric!

            linear_velocity = (1/delta_t)*delta_x
            angular_velocity = np.array([S[2,1], S[0,2], S[1,0]])             # Check the structure of Skew Symmetric matrix!!
            xdot = W @ np.vstack((linear_velocity.reshape(3,1), 
                                angular_velocity.reshape(3,1)))             # Calculate end-effector velocity to reach next waypoint.
            
            J = self.robot.jacob0(q_matrix[i,:])                                    # Get Jacobian at current joint state
            m[i] = np.sqrt(linalg.det(J @ J.T))
            if m[i] < epsilon:                                                # If manipulability is less than given threshold
                m_lambda = (1 - m[i]/epsilon) * 0.05
            else:
                m_lambda = 0

            inv_j = linalg.inv(J.T @ J + m_lambda * np.eye(6)) @ J.T          # DLS Inverse
            qdot[i,:] = (inv_j @ xdot).T                                      # Solve the RMRC equation (you may need to transpose the         vector)
            for j in range(6):                                                # Loop through joints 1 to 6
                if q_matrix[i,j] + delta_t*qdot[i,j] < qlim[j,0]:             # If next joint angle is lower than joint limit...
                    qdot[i,j] = 0 # Stop the motor
                elif q_matrix[i,j] + delta_t*qdot[i,j] > qlim[j,1]:           # If next joint angle is greater than joint limit ...
                    qdot[i,j] = 0 # Stop the motor
                
            q_matrix[i+1,:] = q_matrix[i,:] + delta_t*qdot[i,:]               # Update next joint state based on joint velocities

        return q_matrix