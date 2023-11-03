'''
THIS IS WHERE THE UR3 BURGER ASSSEMBLY WILL BE DONE BEFORE PUSHING IT INTO MAIN.PY
'''


import roboticstoolbox as rtb
from spatialmath import SE3
from ir_support import UR3
from math import pi, ceil
from spatialmath.base import *
from roboticstoolbox import models, jtraj, trapezoidal
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy import linalg
import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support import UR5, UR3, LinearUR5
import spatialgeometry as geometry
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
import scipy.io
import numpy as np
# Useful variables
from math import pi

class Finger1(DHRobot3D):
    def __init__(self):
        linksf1 = self._create_finger1_DH()
        link3D_names = dict(link0='Finger1Base', color0=(0.2, 0.2, 0.2, 1),
                        link1='Finger1Link1',
                        link2='Finger1Link2',
                        link3='Finger1Link3')
        qtest = [0, 0, 0]
        qtest_transforms = [spb.transl(-0.035,0.075,0.035),
                            spb.transl(-0.065,0.115,0.035),
                            spb.transl(-0.095,0.155,0.035),
                            spb.transl(-0.105,0.175,0.035)]
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(linksf1, link3D_names, name='Finger1', link3d_dir=current_path, qtest=qtest, qtest_transforms=qtest_transforms)
        self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2)
        self.q = qtest
    def _create_finger1_DH(self):
        links=[]
        a = [-0.065, -0.03, -0.01]
        d = [0, 0, 0]
        alpha = [0, 0, 0]
        qlim = [[0,-pi/15], [0,-pi/30], [0,-pi/45]]
        for i in range(3):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i])
            links.append(link)
        return links
class Finger2(DHRobot3D):
    def __init__(self):
        linksf2 = self._create_finger2_DH()
        link3D_names = dict(link0='Finger2Base', color0=(0.2, 0.2, 0.2, 1),
                        link1='Finger2Link1',
                        link2='Finger2Link2',
                        link3='Finger2Link3')
        qtest = [0, 0, 0]
        qtest_transforms = [spb.transl(-0.035,0.075,-0.035),
                            spb.transl(-0.065,0.115,-0.035),
                            spb.transl(-0.095,0.155,-0.035),
                            spb.transl(-0.105,0.175,-0.035)]
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(linksf2, link3D_names, name='Finger2', link3d_dir=current_path, qtest=qtest, qtest_transforms=qtest_transforms)
        self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2)
        self.q = qtest
    def _create_finger2_DH(self):
        links=[]
        a = [-0.065, -0.03, -0.01]
        d = [0, 0, 0]
        alpha = [0, 0, 0]
        qlim =  [[0,-pi/15], [0,-pi/30], [0,-pi/45]]
        for i in range(3):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i])
            links.append(link)
        return links
class Finger3(DHRobot3D):
    def __init__(self):
        linksf3 = self._create_finger3_DH()
        link3D_names = dict(link0='Finger3Base', color0=(0.2, 0.2, 0.2, 1),
                        link1='Finger3Link1',
                        link2='Finger3Link2',
                        link3='Finger3Link3')
        qtest = [0, 0, 0]
        qtest_transforms = [spb.transl(0.035,0.075,0) @ spb.troty(-pi),
                            spb.transl(0.065,0.115,0) @ spb.troty(-pi),
                            spb.transl(0.095,0.155,0) @ spb.troty(-pi),
                            spb.transl(0.105,0.175,0) @ spb.troty(-pi),]
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(linksf3, link3D_names, name='Finger3', link3d_dir=current_path, qtest=qtest, qtest_transforms=qtest_transforms)
        self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2) 
        self.q = qtest
    def _create_finger3_DH(self):
        links=[]
        a = [0.065, 0.03, 0.01]
        d = [0, 0, 0]
        alpha = [0, 0, 0]
        qlim =  [[0,pi/15], [0,pi/30], [0,pi/45]]
        for i in range(3):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i])
            links.append(link)
        return links


class GripperRobot():
    def __init__(self):
        self.palm_mesh = geometry.Mesh(r'LinearUR3\BaseGripper.dae', pose = SE3(0,0,0))
        self.palm_mesh.T = SE3.Trans(-0.1942 ,0.083765,0.61096).A @ SE3.Rz(pi/2).A  @ SE3.Ry(pi/2).A
        self.finger1 = Finger1()
        self.finger2 = Finger2()
        self.finger3 = Finger3()
        self.attachFingers()
        self.is_open = True
    def addtoEnv(self,env):
        env.add(self.palm_mesh)
        self.finger1.add_to_env(env)
        self.finger2.add_to_env(env)
        self.finger3.add_to_env(env)
    def attachFingers(self):
        # Attach Finger1 at some offset from the base
        offset_f1 = SE3(0,0,0)  # Adjust this to the correct position
        self.finger1.base = self.palm_mesh.T * offset_f1

        # Similarly for Finger2 and Finger3
        offset_f2 = SE3(0,0,0)  # Adjust this
        self.finger2.base = self.palm_mesh.T * offset_f2

        offset_f3 = SE3(0,0,0)  # Adjust this
        self.finger3.base = self.palm_mesh.T * offset_f3

    def move(self, T):
        # Move the gripper robot to the specified transformation matrix
        adjusted_T = T @ SE3.Rx(pi/2).A  # Rotate around the Z-axis by pi/2
        self.palm_mesh.T = adjusted_T
        self.finger1.base = adjusted_T
        self.finger1.q = self.finger1.q
        self.finger2.base = adjusted_T
        self.finger2.q = self.finger2.q
        self.finger3.base = adjusted_T
        self.finger3.q = self.finger3.q
    def open(self):
        return [0, 0, 0]
    def closed(self):
        return [-pi/18, -pi/36, -pi/54]
    def openhandtraj(self):
        return rtb.jtraj(self.closed(),self.open(), 25).q
    def closedhandtraj(self):
        return rtb.jtraj( self.open(), self.closed(),50).q

class AssemblyRobot:

    def __init__(self):
        self.robot = UR3()
        self.robot.gripper_robot = GripperRobot()
        self.currentPoint = [0,0,0]
        self.robot.q = [pi/2,-pi/2,-pi/5,7*pi/6,pi/2,0]

    def setPose(self, position):
        self.robot.base = position

    def generateRoutine(self):

        qMatrixRoutine = []

        qMatrixRoutine.append(self.move(mode='point', targetPoint=[0.13,0.36,0.02], targetRPY=[pi/2,0,pi], t=1)) #go to plate
        qMatrixRoutine.append(self.move(mode='point', targetPoint=[0.3,0.33,0.18], targetRPY=[pi/2,-pi/2,pi], t=1)) #carry to fetch bot
        qMatrixRoutine.append(self.move(mode='point', targetPoint=[0.13,0.36,0.02], targetRPY=[pi/2,0,pi], t=1)) #return plate
        qMatrixRoutine.append(self.move(mode='point', targetPoint=[-0.18,0.28,0.1], targetRPY=[pi/2,0,pi], t=1)) #move to sauce
        qMatrixRoutine.append(self.move(mode='point', targetPoint=[-0.05,0.36,0.2], targetRPY=[-pi/2,0,pi], t=1)) #move sauce over plate and turn over
        qMatrixRoutine.append(self.move(mode='circle', t=1.5, targetRPY=[-pi/2,0,pi])) #apply sauce
        qMatrixRoutine.append(self.move(mode='point', targetPoint=[-0.18,0.28,0.1], targetRPY=[pi/2,0,pi], t=1)) #return sauce
        qMatrixRoutine.append(self.move(mode='point', targetPoint=[0.13,0.36,0.02], targetRPY=[pi/2,0,pi], t=1)) #return to plate
        qMatrixRoutine.append(self.move(mode='point', targetPoint=[0.3,-0.33,0.33], targetRPY=[pi/2,3*pi/4,pi], t=1)) #carry plate to window

        return qMatrixRoutine






    def move(self, mode, targetPoint=[0,0,0], targetRPY=[0,0,0], t=3):
                                             # Total time (s)
        delta_t = 0.02                             # Control frequency
        steps = int(t/delta_t)                     # No. of steps for simulation
        delta = 2*pi/steps                         # Small angle change
        epsilon = 0.1                              # Threshold value for manipulability/Damped Least Squares
        W = np.diag([1, 1, 1, 0.3, 0.3, 0.3])      # Weighting matrix for the velocity vector

        if(steps%2!=0):
            steps+=1

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
            circleRadius = 0.02
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

                    theta[0,i] = targetRPY[0]                  
                    theta[1,i] = targetRPY[1]           
                    theta[2,i] = targetRPY[2]

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
                # theta[0,i] = (1-s[i])*(roll) + s[i]*(targetRPY[0])                         # Roll angle 
                # theta[1,i] = (1-s[i])*(pitch) + s[i]*(targetRPY[1])                # Pitch angle
                # theta[2,i] = (1-s[i])*(yaw) + s[i]*(targetRPY[2])                        # Yaw angle
                theta[0,i] = targetRPY[0]                  
                theta[1,i] = targetRPY[1]           
                theta[2,i] = targetRPY[2]

        
        # T = transl(x[:,0]) @ rpy2tr(theta[2,0], theta[0,0], theta[1,0])       # Create transformation of first point and angle     
        T = fkine       # Create transformation of first point and angle     
        q0 = np.zeros([1,6])                                                  # Initial guess for joint angles
        #q_matrix[0,:] = self.robot.ikine_LM(T, q0).q                                # Solve joint angles to achieve first waypoint
        q_matrix[0,:] = self.robot.ikine_LM(T, self.robot.q, joint_limits=True).q 
        qlim = np.transpose(self.robot.qlim)
        
        # 1.4) Track the trajectory with RMRC
        for i in range(steps-1):
            T = self.robot.fkine(q_matrix[i,:]).A                                   # Get forward transformation at current joint state
            delta_x = x[:,i+1] - T[:3,3]                                      # Get position error from next waypoint
            Rd = rpy2r(theta[2,i+1], theta[0,i+1], theta[1,i+1])              # Get next RPY angles, convert to rotation matrix
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

        self.robot.q = q_matrix[-1]

        return q_matrix