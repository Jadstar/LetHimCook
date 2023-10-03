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

# -----------------------------------------------------------------------------------#
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

class LinearUR3(DHRobot3D):
    def __init__(self):         
        # DH links
        links = self._create_DH()     
        
        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'base_rail', color0 = (0.2,0.2,0.2,1),      # color option only takes effect for stl file
                            link1 = 'slider_rail', color1 = (0.1,0.1,0.1,1),
                            link2 = 'shoulder_ur3', 
                            link3 = 'upperarm_ur3', 
                            link4 = 'forearm_ur3', 
                            link5 = 'wrist1_ur3', 
                            link6 = 'wrist2_ur3', 
                            link7 = 'wrist3_ur3',
                            )
        # A joint config and the 3D object transforms to match that config
        qtest = [0, 0, -pi/2, 0, 0, 0, 0]
        
        
        qtest_transforms = [spb.transl(0, 0, 0), 
                            spb.trotx(-pi/2),
                            spb.transl(0,0.15239,0) @ spb.rpy2tr(0,0,-pi/2, order = 'xyz'), 
                            spb.transl(0,0.1524,-0.12) @ spb.rpy2tr(0,0,-pi/2, order = 'xyz'),    
                            spb.transl(0,0.39583,-0.027115) @ spb.rpy2tr(0,0,-pi/2, order = 'xyz'), 
                            spb.transl(0,0.60903,-0.027316) @ spb.rpy2tr(0,pi/2,-pi/2, order = 'xyz'), 
                            spb.transl(0.000389,0.60902,-0.11253) @ spb.rpy2tr(0,pi/2,-pi/2, order = 'xyz'), 
                            spb.transl(0.083765,0.61096,-0.11333) @ spb.rpy2tr(0,0,-pi/2, order = 'xyz'),
                        ]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'LinearUR3', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2)
        self.gripper_robot = GripperRobot()
        self.q = qtest
        
        
    # -----------------------------------------------------------------------------------#    
    def _create_DH(self):
        links = [rtb.PrismaticDH(theta= pi, a= 0, alpha= pi/2, qlim= [-0.8, 0])]    # Prismatic Link
        # Given DH parameters for UR3 and the Linear Rail
    
        # Combined DH parameters
        a = [0, -0.24365, -0.21325, 0, 0, 0]
        d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
        alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]
        qlim = [[-2*pi, 2*pi] for _ in range(6)]
        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i])
            links.append(link)
        return links
                    
                    

    def test(self):
        # Load the .mat file
        data = scipy.io.loadmat(r'../ur3_q.mat')
        r = UR3()
        env = swift.Swift()
        env.launch(realtime=True)
        r.add_to_env(env)
        # Extract joint values (assuming they are stored in a variable named 'q_values' inside the .mat file)
        q_values = data['q']
        
        # Iterate through the joint values and apply them to your robot
        for q in q_values :
            r.q = q
            # If using a visualization environment, you'd update the visualization here
            env.step(0.02)  # Update the environment
            time.sleep(3)  # Optionally add a delay for visualization purposes
    def getjointstate(self, target_pose):
        joint_states = self.ikine_LM(target_pose)
        
        return joint_states

    def teach_robot(self):
 
        # Create a Swift environment
        env = swift.Swift()
        env.launch(realtime=True)
        
        # Add the robot to the environment
        self.add_to_env(env)
        q_initial = self.q
        # Call the teach method
        self.teach(q_initial)
        
        # Keep the environment running
        while True:
            env.step(0.1)
            time.sleep(0.1)
    def goto_jointstate(self, desired_pose, env, steps=50):
        
        # Get the desired joint states for the desired pose
        desired_joint_states = self.getjointstate(desired_pose)
        
        # Compute a joint trajectory from the current state to the desired state
        traj = rtb.jtraj(self.q, desired_joint_states.q, steps)
        
        # Move the robot through the trajectory
        for q in traj.q:
            self.q = q
            env.step(0.02)  # Update the environment
            time.sleep(0.02)
            
        # Compute the actual end effector pose after moving
        actual_pose = self.fkine(self.q)
        
        # Calculate the distance (Euclidean) between the desired and actual end effector positions
        distance = np.linalg.norm(desired_pose.t - actual_pose.t)
        return distance * 1000 #mm
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":  
    r = LinearUR3()
    # r.test()
    

    