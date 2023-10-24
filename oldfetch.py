import swift
import roboticstoolbox as rtb
from spatialmath.base import *
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
# Useful variables
from math import pi
import scipy.spatial
import numpy as np
import matplotlib.pyplot as plt




class FetchRobot(DHRobot3D):
    '''
    Used to Incorporate UR3 onto existing Linear Rails
    '''
    
    def __init__(self):
        self.link3D_names = dict(link0 = 'fetch_meshes/robotbase', color0 = (0.8,0.8,0.8,1),      # color option only takes effect for stl file
                            link1 = 'fetch_meshes/shoulder_pan_link', color1 = (0.8,0.8,0.8,1),
                            link2 = 'fetch_meshes/shoulder_lift_link', color2 = (0.8,0.8,0.8,1),
                            link3 = 'fetch_meshes/upperarm_roll_link', color3 = (0.8,0.8,0.8,1),
                            link4 = 'fetch_meshes/elbow_flex_link', color4 = (0.8,0.8,0.8,1),
                            link5 = 'fetch_meshes/forearm_roll_link', color5 = (0.8,0.8,0.8,1),
                            link6 = 'fetch_meshes/wrist_flex_link',color6 = (0.8,0.8,0.8,1),
                            link7 = 'fetch_meshes/joinedgripper',color7 = (0.8,0.8,0.8,1),
                            link8 = 'fetch_meshes/gripper_finger',color8 = (0.8,0.8,0.8,1),
                            link9 = 'fetch_meshes/gripper_finger',color9= (0.8,0.8,0.8,1)
                            )
     
        self.loadModel()
    
    def DHParams(self):
        '''
        TODO: Find the appropriate DH Params for the Fetch Robot 
        '''
        links = []    # Prismatic Link
        # print (links[0])
       # Combined DH parameters
        d = [ 0.065, 0.218, 0.082, 0.196, 0.122, 0.31, 0, 0,0]
        a = [0.12, 0, 0, 0, 0, 0, 0, 0,0]
        alpha = [-pi/2,0,pi/2,-pi/2,0,pi/2,-pi/2,0,pi]
        # offset = [0,0,pi/2,0,pi/2,0,pi/2,0,0]
        qlim = [[-pi, pi] for _ in range(9)]
        for i in range(9):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i])
            links.append(link)

        # links = [rtb.PrismaticDH(theta= 0, a= 0, alpha= 0, qlim= [-0.05, 0]),
        #          rtb.PrismaticDH(theta= 0, a= 0, alpha=0, qlim= [0, 0.05])]  
    
        return links
    

    def loadModel(self):
        '''
        Loads model into Swift simulation
        based off ir_support/robots:
            - LinearUR5
            - UR3

        UPDATES:

        Attempt 4: 16/10/2023
        Jared: Cannot figure it out and finds the fetch robot in roboticstoolbox. AWESOME :(

        Attempt 3: 15/10/23:
        Jaden: When setting the DH models it seems that the origin 'q' will always stay at origin, so changing transformation to account for that. 

        Attempt 2: 14/10/23:
        Jaden: Attempted to do make base prismatic link,however would need an additional 'base' .dae model
        Its possible but may keep it simple for now considering there is no need for it

        Attempt 1: 13/10/23
        Jaden/Jared: Models transformed in swift env however DH params not set
        '''
                # A joint config and the 3D object transforms to match that config
        qtest = [0,0,0,0,0,0,0,0,0]

        qtest_transforms = [ spb.transl(-0.12, 0,-0.6775), # Base
                            spb.transl(0, 0,0) , # Shoulder Pan Link
                            spb.transl(0.12, 0,0.065) , # Shoulder Lift Link
                            spb.transl(0.338,0,0.065) ,  # Upperarm Roll
                            spb.transl(0.47,0, 0.065)  , # Elbow Flex
                            spb.transl(0.666, 0,0.065),# Forearm Roll
                            spb.transl(0.792, 0,0.065) ,   # Wrist Flex
                            spb.transl(0.933, 0,0.065)  ,# Wrist Roll + Gripper
                            spb.transl(1.168, 0.04,0.0645)  , #L Gripper Finger
                            spb.transl(1.168, -0.04,0.0645), #R Gripper Finger
                            ] 
        current_path = os.path.abspath(os.path.dirname(__file__))
        links= self.DHParams()
        print(len(links))
        super().__init__(links, self.link3D_names, name = 'Fetch_Robot', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        # self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2)

        # Shifting Model up so appears to sit on plane
        self.base = self.base * SE3(0.12,0,0.6675)
        self.q = qtest
    
    def test2(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)
        # self.add_to_env(env)
        fetch = rtb.models.Fetch()
        print(fetch.q)
        # Add the robot to the simulator
        env.add(fetch)

        q_goal = [fetch.q[i]-pi/3 for i in range(len(fetch.q))]
        # q_goal[0] = self.q[0]
        # q_goal[0] = -1 # Move the rail link
        qtraj = rtb.jtraj(fetch.q, q_goal, 50).q


        fig = self.plot(self.q, limits= [-1,1,-1,1,-1,1])
        fig._add_teach_panel(self, self.q)

        # env.hold()
        for q in qtraj:
            fetch.q = q
            env.step(0.02)
            fig.step(0.01)
            # time.sleep(1)
        fig.hold()
        env.hold()
        # time.sleep(3)    
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)
        self.add_to_env(env)
        
        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        # q_goal[0] = self.q[0]
        # q_goal[0] = -1 # Move the rail link
        qtraj = rtb.jtraj(self.q, q_goal, 50).q


        fig = self.plot(self.q, limits= [-1,1,-1,1,-1,1])
        fig._add_teach_panel(self, self.q)

        # env.hold()
        for q in qtraj:
            self.q = q
            env.step(0.02)
            fig.step(0.01)
            # time.sleep(1)
        fig.hold()
        env.hold()
        # time.sleep(3)    

if __name__ == "__main__":  
    r = FetchRobot()    
    # r.test()    # The original
    r.test2()   # NEW


    