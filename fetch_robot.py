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
        self.link3D_names = dict(link0 = 'fetch_meshes/base_link', color0 = (0.8,0.8,0.8,1),      # color option only takes effect for stl file
                            link1 = 'fetch_meshes/torso_lift_link',color1 = (0.8,0.8,0.8,1),
                            link2 = 'fetch_meshes/head_pan_link', color2 = (0.8,0.8,0.8,1),
                            link3 = 'fetch_meshes/head_tilt_link', color3 = (0.8,0.8,0.8,1),
                            link4 = 'fetch_meshes/shoulder_pan_link', color4 = (0.8,0.8,0.8,1),
                            link5 = 'fetch_meshes/shoulder_lift_link', color5 = (0.8,0.8,0.8,1),
                            link6 = 'fetch_meshes/upperarm_roll_link', color6 = (0.8,0.8,0.8,1),
                            link7 = 'fetch_meshes/elbow_flex_link',color7 = (0.8,0.8,0.8,1),
                            link8 = 'fetch_meshes/forearm_roll_link',color8 = (0.8,0.8,0.8,1),
                            link9 = 'fetch_meshes/wrist_flex_link',color9 = (0.8,0.8,0.8,1),
                            link10 = 'fetch_meshes/wrist_roll_link',color10 = (0.8,0.8,0.8,1)
                            )
        self.loadModel()
    
    def DHParams(self):
        '''
        TODO: Find the appropriate DH Params for the Fetch Robot 
        '''
        links = [rtb.PrismaticDH(theta= 0, a= 0, alpha= -pi/2, qlim= [-0.8, 0])]    # Prismatic Link
        # print (links[0])
       # Combined DH parameters
        a = [0, -0.24365, -0.21325, 0, 0, 0]
        d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
        alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]
        qlim = [[-2*pi, 2*pi] for _ in range(10)]
        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i])
            links.append(link)
        return links
    

    def loadModel(self):
        '''
        Loads model into Swift simulation
        based off ir_support/robots:
            - LinearUR5
            - UR3
        '''
                # A joint config and the 3D object transforms to match that config
        qtest = [1.3200, 1.3999, -0.1998, 1.7199, 0.0, 1.6600, 0.0]
        qtest_transforms = [spb.transl(0, 0, 0) @ spb.trotx(-pi/2), # Base
                            spb.transl(0.001291, 0.187380, 0.055325) @ spb.trotx(-pi/2), # Torst Lift
                            spb.transl(0,-0.374760,0)  @ spb.trotx(-pi/2), # Head Pan
                            spb.transl(-0.086875,0.0187380,0.322104)  @ spb.trotx(-pi/2),  # Head Tilt  
                            spb.transl(0,0,-0.00664)  @ spb.trotx(-pi/2), # Shoulder Pan
                            spb.transl(0.053125,0,0.609641)  @ spb.trotx(-pi/2), #Shoulder Lift
                            spb.transl(0.14253, 0, 0.05799)  @ spb.trotx(-pi/2), # Upper Arm Roll
                            spb.transl(-0.076129,0,-0.31242)  @ spb.trotx(-pi/2), # Elbow Flex
                            spb.transl(-0.264823, 0.11334, 0.04)  @ spb.trotx(-pi/2), # Forearm Roll
                            spb.transl(0.000389,0.60902,-0.11253)  @ spb.trotx(-pi/2), # Wrist Flex
                            spb.transl(0, 0, 0)  @ spb.trotx(-pi/2), # Wrist Roll
                            ]     
           
        current_path = os.path.abspath(os.path.dirname(__file__))
        links= self.DHParams()
        # print(len(links))
        super().__init__(links, self.link3D_names, name = 'LinearUR3', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2)
        self.q = qtest
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)
        self.q = self._qtest        
        self.add_to_env(env)
        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        q_goal[0] = -0.8 # Move the rail link
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        fig = self.plot(self.q, limits= [-1,1,-1,1,-1,1])
        fig._add_teach_panel(self, self.q)

        time.sleep(3)
        for q in qtraj:
            self.q = q
            env.step(0.02)
            fig.step(0.01)
        fig.hold()
        env.hold()
        # time.sleep(3)    

if __name__ == "__main__":  
    r = FetchRobot()
    r.test()


    