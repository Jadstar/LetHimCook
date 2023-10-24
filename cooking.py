'''
THIS IS WHERE THE TESTING AND USAGE OF THE FETCH ROBOT FLIPPING BURGERS WILL BE DONE BEFORE PUSHING INTO main.py

'''
import swift
import roboticstoolbox as rtb
from fetch_robot import Fetch
from FetchCam import FetchCamera
from spatialmath import SE3
import os
import spatialgeometry as geometry
import numpy as np
from math import pi

ROOM_TEMP = 20  # in Celsius
FLIP_TEMP = 70  # temperature to flip the patty
DONE_TEMP = 90  # temperature the patty is fully cooked
TIME_STEP = 0.02  # simulation time step


patty_position = (0,1,0)
above_patty = (0,1,.5)
plate_position = (0,2,0)
above_plate = (0,2,0.5)

class Patty:
    def __init__(self, position, env):
        '''
        Initializes the Patty in the simulation environment.
        
        position: A tuple (x, y, z) indicating where the patty should be placed.
        env: The simulation environment.
        '''
        self.position = position
        self.env = env

        # Initialize patty temperature to room temperature
        self.temperature = ROOM_TEMP
        
        # Determine color based on temperature
        self.is_flipped = False
        self.color = self.get_color(self.temperature)
        # Load the patty into the simulation at the given position
        self.mesh = geometry.Mesh('assets/BurgerPatty.stl', pose=SE3(position), color=self.color)
        self.env.add(self.mesh)
        self.update_color()

    def heat(self, delta_temp):
        '''
        Increase the temperature of the patty and update its color accordingly.

        delta_temp: Amount of temperature increase.
        '''
        self.temperature += delta_temp
        self.update_color()

    def get_color(self, temp):
        '''
        Determines the color of the patty based on its temperature.
        
        temp: The current temperature of the patty.
        
        Returns: A tuple representing the RGB color.
        '''
        # Use linear interpolation to transition from pink (uncooked) to brown (cooked)
        # based on the temperature relative to the flipping and done temperatures.
        if temp < FLIP_TEMP:
            fraction_cooked = (temp - ROOM_TEMP) / (FLIP_TEMP - ROOM_TEMP)
        else:
            fraction_cooked = 1 + (temp - FLIP_TEMP) / (DONE_TEMP - FLIP_TEMP)
        
        # Pink color
        pink = np.array([255, 182, 193])
        # Brown color
        brown = np.array([139, 69, 19])

        # Interpolate between pink and brown
        color = pink + fraction_cooked * (brown - pink)
        clamped_color = np.clip(color, 0, 255)

        # Return as a tuple normalized to [0, 1]
        return tuple(clamped_color / 255.0)

    def update_color(self):
        '''
        Updates the color of the patty mesh in the environment based on its temperature.
        '''
        # Calculate the new color based on temperature
        new_color = self.get_color(self.temperature)
        self.mesh.color = new_color
        
        # Re-add the patty to the environment to force a visual update
        self.env.remove(self.mesh)
        self.mesh = geometry.Mesh('assets/BurgerPatty.stl', pose=SE3(self.position), scale=[0.005, 0.005, 0.005], color=new_color)
        self.env.add(self.mesh)
        

class CookingRobot:
    '''
    This class combines the components that will make up the Fetch Robot Including:

    Fetch Robot - The Robotic Arm
    Fetch Camera - For sensing
    Spatula Mount - To mount the Spatula to the Robot
    Spatula - To Flip the Burgers

    '''
    def __init__(self):
     
        self.robot = Fetch()
        self.camera = FetchCamera()
        spatula_mount_path = 'fetch_robotdata/accessories/SpatulaMount.STL'
        spatula_path = 'fetch_robotdata/accessories/Spatula.stl'

        #Moving Spatula to correct place
        self.spatula_offset = SE3(0,0,-0.07)
        self.mount_offset = SE3(-0.085,0,0)

        spatula_pose = self.robot.fkine(self.robot.q).A * SE3.Rz(pi/2) * self.spatula_offset
        spatula_mount_pose = self.robot.fkine(self.robot.q).A * self.mount_offset
        
        #Creating Meshes
        self.spatula = geometry.Mesh(spatula_path, pose =spatula_pose,scale=[0.01,0.01,0.01])
        self.spatula_mount = geometry.Mesh(spatula_mount_path, pose =spatula_mount_pose, scale=[0.001,0.001,0.001])

        #Adding to Swift environment

        # Initialization
        self.patty_flipped = False
        self.patty_is_cooked = False

    def fetch_fkine(self,q):
        '''
        Calculate the forward kinematics of robot as the last link is not the exact end effector
        '''
        
    def AddtoEnv(self,env):
        env.add(self.robot)
        env.add(self.camera)
        env.add(self.spatula)
        env.add(self.spatula_mount)

    def MotionTest(self):
        '''
        Testing the motion of components together
        '''
        env = swift.Swift()
        env.launch(realtime= True)
        self.AddtoEnv(env)

        goal_test = SE3(1,1,1)

        q_goal = self.robot.ikine_LM(goal_test,joint_limits=True).q
        # q_goal = [-pi/3, -pi/3,  -pi/3,  0 , 0,  0, -1.06027914,  1.42696591,  1.70241279, -0.68058669]
        qtraj = rtb.jtraj(self.robot.q, q_goal,50).q

        for q in qtraj:
            self.CookMove(q)
            env.step(0.02)
        env.hold()


    def CookMove(self,q):
        '''
        Very Similar to how SE3 works but makes sure that everything moves with each other
        (i.e. arm, camera and spatula all move together)

        For some reason the prismatic torso lift is allowed to have a q value < 0 even though it has limit on it??
        Specify that q[3] has to be always greater than 0 and less than limit to get correct end effector
        '''
        self.robot.q = q 
        if q[2] <0 or q[2] > 0.38615:
            print("invalid q values")
            if q[2] < 0 :
                self.robot.q[2] = 0
            else:
                self.robot.q[2] = 0.38615
                
        tr = self.robot.fkine(self.robot.q).A
        #moving Gripper as well
        # print(tr)
        # print(q)
        self.spatula.T = tr *  SE3.Rz(pi/2) * self.spatula_offset
        self.spatula_mount._T = tr * self.mount_offset
        self.camera.q[:3] = self.robot.q[:3]            # FetchCamera only has 5 links so only uses that are necesary
    

    def locatePatty(self,patty):
        '''
        Finds the pose of the patty to be locked into the flip
        '''


    def flip_patty(self, location):
        # 1. Move gripper above the patty
        q_above_patty = self.robot.ik(above_patty)  # This should be a position above the patty
        self.robot.q = q_above_patty
        
        # 2. Descend to grip the patty
        q_grip_patty = self.robot.ik(patty_position)  # This should be a position to grip the patty
        self.robot.q = q_grip_patty
        
        # 3. Lift it slightly
        self.robot.q = q_above_patty
        
        # 4. Rotate the wrist (or suitable joint) to flip
        q_flipped = q_above_patty.copy()
        q_flipped[-2] += 3.14  # Assuming the second last joint controls the wrist rotation
        self.robot.q = q_flipped
        
        # 5. Lower the patty back
        self.robot.q = q_grip_patty

        # 6. Return to initial pose or suitable rest pose
        self.robot.q = q_above_patty

    def move_to_plate(self):
        # 1. Move gripper above the patty
        q_above_patty = self.robot.ik(above_patty)
        self.robot.q = q_above_patty
        
        # 2. Descend to grip the patty
        q_grip_patty = self.robot.ik(patty_position)
        self.robot.q = q_grip_patty
        self.robot.gripper.close()
        
        # 3. Lift and navigate to above the plate
        q_above_plate = self.robot.ik(plate_position)  # Above the plate
        self.robot.q = q_above_plate
        
        # 4. Lower the patty onto the plate
        q_on_plate = self.robot.ik(above_plate)  # Position on the plate
        self.robot.q = q_on_plate
        self.robot.gripper.open()
        
        # 5. Return to initial pose or suitable rest pose
        self.robot.q = q_above_plate

    def cook(self):
        while not self.patty_is_cooked:
            # Simulate cooking
            self.patty.heat(0.5)  # This updates the patty's temperature and color

            # Check if it's time to flip the patty
            if self.patty.temperature >= FLIP_TEMP and not self.patty_flipped:
                self.flip_patty()
                self.patty_flipped = True

            # Check if patty is cooked
            if self.patty_flipped and self.patty.temperature >= DONE_TEMP:
                self.move_to_plate()
                self.patty_is_cooked = True

            self.env.step(TIME_STEP)



if __name__ == "__main__":
    # env = swift.Swift()
    # env.launch(realtime= True)
    r = CookingRobot()
    r.MotionTest()
