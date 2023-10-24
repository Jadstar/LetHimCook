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
    def __init__(self, env):
        self.env = env
        self.robot = Fetch()
        self.camera = FetchCamera()
        self.spatula_mount = ''
        self.spatula = ''
        patty_position = (0, 0, 0)
        self.patty = Patty(patty_position, self.env)  # Provide the position and environment when creating the Patty


        # Initialization
        self.patty_flipped = False
        self.patty_is_cooked = False

    def CookRobotMove(self,coords,rot_angle,axis):
        '''
        Very Similar to how SE3 works but makes sure that everything moves with each other
        (i.e. arm, camera and spatula all move together)
        '''
        if coords is not None:
            self.robot.base = SE3(coords)
            self.camera = SE3(coords)
            self.spatula = SE3(coords)
            self.spatula_mount = SE3(coords)
        

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
