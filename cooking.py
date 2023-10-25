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
    def __init__(self, position=SE3(0,0,0),env=None):
        '''
        Initializes the Patty in the simulation environment.
        
        position: in SE3 Format (x, y, z) indicating where the patty should be placed.
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
        self.scale = [0.0015, 0.0015, 0.0015]
        self.mesh = geometry.Mesh('assets/BurgerPatty.stl', pose=position, color=self.color,scale=self.scale)
        # self.update_color()

    def getPose(self):
        '''
        Returns Patty Pose
        '''
        return self.mesh.T
    
    def setPose(self,pose):
        '''
        Sets the position of patty, useful when it needs to be moved

        Position must be in SE3 format
        '''
        self.mesh.T = pose

    def AddtoEnv(self,env):
        env.add(self.mesh)

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
        self.mesh = geometry.Mesh('assets/BurgerPatty.stl', pose=SE3(self.position), scale=self.scale, color=new_color)
        self.AddtoEnv(self.env)

        

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
        #The offset to make sure the end effector is the spatula, not gripper
        self.patty_offset = SE3(-0.13,0,0.065)
        self.flipoffset = SE3(0.13,0,-0.05)    #So when flipping, the patty stays on the spatula

        spatula_pose = self.robot.fkine(self.robot.q).A * SE3.Rz(pi/2) * self.spatula_offset
        spatula_mount_pose = self.robot.fkine(self.robot.q).A * self.mount_offset
        
        #Creating Meshes
        self.spatula = geometry.Mesh(spatula_path, pose =spatula_pose,scale=[0.01,0.01,0.01])
        self.spatula_mount = geometry.Mesh(spatula_mount_path, pose =spatula_mount_pose, scale=[0.001,0.001,0.001])

        #Adding to Swift environment

        # Initialization
        self.patty_flipped = False
        self.patty_is_cooked = False

    def AddtoEnv(self,env):
        env.add(self.robot)
        env.add(self.camera)
        env.add(self.spatula)
        env.add(self.spatula_mount)


    def PattyFlipTest(self):
        '''
        Testing the robot flipping capabilities
        '''
        env = swift.Swift()
        env.launch(realtime= True)
        self.AddtoEnv(env)

        patty = Patty(env=env)
        patty.setPose(SE3(2,1,1))
        patty.AddtoEnv(env)

        goal_test = patty.getPose()

        #Flip Patty Function (first part of list is moving towards patty, next part is flipping)
        fullq = self.flip_patty(patty)
        # patty_offset = goal_test *pattyoffset 

        # q_goal = self.robot.ikine_LM(patty_offset,joint_limits=True).q
        # # q_goal = [-pi/3, -pi/3,  -pi/3,  0 , 0,  0, -1.06027914,  1.42696591,  1.70241279, -0.68058669]
        # qtraj = rtb.jtraj(self.robot.q, q_goal,50).q

        for q in fullq[0]:
            self.CookMove(q)
            # tr = self.robot.fkine(q).A
            # patty.setPose(tr * self.patty_offset)
            env.step(0.02)
            
        # Flipping Patty
        for q in fullq[1]:
            self.CookMove(q)
            tr = self.robot.fkine(q).A
            patty.setPose(tr * self.flipoffset)
            env.step(0.02)
        #Gravity
        for s in self.PattyGravity(patty):
            patty.setPose(s)
            env.step(0.02)

        env.hold()

    def PattyGravity(self,patty : type(Patty)):
        '''
        To simulate the patties falling back onto the grill, we use this fuction to move the patty slightly down to its original position after it was flipped
        Outputs a list of values to be used in env to fall down
        '''
        steps = 10
        currpose = patty.getPose()
        offset_z = self.flipoffset.A[2, 3]  # Assuming Z value is at row 2, column 2
        print(offset_z)
        # Create finalpose by copying all data from currpose
        finalpose = currpose.copy()

        # Modify the Z value in finalpose
        finalpose[2, 3] = currpose[2, 3] + offset_z

        print(currpose)
        print(finalpose)
        interpolated_poses=[]
        for i in range(steps):
            t = i / (steps - 1)  # Linear interpolation parameter

            # Create an interpolated pose by linearly interpolating the Z value
            interpolated_pose = currpose.copy()
            interpolated_pose[2, 3] = (1 - t) * currpose[2, 3] + t * finalpose[2, 3]

            interpolated_poses.append(interpolated_pose)

        return interpolated_poses
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
    

    def locatePatties(self):
        '''
        Uses Fetch Camera to Find the patties on the grill

        input: list of patties
        output: a list of coordinates where the patties are
        '''
        pattycoords = []

        return pattycoords


    def flip_patty(self, patty : type(Patty)):
        '''
        Creates a big list of joint values that move the spatula to the patty, and flip it over
        '''
        full_qlist = []
        location = patty.getPose()
        print(location)
        offset_location = location * self.patty_offset #Offset for the spatula to be in correct position to patty
        

        # 1. Move gripper above the patty
        q0 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        q=[0.9065, 1.229, 0.2164, -0.8978, -0.08897, -0.03925, -0.008564, -0.05373, 0.09794, 0.09256]
        q1 = self.robot.ikine_LM(offset_location,q0=q0, joint_limits=True)  # This should be a position above the patty
        if q1.success:
            print(q1.q)
            qtraj = rtb.jtraj(self.robot.q, q1.q,50).q
            full_qlist.append(qtraj)
        
        # 2. Flip patty, use q1 pose as initial value for ikine
        flip_loc = offset_location * SE3.Rx(pi) 
        
        #It is ideal to reduce robot movement to prevent collsions, so x,y values with be masked in ikine and replaced with the q1 cartesian values
        q2 = self.robot.ikine_LM(flip_loc,q0=q1.q,joint_limits=True,mask=[0,0,1,1,1,1])  # This should be a position above the patty
        q2.q[:2] = q1.q[:2]
        if q2.success:
            print(q2.q)
            qtraj = rtb.jtraj(q1.q, q2.q,50).q
            full_qlist.append(qtraj)

    
        # # 3. Lift it slightly
        # self.robot.q = q_above_patty
        
        # # 4. Rotate the wrist (or suitable joint) to flip
        # q_flipped = q_above_patty.copy()
        # q_flipped[-2] += 3.14  # Assuming the second last joint controls the wrist rotation
        # self.robot.q = q_flipped
        
        # # 5. Lower the patty back
        # self.robot.q = q_grip_patty

        # # 6. Return to initial pose or suitable rest pose
        # self.robot.q = q_above_patty


        return full_qlist
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
    # r.MotionTest()
    r.PattyFlipTest()
