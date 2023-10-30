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
import math

ROOM_TEMP = 20  # in Celsius
FLIP_TEMP = 70  # temperature to flip the patty
DONE_TEMP = 90  # temperature the patty is fully cooked
TIME_STEP = 0.02  # simulation time step
import time
class Patty:
    def __init__(self, position=SE3(0,0,0)):
        '''
        Initializes the Patty in the simulation environment.
        
        position: in SE3 Format (x, y, z) indicating where the patty should be placed.
        env: The simulation environment.

        '''
        
        self.position = position
        # self.env = env
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

    def heat(self, delta_temp, env):
        '''
        Increase the temperature of the patty and update its color accordingly.

        delta_temp: Amount of temperature increase.
        '''
        self.temperature += delta_temp
        self.update_color(env)

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

    def update_color(self, env):
        '''
        Updates the color of the patty mesh in the environment based on its temperature.
        '''
        # Calculate the new color based on temperature
        new_color = self.get_color(self.temperature)
        self.mesh.color = new_color
        
        # Re-add the patty to the environment to force a visual update
        self.mesh = geometry.Mesh('assets/BurgerPatty.stl', pose=SE3(self.mesh.T), scale=self.scale, color=new_color)
        self.AddtoEnv(env)

        

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
    
    def getPose(self):
        return self.robot.base
    def setPose(self,position):
        '''
        Sets the position of patty, useful when it needs to be moved

        Position must be in SE3 format
        '''
        self.robot.base = position
        self.camera.base = position
        tr = self.robot.fkine(self.robot.q).A
        
        self.spatula.T = tr *  SE3.Rz(pi/2) * self.spatula_offset
        self.spatula_mount._T = tr * self.mount_offset
        self.camera.q[:3] = self.robot.q[:3]            # FetchCamera only has 5 links so only uses that are necesary

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
        patty.setPose(SE3(1,1,1))
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
        finalpose[2, 3] = currpose[2, 3] + offset_z + offset_z

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
    
    def moveToPos(self,position):
        '''
        Used to move the fetch robot around place. 
        Input: SE3 coordinates
        Returns a list of jtraj() values to be used in main
        '''
        q_goal = self.robot.ikine_LM(position,q0=self.robot.q,joint_limits=True,mask=[1,0,0,0,0,0])

        if q_goal.success:
            print(q_goal.q)
            qtraj = rtb.jtraj(self.robot.q, q_goal.q,50).q
        return qtraj
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


    def CookMove(self, target_q):
        '''
        Moves the robot instantly to the given joint configuration.
        Ensures that the arm, camera, and spatula all move together.
        Prints the initial and final positions.
        '''
        # Store the current joint configuration and compute its forward kinematics
        initial_q = self.robot.q
        initial_position = self.robot.fkine(initial_q).A[:3, 3]  # Extracting the translation part
        # print('cookmove RobotPos')
        # print(self.robot.fkine(initial_q).A[:3, 3])
        # Adjust joint values for the prismatic torso lift
        if target_q[2] < 0:
            target_q[2] = 0
        elif target_q[2] > 0.38615:
            target_q[2] = 0.38615
        
        # Set the robot's joint values to the provided configuration
        self.robot.q = target_q
        final_position = self.robot.fkine(target_q).A[:3, 3]  # Extracting the translation part
        
        # Update the positions of the spatula, spatula mount, and camera
        tr = self.robot.fkine(target_q).A
        self.spatula.T = tr * SE3.Rz(pi/2) * self.spatula_offset
        self.spatula_mount._T = tr * self.mount_offset
        self.camera.q[:3] = target_q[:3]  # Assuming the camera's joint values align with the robot's first 3 joints
        
        # Print the movement details
        # print(f"Moved from joint configuration: {initial_q} to {target_q}")
        # print(f"End effector moved from position: {initial_position} to {final_position}")

    def locatePatties(self):
        '''
        Uses Fetch Camera to Find the patties on the grill

        input: list of patties
        output: a list of coordinates where the patties are
        '''
        pattycoords = []

        return pattycoords

    def calculate_distance(self, robot, q, offset_location):
        calculated_position = robot.fkine(q).A[:3, 3]
        return np.linalg.norm(calculated_position - np.array([offset_location[0, 3], offset_location[1, 3], offset_location[2, 3]]))

    def check_trajectory_collision(self, robot, trajectory, boundary):
            '''
            Checks if any configuration in the trajectory causes a collision.
            
            robot: The robot object.
            trajectory: List of joint configurations.
            boundary: CollisionBoundary object.
            
            Returns True if collision is detected, False otherwise.
            '''
            for q in trajectory:
                # Convert the joint configuration to its Cartesian coordinate
                poses = self.robot.fkine_all(q).A
                for pose in poses:
                    position = (pose[0, 3], pose[1, 3], pose[2, 3])
                    
                    # Check for collisions with the boundary
                    if boundary.check_collision(position):
                        return True
            return False
    
    def flip_patty(self, patty):
        '''
        Creates a list of joint values to move the spatula to the patty and flip it over.
        This version also prints the global locations the robot will move between.
        It also checks for collisions using the predefined collision boundary.
        '''
        
        

        full_qlist = []

        # 1. Get Patty Location
        location = patty.getPose()
        print(f"Patty Location: x={location[0, 3]}, y={location[1, 3]}, z={location[2, 3]}")

        offset_location = location * self.patty_offset
        print(f"Patty Location: x={offset_location[0, 3]}, y={offset_location[1, 3]}, z={offset_location[2, 3]}")
        print(f"Initial Location: {self.robot.fkine(self.robot.q).A[:3, 3]}")

        q0=[1.62, -2.166, 0.08587, -0.4418, 0.007674, -0.5102, 0.1855, 1.453, -0.2802, -0.9157]
        q=[0.9065, 1.229, 0.2164, -0.8978, -0.08897, -0.03925, -0.008564, -0.05373, 0.09794, 0.09256]
        max_attempts = 100000  # Maximum number of attempts to find a valid IK solution.
        attempts = 0
        mindist = 10000
        mindist2 = 10000
        q1_successful = False
        q2_successful = False
        boundary = CollisionBoundary(x_bounds=[-0.12,0.62], y_bounds=[0.7,1.25], z_bounds=[0,0.54])

        while attempts < max_attempts:
            q1 = self.robot.ikine_LM(offset_location, mask=[0.5,0.5,1,1,1,1])
            qtraj_to_patty = rtb.jtraj(self.robot.q, q1.q, 50).q
            for qt in qtraj_to_patty:
                f1 = self.robot.fkine_all(qt)
                # for j in f1:
                    # print(j)s
                    
            # Convert joint values to Cartesian coordinates
            print("===================^")
            print(f"tried Moving to Patty: {self.robot.fkine(q1.q).A[:3, 3]}")
            print('===but distance===')
            print(self.calculate_distance(self.robot, q1.q, offset_location))
            print(q1)
            print("ATTEMPTING++++++++++++++++++++")
            print(attempts)

            if not self.check_trajectory_collision(self.robot, qtraj_to_patty, boundary):
                q1_successful = True
            if q1_successful:
                if mindist > self.calculate_distance(self.robot, q1.q, offset_location):
                    q0=q1.q
                    mindist = self.calculate_distance(self.robot, q1.q, offset_location)
                if q1.success and self.calculate_distance(self.robot, q1.q, offset_location) <= 0.01:
                    break
                attempts += 1
            print(q1.q)
        
       
        if q1.success:
            full_qlist.append(qtraj_to_patty)
            print(f"Moving to Patty: {self.robot.fkine(q1.q).A[:3, 3]}")

        # 3. Flip Patty
        flip_loc = offset_location * SE3.Rx(pi)
        attempts = 0
        q0 = q1.q
        while attempts < max_attempts:
            q2 = self.robot.ikine_LM(flip_loc,q0=q0, mask=[0.5,0.5,1,1,1,1])
            
            # Convert joint values to Cartesian coordinates
            print("===================^")
            print(f"tried 2 Moving to Patty: {self.robot.fkine(q2.q).A[:3, 3]}")
            print('===but 2 distance===')
            print(self.calculate_distance(self.robot, q2.q, flip_loc))
            print(q1)

            qtraj_to_patty = rtb.jtraj(q1.q, q2.q, 50).q
            if not self.check_trajectory_collision(self.robot, qtraj_to_patty, boundary):
                q2_successful = True
            if q2_successful:
                if mindist2 > self.calculate_distance(self.robot, q2.q, flip_loc):
                    q0=q2.q
                    mindist2 = self.calculate_distance(self.robot, q2.q, flip_loc)
                if q2.success and self.calculate_distance(self.robot, q2.q, flip_loc) <= 0.01:
                    break
                attempts += 1
            print(q2.q)
        
       
        if q2.success:
            full_qlist.append(qtraj_to_patty)
            print(f"Moving 2 to Patty: {self.robot.fkine(q2.q).A[:3, 3]}")

        return full_qlist

    # def move_to_plate(self):
    #     '''
    #     TODO: Fix this function 
    #     '''
    #     # 1. Move gripper above the patty
    #     q_above_patty = self.robot.ik(above_patty)
    #     self.robot.q = q_above_patty
        
    #     # 2. Descend to grip the patty
    #     q_grip_patty = self.robot.ik(patty_position)
    #     self.robot.q = q_grip_patty
    #     self.robot.gripper.close()
        
    #     # 3. Lift and navigate to above the plate
    #     q_above_plate = self.robot.ik(plate_position)  # Above the plate
    #     self.robot.q = q_above_plate
        
    #     # 4. Lower the patty onto the plate
    #     q_on_plate = self.robot.ik(above_plate)  # Position on the plate
    #     self.robot.q = q_on_plate
    #     self.robot.gripper.open()
        
    #     # 5. Return to initial pose or suitable rest pose
    #     self.robot.q = q_above_plate

    def cook(self, patty):
        while not self.patty_is_cooked:
            # Simulate cooking
            patty.heat(0.5)  # This updates the patty's temperature and color

            # Check if it's time to flip the patty
            if patty.temperature >= FLIP_TEMP and not self.patty_flipped:
                self.flip_patty()
                self.patty_flipped = True

            # Check if patty is cooked
            if self.patty_flipped and self.patty.temperature >= DONE_TEMP:
                self.move_to_plate()
                self.patty_is_cooked = True

            self.env.step(TIME_STEP)

class CollisionBoundary:
    def __init__(self, x_bounds, y_bounds, z_bounds):
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.z_bounds = z_bounds

    def check_collision(self, point):
        """
        Checks if the given point collides with the boundary.
        
        :param point: A 3-tuple (x, y, z)
        :return: True if there is a collision, False otherwise
        """
        x, y, z = point
        
        # Check if the point is outside the boundary on any axis
        if (x < self.x_bounds[0] or x > self.x_bounds[1] or
            y < self.y_bounds[0] or y > self.y_bounds[1] or
            z < self.z_bounds[0] or z > self.z_bounds[1]):
            print("============FALSE=============")
            print(self.x_bounds[0], " <-- ",x, " --> ", self.x_bounds[1])
            print(self.y_bounds[0], " <-- ",y, " --> ", self.y_bounds[1])
            print(self.z_bounds[0], " <-- ",z, " --> ", self.z_bounds[1])
            print("============FALSE=============")
            return False
        print("============TRUE=============")
        print(self.x_bounds[0], " <-- ",x, " --> ", self.x_bounds[1])
        print(self.y_bounds[0], " <-- ",y, " --> ", self.y_bounds[1])
        print(self.z_bounds[0], " <-- ",z, " --> ", self.z_bounds[1])
        print("============TRUE=============")
        return True

# if __name__ == "__main__":
    # env = swift.Swift()
    # env.launch(realtime= True)
    # r = CookingRobot()
    # # r.MotionTest()
    # r.PattyFlipTest()
    # # Initialize the collision boundary with the given bounds

    # Initialize the collision boundary with the given bounds
    

    # # Test the collision boundary
    # test_point = (0, 1.9, 0.2)
    # collision = boundary.check_collision(test_point)
    # print(collision)



