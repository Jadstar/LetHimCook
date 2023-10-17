'''
THIS IS WHERE THE TESTING AND USAGE OF THE FETCH ROBOT FLIPPING BURGERS WILL BE DONE BEFORE PUSHING INTO main.py

'''
import swift
import roboticstoolbox as rtb

ROOM_TEMP = 20  # in Celsius
FLIP_TEMP = 70  # temperature to flip the patty
DONE_TEMP = 90  # temperature the patty is fully cooked
TIME_STEP = 0.02  # simulation time step
patty_position = (0,1,0)
above_patty = (0,1,.5)
plate_position = (0,2,0)
above_plate = (0,2,0.5)
class CookingRobot:
    def __init__(self, env):
        self.env = env
        self.robot = rtb.models.Fetch()
        self.patty = self.env.load_stl("BurgerPatty.stl")
        self.plate = self.env.load_stl("Choppingboard.stl")

        # Initialization
        self.patty_temp = ROOM_TEMP
        self.patty_flipped = False
        self.patty_is_cooked = False

    def flip_patty(self):
        # 1. Move gripper above the patty
        q_above_patty = self.robot.ik(above_patty)  # This should be a position above the patty
        self.robot.q = q_above_patty
        
        # 2. Descend to grip the patty
        q_grip_patty = self.robot.ik(patty_position)  # This should be a position to grip the patty
        self.robot.q = q_grip_patty
        self.robot.gripper.close()  # Assuming a gripper close function
        
        # 3. Lift it slightly
        self.robot.q = q_above_patty
        
        # 4. Rotate the wrist (or suitable joint) to flip
        q_flipped = q_above_patty.copy()
        q_flipped[-2] += 3.14  # Assuming the second last joint controls the wrist rotation
        self.robot.q = q_flipped
        
        # 5. Lower the patty back
        self.robot.q = q_grip_patty
        self.robot.gripper.open()  # Release the patty

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
            self.patty_temp += 0.5  # This is a simple linear simulation for illustration

            # Check if it's time to flip the patty
            if self.patty_temp >= FLIP_TEMP and not self.patty_flipped:
                self.flip_patty()
                self.patty_flipped = True

            # Check if patty is cooked
            if self.patty_flipped and self.patty_temp >= DONE_TEMP:
                self.move_to_plate()
                self.patty_is_cooked = True

            self.env.step(TIME_STEP)
