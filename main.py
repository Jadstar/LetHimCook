'''

 THIS IS WHERE THE THE ROBOTS WILL DO THE MAJORITY OF THEIR POSES AND SIMULATE PROCCESS

 
'''
import swift
import roboticstoolbox as rtb
from cooking import CookingRobot, FLIP_TEMP, DONE_TEMP, TIME_STEP

def main():
    # Initialize the simulation environment
    env = swift.Swift()
    env.launch(realtime=True)

    # Load the robot and cooking items
    robot = CookingRobot(env)

    # Start the cooking process
    robot.cook()

if __name__ == "__main__":
    main()
