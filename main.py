'''

 THIS IS WHERE THE THE ROBOTS WILL DO THE MAJORITY OF THEIR POSES AND SIMULATE PROCCESS

 
'''
import swift
import time
import roboticstoolbox as rtb
from cooking import CookingRobot, FLIP_TEMP, DONE_TEMP, TIME_STEP

def testpatty(robot):
    # Test heating the patty and visualizing the color change
    while robot.patty.temperature < DONE_TEMP:
        robot.patty.heat(1)  # Increase temperature by 1 degree for testing
        print(f"Current Patty Temperature: {robot.patty.temperature}°C")  # Print the current temperature
        time.sleep(0.2)
def main():
    # Initialize the simulation environment
    env = swift.Swift()
    env.launch(realtime=True)

    # Load the robot and cooking items
    robot = CookingRobot(env)
    
    # Test the patty color change
    testpatty(robot)

    # Start the cooking process
    robot.cook()

if __name__ == "__main__":
    main()