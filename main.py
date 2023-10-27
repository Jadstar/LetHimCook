'''
 THIS IS WHERE THE THE ROBOTS WILL DO THE MAJORITY OF THEIR POSES AND SIMULATE PROCCESS
'''
import swift
import time
import roboticstoolbox as rtb
from cooking import CookingRobot,Patty, FLIP_TEMP, DONE_TEMP, TIME_STEP
import spatialgeometry as geometry
from spatialmath import SE3

#Determines how many patties will be on grill
NUM_OF_PATTIES =3 


def configEnviro(env,pattylist: list[Patty]):
    '''
    Implementing all environmental components in the env
    As well as placing the patties on the grill 
    '''
    #Adding Grill
    grill_path = 'assets/BBQ.stl'
    grill_pose = SE3(0,2,0)
    grill = geometry.Mesh(grill_path,pose=grill_pose,scale=[0.01,.01,.01])
    env.add(grill)

    #Adding Patties on top of grill
    print(grill_pose.A[1,3])
    patty_x_bounds = grill_pose.A[0,3]
    patty_y_bounds = grill_pose.A[1,3]
    patty_z = 0.9
    for patty in pattylist:
        patty.setPose(SE3(patty_x_bounds,patty_y_bounds,patty_z))
        patty.AddtoEnv(env)

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

    #Generate a number of Patties on Grill
    pattylist = []
    for i in range(NUM_OF_PATTIES):
        patty = Patty()
        pattylist.append(patty)

    #Config Environment
    configEnviro(env,pattylist)

    # Load the robot and cooking items
    robot = CookingRobot()
    robot.AddtoEnv(env)

    env.hold()
    # Test the patty color change
    testpatty(robot)

    # Start the cooking process
    robot.cook()

if __name__ == "__main__":
    main()