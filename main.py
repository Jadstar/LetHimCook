import swift
import time
from cooking import CookingRobot,Patty, FLIP_TEMP, DONE_TEMP, TIME_STEP
from assembly import AssemblyRobot, GripperRobot
import spatialgeometry as geometry
from PyQt5.QtWidgets import QApplication
from spatialmath import SE3
import threading
from math import pi
import random
from GUI import PattyVisualizer
from spatialmath.base import transl
import numpy as np
import roboticstoolbox as rtb

#Determines how many patties will be on grill
NUM_OF_PATTIES =12
#Where the floor starts (z axis) on the swift env
FLOOR_LVL = 0.2

def configEnviro(env,pattylist: list[Patty]):
    '''
    Implementing all environmental components in the env
    As well as placing the patties on the grill 
    '''
    #Adding Grill
    grill_path = 'assets/krustykrab.dae'
    grill_pose = SE3(-0.2,-11.5,1.5)
    grill = geometry.Mesh(grill_path,pose=grill_pose,scale=[5,5,5])
    env.add(grill)

    #Adding Patties on top of grill
    # print(grill_pose.A[1,3])

    patty_x_bounds = [-0.82, -0.02]
    patty_y_bounds = [10.25, 10.7]
    patty_z = 0.75

    # Number of patties to create

    x_step = 0.22
    y_step = 0.22

    current_x = patty_x_bounds[0]
    current_y = patty_y_bounds[0]

    for i, patty in enumerate(pattylist):
        if current_x > patty_x_bounds[1]:
            current_x = patty_x_bounds[0]
            current_y += y_step

        patty.setPose(SE3(current_x, current_y, patty_z))
        patty.AddtoEnv(env)
        current_x += x_step

    # add assembly bench

    benchPath = 'assets/workingBench.stl'
    benchPose = SE3(-1.8, 0.4, 0) @ SE3.Rx(pi/2)
    bench = geometry.Mesh(benchPath, base=benchPose, scale=(0.0005,0.00125,0.001))
    bench.color = (0.37,0.57,0.97,1)
    env.add(bench)



def main():
    # Initialize the simulation environment
    env = swift.Swift()
    env.launch(realtime=True)
   
      #Generate a number of Patties on Grill
    pattylist = []
    for i in range(NUM_OF_PATTIES):
        patty = Patty()
        pattylist.append(patty)
        
        
    # Load the robot and cooking items
    robot = CookingRobot()
    robot.AddtoEnv(env=env)
    
    # Initialize the GUI for Patty Visualization
    # app = QApplication([])
    # window = PattyVisualizer()
    # window.show()

    # Start the testpatty function in a separate thread
    #test_thread = threading.Thread(target=testpatty, args=(patty, window, env))
    #test_thread.start()

    # Start the GUI's main loop
    # app.exec_()  

    #Config Environment
    configEnviro(env,pattylist)

    # Load the robot and cooking items
    robot = CookingRobot()
    robot.setPose(SE3(0,11.5,FLOOR_LVL) *SE3.Rz(-pi/2))
    robot.CookMove(robot.robot.qr )
    robot.AddtoEnv(env)



    tomatoSaucePath = 'assets/tomatoSauce.stl'
    tomatoSaucePose = SE3(-1.6, -0.7, 0.67) @ SE3.Rx(pi/2)
    tomatoSauce = geometry.Mesh(tomatoSaucePath, base=tomatoSaucePose, scale=(0.001,0.001,0.001))
    tomatoSauce.color = (1.0,0,0,1.0)
    env.add(tomatoSauce)

    platePath = 'assets/dinnerPlate.stl'
    platePose = SE3(-1.4, -0.6, 0.7)
    plate1 = geometry.Mesh(platePath, base=platePose, scale=(0.0009,0.0009,0.0009))
    plate1.color = (1.0,1.0,1.0,1.0)
    env.add(plate1)

    assemblyRobot = AssemblyRobot()
    assemblyRobot.setPose(SE3(-1.39, -0.97, 0.685))
    assemblyRobot.robot.add_to_env(env)
    gripper = GripperRobot()
    gripper.palm_mesh.T = gripper.palm_mesh.T * SE3(0, 0, 0)
    gripper.finger1.base = gripper.palm_mesh.T
    gripper.finger2.base = gripper.palm_mesh.T
    gripper.finger3.base = gripper.palm_mesh.T
    gripper.addtoEnv(env)  # Add the gripper r
    repeats = 5 #how many times to move through the routine
    qtrajclosed = gripper.closedhandtraj()
    # Define gripper opening trajectory
    qtrajopen = gripper.openhandtraj()
    prev_q = AssemblyRobot.q 
    for n in range(repeats):
        assemblyRoutine = assemblyRobot.generateRoutine()
        for i in range(len(assemblyRoutine)):
            for q in assemblyRoutine[i]:
                assemblyRobot.robot.q = q
                end_effector_pose = assemblyRobot.fkine(prev_q)
                gripper.move(end_effector_pose.A)
                if(i==4 or i==5 or i==6): #move sauce at these steps

                    fkine = assemblyRobot.robot.fkine(q).A @ transl(0.1,0,0.038) @ SE3.Rz(pi/2).A
                    tomatoSauce.T = fkine

                if(i==0 or i==1 or i==2 or i==8): #move plate at these steps
                    fkine = assemblyRobot.robot.fkine(q).A @ transl(0,0,0.14) @ SE3.Ry(-pi/2).A
                    plate1.T = fkine

                env.step(0.05)



    input()
    for patty in pattylist:
        # First part of array finds patty, second part flips
        find_and_flip = robot.flip_patty(patty)

        for q in find_and_flip[0]:
             robot.CookMove(q)
             env.step(0.02)
        # for q in find_and_flip[1]:
        #     robot.CookMove(q)
        #     tr = robot.robot.fkine(q).A
        #     patty.setPose(tr * robot.flipoffset)

        #     env.step(0.02)
        #Gravity
        for s in robot.PattyGravity(patty):
            patty.setPose(s)
            env.step(0.02)
        
    # Test the patty color change
    testpatty(robot)
    env.hold()
    
    #test_thread.join()

if __name__ == "__main__":
    main()