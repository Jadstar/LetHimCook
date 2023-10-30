import swift
import roboticstoolbox as rtb
from spatialmath import SE3
import spatialgeometry
from ir_support import UR5, UR3
from ir_support import *
from math import pi
from LinearUR3.LinearUR3 import LinearUR3
from spatialmath.base import *
from roboticstoolbox import models, jtraj, trapezoidal
import numpy as np
import time
import matplotlib.pyplot as plt


operation = 1 #0 - brick stacking, 1 - reach/ volume simulation
debugMode = 0 #0 basic, 1 developer

tableHeight = 0.6921
steps = 50
gripperDegrees = 14

r1 = LinearUR3()
#qStart = np.zeros([1,7])
qStart = [0,0,-pi/2,0,0,0,0]
r1.q = qStart

r1.base = r1.base * SE3(0,tableHeight,0)

# Create a Swift environment an add them into


brickFilename = "C:\\Users\\Owner\\OneDrive - UTS\\Industrial Robotics\\Assignment1\\HalfSizedRedGreenBrick.stl"

gripperBaseFilename = "C:\\Users\\Owner\\OneDrive - UTS\\Industrial Robotics\\Assignment1\\GripperBase.stl"
gripperFinger1Filename = "C:\\Users\\Owner\\OneDrive - UTS\\Industrial Robotics\\Assignment1\\GripperFinger.stl"
gripperFinger2Filename = "C:\\Users\\Owner\\OneDrive - UTS\\Industrial Robotics\\Assignment1\\GripperFinger2.stl"
gripperFinger3Filename = "C:\\Users\\Owner\\OneDrive - UTS\\Industrial Robotics\\Assignment1\\GripperFinger3.stl"

tableFilename = "C:\\Users\\Owner\\OneDrive - UTS\\Industrial Robotics\\Assignment1\\Table.stl"
barrierFilename = "C:\\Users\\Owner\\OneDrive - UTS\\Industrial Robotics\\Assignment1\\barrier.stl"
wallFilename = "C:\\Users\\Owner\\OneDrive - UTS\\Industrial Robotics\\Assignment1\\Wall.stl"
extinguisherFilename = "C:\\Users\\Owner\\OneDrive - UTS\\Industrial Robotics\\Assignment1\\Extinguisher.stl"
emergencyButtonFilename = "C:\\Users\\Owner\\OneDrive - UTS\\Industrial Robotics\\Assignment1\\EmergencyButton.stl"

secondTableFilename = "C:\\Users\\Owner\\OneDrive - UTS\\Industrial Robotics\\Assignment1\\secondTable.dae"



brickStartPos = [
    [0,-0.5,tableHeight],
    [-0.1,-0.5,tableHeight],
    [-0.2,-0.5,tableHeight],
    [-0.3,-0.5,tableHeight],
    [-0.4,-0.5,tableHeight],
    [-0.5,-0.5,tableHeight],
    [-0.6,-0.5,tableHeight],
    [-0.7,-0.5,tableHeight],
    [-0.8,-0.5,tableHeight]
]

brickEndPos = [
    [0,0.5,tableHeight],
    [-0.135,0.5,tableHeight],
    [-0.27,0.5,tableHeight],
    [0,0.5,tableHeight+0.035],
    [-0.135,0.5,tableHeight+0.035],
    [-0.27,0.5,tableHeight+0.035],
    [0,0.5,tableHeight+0.07],
    [-0.135,0.5,tableHeight+0.07],
    [-0.27,0.5,tableHeight+0.07]
]

bricks = []



# gripperModelScale = 0.0005
# gripper = spatialgeometry.Mesh(gripperFilename, scale=(gripperModelScale,gripperModelScale,gripperModelScale))

# gripper.T=r1.fkine(qStart).A @ SE3.Rx(pi/2).A
# env.add(gripper)

gripperScale = (0.65,0.65,0.65)
gripperBase = spatialgeometry.Mesh(gripperBaseFilename, scale=gripperScale)
gripperFinger1 = spatialgeometry.Mesh(gripperFinger1Filename, scale=gripperScale)
gripperFinger2 = spatialgeometry.Mesh(gripperFinger2Filename, scale=gripperScale)
gripperFinger3 = spatialgeometry.Mesh(gripperFinger3Filename, scale=gripperScale)


fkine = r1.fkine(qStart).A @ SE3.Rx(pi/2).A
gripperBase.T = fkine
gripperFinger1.T = fkine
gripperFinger2.T = fkine
gripperFinger3.T = fkine



table = spatialgeometry.Mesh(tableFilename, scale=(0.015,0.015,0.015), pose=SE3(-0.3,0,0))
table.color = [0,0.5,0.5,1]

barrier1 = spatialgeometry.Mesh(barrierFilename, scale=(0.0035,0.0035,0.0035), pose=SE3(-0.3,1.2,0))
barrier2 = spatialgeometry.Mesh(barrierFilename, scale=(0.0035,0.0035,0.0035), pose=SE3(-0.3,-1.2,0))
barrier3 = spatialgeometry.Mesh(barrierFilename, scale=(0.0035,0.0035,0.0035), pose=(SE3(-1.6,0,0) @ SE3.Rz(pi/2)))
barrier4 = spatialgeometry.Mesh(barrierFilename, scale=(0.0035,0.0035,0.0035), pose=(SE3(1,0,0) @ SE3.Rz(pi/2)))

#wall = spatialgeometry.Mesh(wallFilename, scale=(1,1,1), pose=(SE3(1,0,0)))
extinguisher = spatialgeometry.Mesh(extinguisherFilename, scale=(1,1,1), pose=(SE3(2.7,2.3,0.45) @ SE3.Rx(pi/2) ))

emergencyButton = spatialgeometry.Mesh(emergencyButtonFilename, scale=(0.0005,0.0005,0.0005), pose=(SE3(2.3,2.3,0.55) @ SE3.Ry(pi/2)))
emergencyButton.color = [1.0,0,0,1.0]

secondTable = spatialgeometry.Mesh(secondTableFilename, scale=(0.01, 0.01, 0.01), pose=(SE3(2.5,2.5,0)))
secondTable.color = [1.0,0,0,1.0]


if(operation == 0):

    env = swift.Swift()
    env.launch(realtime = True)

    for brick in brickStartPos:
        myobj = spatialgeometry.Mesh(brickFilename, pose = SE3(brick), scale=(1,1,1))
        myobj.color = [0.8,0.8,0,1]

        bricks.append(myobj)
        env.add(myobj)

    r1.add_to_env(env)

    env.add(gripperBase)
    env.add(gripperFinger1)
    env.add(gripperFinger2)
    env.add(gripperFinger3)

    env.add(table)

    env.add(barrier1)
    env.add(barrier2)
    env.add(barrier3)
    env.add(barrier4)

    env.add(emergencyButton)
    env.add(secondTable)
    env.add(extinguisher)

    
    for i in range(9):

        print("Moving brick at index " + str(i) + "/8")

        #Move to  brick start
        T1 = transl(brickStartPos[i]) @ trotx(pi) @ transl(0,0,-0.13)
        q1 = r1.ikine_LM(T1, q0 = qStart, joint_limits=True).q

        qStart = q1

        q_matrix = rtb.jtraj(r1.q, q1, steps).q   

        fkine = r1.fkine(q1).A @ SE3.Rx(pi/2).A
        for q in q_matrix:
            r1.q = q

            #fkine = r1.fkine(q).A
            # gripper.T = fkine @ SE3.Rx(pi/2).A
            fkine = r1.fkine(q).A @ SE3.Rx(pi/2).A
            gripperBase.T = fkine
            gripperFinger1.T = fkine @ SE3.Rz(-pi/180).A 
            gripperFinger2.T = fkine @ SE3.Rz(pi/180).A 
            gripperFinger3.T = fkine @ SE3.Rz(-pi/180).A 

            # gripperFinger1.T = gripperFinger1.T @ SE3.Rz(-pi/180).A 
            # gripperFinger2.T = gripperFinger2.T @ SE3.Rz(pi/180).A
            # gripperFinger3.T = gripperFinger3.T @ SE3.Rz(-pi/180).A  


            env.step(0.05)

        print("Target: " + str(brickStartPos[i]))
        print("Brick grabbed at: ")
        if(debugMode == 0):
            print(str(fkine[0][3]) + " " + str(fkine[1][3]) + " " + str(fkine[2][3] - 0.13))
        if(debugMode == 1):
            print(str(fkine))
        
        print("")

        #Close Gripper
        # for j in range(gripperDegrees):
        #     gripperFinger1.T = gripperFinger1.T @ SE3.Rz(-pi/180).A 
        #     gripperFinger2.T = gripperFinger2.T @ SE3.Rz(pi/180).A
        #     gripperFinger3.T = gripperFinger3.T @ SE3.Rz(-pi/180).A  
        #     env.step(0.05)

        #Grab brick


        

        #Move to brick end
        T1 = transl(brickEndPos[i]) @ trotx(pi) @ trotz(pi/2) @ transl(0,0,-0.13)
        q1 = r1.ikine_LM(T1, q0 = qStart, joint_limits=True).q

        qStart = q1

        q_matrix = rtb.jtraj(r1.q, q1, steps).q   

        fkine = r1.fkine(q1).A @ SE3.Rx(pi/2).A
        for q in q_matrix:
            r1.q = q

            fkine = r1.fkine(q).A
            # gripper.T = fkine @ SE3.Rx(pi/2).A
            fkineForGripper = fkine @ SE3.Rx(pi/2).A
            gripperBase.T = fkineForGripper
            gripperFinger1.T = fkineForGripper @ SE3.Rz((-pi/180) * gripperDegrees).A
            gripperFinger2.T = fkineForGripper @ SE3.Rz((pi/180) * gripperDegrees).A
            gripperFinger3.T = fkineForGripper @ SE3.Rz((-pi/180) * gripperDegrees).A


            bricks[i].T = fkine @ transl(0,0,0.09)
            env.step(0.05)

        #Open gripper
        for j in range(gripperDegrees):
            gripperFinger1.T = gripperFinger1.T @ SE3.Rz(pi/180).A 
            gripperFinger2.T = gripperFinger2.T @ SE3.Rz(-pi/180).A
            gripperFinger3.T = gripperFinger3.T @ SE3.Rz(pi/180).A  
            env.step(0.05)

        print("Target: " + str(brickEndPos[i]))
        print("Brick placed at: x , y , z")
        if(debugMode == 0):
            print(str(fkine[0][3]) + " " + str(fkine[1][3]) + " " + str(fkine[2][3] - 0.13))
        if(debugMode == 1):
            print(str(fkine))

        print("")
    
    print("PROCESS COMPLETE")

        #Place brick
if(operation == 1):

    r1.q = [0,pi/2,0,0,pi*3/2,0,0]

    fig = plt.figure(1)
    fig = r1.plot(r1.q, fig=fig, limits= [-1.5,1,-1,1,tableHeight,tableHeight+1])
    ax = plt.gca()
    #fig._add_teach_panel(r1, r1.q)

    # q2 = r1.q
    # q2[0] = -0.7
    steps=15
    q3 = [joint for joint in r1.q]

    instructions = [
        [1, -pi/2],
        [2, -pi],
        [1, pi/2],
        [0, -0.8],
        [1, -pi/2],
        [2, 0],
        [1, pi/2],
        [0,-0.4],
        [1,0],
        [2, -pi]
    ]

    points =[]

    for task in instructions:

        print("Actuating joint q[" + str(task[0]) +"] by " + str(task[1]*180/pi) + " degrs")
        q3[task[0]] = task[1]


        qMatrix = rtb.jtraj(r1.q, q3, steps).q
        for q in qMatrix:
            r1.q = q

            ee_pos = transl(r1.fkine(q).A) # End-effector position 
            points.append([ee_pos[0],ee_pos[1],ee_pos[2]])

            ax.plot(ee_pos[0],ee_pos[1],ee_pos[2], color = 'red', marker = '.', markersize= 2)

            fig.step(0.1)

    xMax = max([point[0] for point in points])
    xMin = min([point[0] for point in points])
    xRange = xMax - xMin

    yMax = max([point[1] for point in points])
    yMin = min([point[1] for point in points])
    yRange = yMax - yMin

    zMax = max([point[2] for point in points])
    zMin = min([point[2] for point in points])
    zRange = zMax - zMin

    print("x-axis : Max / Min -> " + str(xMax) + " / " + str(xMin))
    print("y-axis : Max / Min -> " + str(yMax) + " / " + str(yMin))
    print("z-axis : Max / Min -> " + str(zMax) + " / " + str(zMin))
    print(" ")

    print("x diameter: " + str(xRange))
    print("y diameter: " + str(yRange))
    print("z diameter: " + str(zRange))
    print(" ")

    volumeApprox = xRange*((2/3)*zRange*yRange) #depth * cross-section area under parabola (2/3 * ab - where a is height, b is chord)
    print("Approximate volume is " + str(volumeApprox) + " m3")
    print("Estimated with depth * cross-sectional area under parabloid")

    fig.hold()
    env.hold()


# Create trajectories for three robots

# q1 = rtb.jtraj(r1.q, [0.2,0.2,0.2], steps).q
# q2 = rtb.jtraj(r1.q, [joint - pi/4 for joint in r2.q], steps).q
# q3 = rtb.jtraj(r3.q, [joint - 0.8 for joint in r3.q], steps).q



time.sleep(15)
env.hold()