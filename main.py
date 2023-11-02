import swift
import sys
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication, QDialog
from out_window import Ui_OutputDialog
import time
from cooking import CookingRobot,Patty, FLIP_TEMP, DONE_TEMP, TIME_STEP
import spatialgeometry as geometry
from PyQt5.QtWidgets import QApplication
from spatialmath import SE3
import threading
from math import pi
import random
from GUI import PattyVisualizer
import roboticstoolbox as rtb
from spatialgeometry import Cuboid
from spatialmath.base import transl
import numpy as np
from assembly import AssemblyRobot

# import mainwindow

NUM_OF_PATTIES = 4      #Default No of Patties
FLOOR_LVL = 0           #Where the floor starts (z axis) on the swift env
ESTOP = False
PICKUP_THRESHOLD = 0.05

class Ui_Dialog(QDialog):
    '''
    THIS IS THE CLASS TO RUN THE GUI OPENING TO THE ROBOT
    WHEN RUN, A MENU WILL OPEN WHICH WILL ALLOW USERS TO SCAN PATTIES INTO THE SWIFT ENVIRONMENT
    '''
    def __init__(self):
        super(Ui_Dialog, self).__init__()
        loadUi("mainwindow.ui", self)

        self.runButton.clicked.connect(self.runSlot)

        self._new_window = None
        # self.Videocapture_ = None

    def refreshAll(self):
        """
        Set the text of lineEdit once it's valid
        """
        # self.Videocapture_ = "0"

    @pyqtSlot()
    def runSlot(self):
        """
        Called when the user presses the Run button
        """
        print("Clicked Run")
        self.refreshAll()
        self.hide()  # hide the main window
        
        self.robotrun = threading.Thread(target=main)
        # robotrun.start()
        self.outputWindow_()  # Create and open new output window

        # robotrun.join()

    def outputWindow_(self):
        """
        Created new window for vidual output of the video in GUI
        """
        self._new_window = Ui_OutputDialog()
        self._new_window.patties_detected_signal.connect(self.uploadScannedPatties)
        self._new_window.emergency_button.connect(self.startstop)
        self._new_window.show()
    def uploadScannedPatties(self, cropped_imgs):
        global NUM_OF_PATTIES
        NUM_OF_PATTIES = len(cropped_imgs)
        print(f"Detected {NUM_OF_PATTIES} patties.")  # This will print the number of patties

    def startstop(self,value):
        global ESTOP
        ESTOP = value
        print("ESTOP")
        print(ESTOP)
        if ESTOP == False:
            try:
                # If robot hasnt started, start it
                self.robotrun.start()
            except:
                pass

        # self.robotrun.join()



#Where the floor starts (z axis) on the swift env
FLOOR_LVL = 0

platePose = SE3(1,1,0.5)
plate_translation = platePose.A[:3, 3]  # Get the translation from the 4x4 matrix
print(f"Plate Location: x={plate_translation[0]}, y={plate_translation[1]}, z={plate_translation[2]}")
    
    
def configEnviro(env,pattylist: list[Patty]):
    '''
    Implementing all environmental components in the env
    As well as placing the patties on the grill 
    '''
    #Adding Grill
    grill_path = 'assets/krustykrab.dae'
    grill_pose = SE3(-0.2,11.5,1.3)*SE3.Rz(-pi)
    grill = geometry.Mesh(grill_path,pose=grill_pose,scale=[5,5,5])
    env.add(grill)

    #Adding Patties on top of grill
    # print(grill_pose.A[1,3])

    patty_x_bounds = [-0.12,0.62]
    patty_y_bounds = [0.8,1.25 ]
    patty_z = 0.55

    # Create the bounds of the grill and have the patties pose be moved there
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
    
 


def main():
    global ESTOP
    print(f"ESTOP IS {ESTOP}")
    #Set up opening window
    # add assembly bench
  
    # Initialize the simulation environment
    env = swift.Swift()
    env.launch(realtime=True)
    

    tomatoSaucePath = 'assets/tomatoSauce.stl'
    # tomatoSaucePose = SE3(0.79, -0.8, 0.67) @ SE3.Rx(pi/2)
    tomatoSaucePose = SE3(1.2, 1, 0.5) @ SE3.Rx(pi/2)
    tomatoSauce = geometry.Mesh(tomatoSaucePath, base=tomatoSaucePose, scale=(0.001,0.001,0.001))
    tomatoSauce.color = (1.0,0,0,1.0)
    env.add(tomatoSauce)
    
    # Adding plate
    platePath = 'assets/dinnerPlate.stl'
    plate1 = geometry.Mesh(platePath, pose=platePose, scale=[0.001,0.001,0.001])
    # plate1.color = (1,1,1,1)
    env.add(plate1)

      #Generate a number of Patties on Grill
    pattylist = []
    for i in range(NUM_OF_PATTIES):
        patty = Patty()
        pattylist.append(patty)

    #Config Environment
    configEnviro(env,pattylist)
    print('Environment Configured')
    
    # Load the Cooking Robot
    robot = CookingRobot()
    robot.robot.q = robot.robot.qr  # Update the robot's internal state to reflect its current pose

    # print("++++++++++++++++++")
    # print(robot.robot.fkine(robot.robot.qr).A[:3, 3])
    # print(robot.robot.fkine(robot.robot.q).A[:3, 3])
    # print("++++++++++++++++++")

    robot.CookMove(robot.robot.qr)
    robot.AddtoEnv(env)

    assemblyRobot = AssemblyRobot()
    assemblyRobot.setPose(SE3(0.8, 0.3, 0.4))
    assemblyRobot.robot.add_to_env(env)
  
    shape = Cuboid(scale=[1,0.5,0.5])
    wall = Cuboid(scale=[20,9.85,10])
    wall.T = SE3(0.275,6.25,0)
    # print(shape.to_dict())
    shape.T = SE3(0.25,1.075,0.275)
    env.add(shape)
    env.add(wall)
    shape._added_to_swift =True

    pattyindex = 0
    idx= 0
    while idx < len(pattylist):
        while patty.temperature < FLIP_TEMP:
            patty.heat(1, env)
        if not ESTOP:
            find_and_flip = robot.flip_patty(pattylist[pattyindex])

            for i, q in enumerate(find_and_flip[0]):
                if not ESTOP:
                    robot.CookMove(q)
                    env.step(0.06)
                    robot.robot.q = q  # Update the robot's current configuration

            for i, q in enumerate(find_and_flip[1]):
                if not ESTOP:
                    robot.CookMove(q)
                    tr = robot.robot.fkine(q).A
                    # diff = np.linalg.norm(pattylist[pattyindex].getPose() - tr)
                    # if diff <= PICKUP_THRESHOLD:
                    pattylist[pattyindex].setPose(tr * robot.flipoffset)
                    env.step(0.06)

            for s in robot.PattyGravity(pattylist[pattyindex]):
                if not ESTOP:
                    pattylist[pattyindex].setPose(s)
                    env.step(0.01)

            pattyindex += 1

            idx += 1

    env.hold()

    for patty in pattylist:
        while patty.temperature < DONE_TEMP:
            patty.heat(1, env)
            # time.sleep(5)
        # input('ready for next')
        # First part of array finds patty, second part flips
        # Check for Collisions 
        find_and_flip = robot.move_to_plate(patty, platePose)
        
        print("++++++++++++++++++")
        print(robot.robot.fkine(robot.robot.q).A[:3, 3])
        print("++++++++++++++++++")
        for q in find_and_flip[0]:
            robot.CookMove(q)
            env.step(0.06)
            robot.robot.q = q


        # Perform the flipping action at the plate
        for q in find_and_flip[1]:
            robot.CookMove(q)
            tr = robot.robot.fkine(q).A
            patty.setPose(tr * robot.flipoffset)
            env.step(0.06)

        # Apply gravity effect to the patty if needed
        for s in robot.PattyGravity(patty):
            patty.setPose(s)
            env.step(0.01)

        assemblyQMatrix = assemblyRobot.move(mode='point', targetPoint=[0.22,0.58,0.2], targetRPY=[0,pi,0], t=2)
        for q in assemblyQMatrix:
            assemblyRobot.robot.q = q

            fkine = assemblyRobot.robot.fkine(q).A @ transl(0,-0.1,0)
            tomatoSauce.T = fkine

            env.step(0.05)

        assemblyQMatrix = assemblyRobot.move(mode='circle', t=1)

        for q in assemblyQMatrix:
            assemblyRobot.robot.q = q

            fkine = assemblyRobot.robot.fkine(q).A @ transl(0,-0.1,0)
            tomatoSauce.T = fkine

            env.step(0.05)


        assemblyQMatrix = assemblyRobot.move(mode='point', targetPoint=[0.6,0.6,0.3], targetRPY=[0,pi,0], t=1)

        for q in assemblyQMatrix:
            assemblyRobot.robot.q = q

            fkine = assemblyRobot.robot.fkine(q).A @ transl(0,-0.1,0)
            tomatoSauce.T = fkine

            env.step(0.05)

        # assemblyQMatrix = assemblyRobot.move(mode='point', targetPoint=[0.22,0.58,0.2], targetRPY=[0,pi,0], t=2)

        # for q in assemblyQMatrix:
        #     assemblyRobot.robot.q = q

        #     fkine = assemblyRobot.robot.fkine(q).A @ transl(0,-0.1,0)
        #     tomatoSauce.T = fkine

        #     env.step(0.05)
    env.hold()
    # sys.exit(app.exec_())

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = Ui_Dialog()
    ui.show()
    app.exec_()  # This will block until Ui_Dialog is closed
    ui.robotrun.join()
    # main()