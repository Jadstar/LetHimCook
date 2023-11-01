import swift
import sys
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication, QDialog
# import resource
# from model import Model
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
# import mainwindow

NUM_OF_PATTIES = 4      #Default No of Patties
FLOOR_LVL = 0           #Where the floor starts (z axis) on the swift env
ESTOP = False
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
    
    #Set up opening window

    # Initialize the simulation environment
    env = swift.Swift()
    env.launch(realtime=True)
    
    #Generate a number of Patties on Grill
    pattylist = []
    for i in range(NUM_OF_PATTIES):
        patty = Patty()
        pattylist.append(patty)
            
    # Initialize the GUI for Patty Visualization
    # app = QApplication([])
    # window = PattyVisualizer()
    # window.show()

    #Config Environment
    configEnviro(env,pattylist)
    print('Environment Configured')
    # Load the Cooking Robot
    robot = CookingRobot()
    robot.robot.q = robot.robot.qr  # Update the robot's internal state to reflect its current pose

    print("++++++++++++++++++")
    print(robot.robot.fkine(robot.robot.qr).A[:3, 3])
    print(robot.robot.fkine(robot.robot.q).A[:3, 3])
    print("++++++++++++++++++")

    robot.CookMove(robot.robot.qr)
    robot.AddtoEnv(env)

    # assemblyRobot = AssemblyRobot()
    # assemblyRobot.setPose(SE3(-1.25, 10.3, 0.685))
    # assemblyRobot.robot.add_to_env(env)
    # assemblyRobot.robot.q = [0,-pi/2,pi/4,0,0,0]
    # input('ready to flip')
    pattyindex = 0
    jointindex = 0
    findorflip_index = -1 # 0 if estopped in find, 1 if stopped in flip
    ESTOP_saved_index = [pattyindex,jointindex,findorflip_index]  #we use this to store the index to know where the estop stopped at
    for idx,patty in enumerate(pattylist):

        
        #TODO: Fix Temperature Patty
        # while patty.temperature < FLIP_TEMP:
        #     patty.heat(1, env)
        #     time.sleep(0.05)
        #     print(patty.temperature) # COLOUR CHANGING BROKEN


  

        # Check for Collisions 

        # shape = Cuboid(scale=[0.74,0.55,0.45],color=[0.1,0.1,0.1,0])
        # print(shape.to_dict())
        # shape.T = SE3(0.25,1.025,0.275)
        # env.add(shape)
        # # shape._added_to_swift =True
        # print(f"THIS IS THE SHAPE: {shape.fk_dict()}")
        # print(robot.robot.links[0].closest_point(shape)[0])
        # link0dist = robot.robot.links[2].closest_point(shape)[0]
        # if link0dist == None:
        #     link0dist = 0
        # while robot.robot.collided(shape=shape,q=find_and_flip,skip=True) and link0dist < 0.1:
        #     print("Collided With thing")
        #     find_and_flip = robot.flip_patty(patty)
        #     for i in robot.robot.links[2:]:
        #         print(i.closest_point(shape)[0])
        #         link0dist = robot.robot.links[0].closest_point(shape)[0]
        #         if link0dist == None:
        #             link0dist = 0

        print("++++++++++++++++++")
        print(robot.robot.fkine(robot.robot.q).A[:3, 3])
        print("++++++++++++++++++")

        #Checking if The estop was previously pressed
        tempFindidx = 0
        tempFlipidx = 0
        find_and_flip = robot.flip_patty(pattylist[pattyindex])

        if ESTOP_saved_index[2] != -1:
                  # First part of array finds patty, second part flips
            # start = time.time()
            # end = time.time()
            # elasped = end - start
            # print(f"Execution time: {elasped} seconds")

            findorflip_index = -1
            if ESTOP_saved_index[2] == 0:
                tempFindidx = jointindex
            else:
                tempFlipidx = 0

        for i,q in enumerate(find_and_flip[0][tempFindidx]):
            if ESTOP == False:
                robot.CookMove(q)
                env.step(0.06)
                robot.robot.q = q  # Update the robot's current configuration to the last configuration in the trajectory
            elif i > 0:
                # If estop is pressed, we save the index that it was about to do 
                findorflip_index = 0
                jointindex = find_and_flip[findorflip_index][i]
                pattyindex = idx

        for i,q in enumerate(find_and_flip[1][tempFlipidx]):
            if ESTOP == False:
                robot.CookMove(q)
                tr = robot.robot.fkine(q).A
                patty.setPose(tr * robot.flipoffset)
                env.step(0.06)
            elif i > 0 and findorflip_index != 0:
                # If estop is pressed, we save the index that it was about to do 
                findorflip_index = 1
                jointindex = find_and_flip[findorflip_index][i]
                pattyindex = idx

        for s in robot.PattyGravity(patty):
            patty.setPose(s)
            env.step(0.01)

        # IF E-STOP HAS BEEN PRESSED WE MUST START FROM SAME INDEX UNTIL ITS UNPRESSED
        ESTOP_saved_index =[pattyindex,jointindex,findorflip_index] 
    env.hold()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = Ui_Dialog()
    ui.show()
    app.exec_()  # This will block until Ui_Dialog is closed
    ui.robotrun.join()
    # main()