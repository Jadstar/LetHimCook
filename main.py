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
from assembly import AssemblyRobot
import spatialgeometry as geometry
from PyQt5.QtWidgets import QApplication
from spatialmath import SE3
import threading
from math import pi
import random
from GUI import PattyVisualizer
import roboticstoolbox as rtb
from spatialgeometry import Cuboid
import detect
import mainwindow
NUM_OF_PATTIES = 4
class Ui_Dialog(QDialog):
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
        self.outputWindow_()  # Create and open new output window

    def outputWindow_(self):
        """
        Created new window for vidual output of the video in GUI
        """
        self._new_window = Ui_OutputDialog()
        self._new_window.patties_detected_signal.connect(self.handle_cropped_imgs)

        self._new_window.show()
    def handle_cropped_imgs(self, cropped_imgs):
        global NUM_OF_PATTIES
        NUM_OF_PATTIES = len(cropped_imgs)
        print(f"Detected {NUM_OF_PATTIES} patties.")  # This will print the number of patties
        # self._new_window.startVideo(self.Videocapture_)
        # print("Video Played")
#Sensor mode will use real picture to determine how many patties enter 


    # You can add any further processing code here
Image_check = '' # impage to check

# Run detect Code
# Get output of detect code 

#For length of detect code, add that many patties

#if you can do it, extract the x,y coords of the boxes, 
# find the center, and then thats ur center of ur patty
#then u can do the dist between patties to show how the poses work



#Determines how many patties will be on grill

#Where the floor starts (z axis) on the swift env
FLOOR_LVL = 0

def testpatty(patty, window, env):
    
    while True:
        # If emergency stop is not requested, continue cooking
        while not window.emergency_stop_requested and patty.temperature < DONE_TEMP:
            patty.heat(1)  # Increase temperature by 1 degree for testing
            print(f"Current Patty Temperature: {patty.temperature}°C")
            window.update_display(patty.temperature)
            time.sleep(0.2)

        # If the patty is done cooking, break out of the loop
        if patty.temperature >= DONE_TEMP:
            break

        # If emergency stop is requested, pause and wait for it to be unset
        while window.emergency_stop_requested:
            time.sleep(0.5)
            
def movingIntotest():
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
    robot.setPose(SE3(-20,-12,FLOOR_LVL))
    robot.CookMove(robot.robot.qr )
    robot.AddtoEnv(env)

    #Robot
    move_forward =SE3(0,-12,0)
    move1 = robot.moveToPos(move_forward)
    q1 = robot.robot.q
    qtraj = rtb.ctraj(robot.getPose(),move_forward,t=50)
    input()
    for q in qtraj:
        print(q)
        robot.setPose(q)
        env.step(0.05)
        # time.sleep(1)
    secondmove = SE3(0,-8,0) * SE3.Rz(pi/2)
    qtraj = rtb.ctraj(robot.getPose(),secondmove,t=50)
    

    for q in qtraj:
            print(q)
            robot.setPose(q)
            env.step(0.05)
    thirdmove =SE3(1.1,10,0)* SE3.Rz(pi/2)
    qtraj = rtb.ctraj(robot.getPose(),thirdmove,t=50)
    input()

    for q in qtraj:
            print(q)
            robot.setPose(q)
            env.step(0.1)
    env.hold()

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

    # benchPath = 'assets/workingBench.stl'
    # benchPose = SE3(-1.5,2,0)*SE3.Rx(pi/2)
    # bench = geometry.Mesh(benchPath, pose=benchPose, scale=[0.0005,0.0012,0.001])
    # # bench.color = (0.8,0.2,0.5,1)
    # env.add(bench)

    # platePath = 'assets/dinnerPlate.stl'
    # platePose = SE3(0,0,0)*SE3.Rx(pi/2)
    # plate1 = geometry.Mesh(platePath, pose=platePose, scale=[1,1,1])
    # plate1.color = (1,1,1,1)
    # env.add(plate1)



def testpatty(robot):
    # Test heating the patty and visualizing the color change
    while robot.patty.temperature < DONE_TEMP:
        robot.patty.heat(1)  # Increase temperature by 1 degree for testing
        print(f"Current Patty Temperature: {robot.patty.temperature}°C")  # Print the current temperature
        time.sleep(0.2)

def main():
    # Initialize the simulation environment
    
    # app = QApplication(mainwindow.sys.argv)
    # ui = Ui_Dialog()
    # ui.show()
    # # sys.exit(app.exec_())
    # app.exec_()  # This will block until Ui_Dialog is closed

    env = swift.Swift()
    env.launch(realtime=True)
    
    # patty = Patty(env=env)
    # patty.setPose(SE3(2, 1, 1))
    # patty.AddtoEnv(env)
      #Generate a number of Patties on Grill
    pattylist = []
    for i in range(NUM_OF_PATTIES):
        patty = Patty()
        pattylist.append(patty)
        print('>>>>>>>>>>>>>')
        print(patty)
    # Initialize the GUI for Patty Vis  ualization
    # app = QApplication([])
    # window = PattyVisualizer()
    # window.show()

    # app.exec_()  
    
    #Config Environment
    configEnviro(env,pattylist)
    print('configged enviro')
    # Load the robot and cooking items
    robot = CookingRobot()
    # robot.setPose(SE3(1.2,11.5,FLOOR_LVL) *SE3.Rz(-pi/2))
    robot.robot.q = robot.robot.qr  # Update the robot's internal state to reflect its current pose
    print("++++++++++++++++++")
    print(robot.robot.fkine(robot.robot.qr).A[:3, 3])
    print(robot.robot.fkine(robot.robot.q).A[:3, 3])
    print("++++++++++++++++++")
    robot.CookMove(robot.robot.qr)
    # assemblyRobot = AssemblyRobot()
    # assemblyRobot.setPose(SE3(-1.25, 10.3, 0.685))
    # assemblyRobot.robot.add_to_env(env)
    # assemblyRobot.robot.q = [0,-pi/2,pi/4,0,0,0]
    robot.AddtoEnv(env)
    shape = Cuboid(scale=[1.05,0.55,0.5])
    wall = Cuboid(scale=[20,9.85,10])
    wall.T = SE3(0.275,6.25,0)
    # print(shape.to_dict())
    shape.T = SE3(0.25,0.975,0.275)
    env.add(shape)
    env.add(wall)
    shape._added_to_swift =True
    for patty in pattylist:
        while patty.temperature < FLIP_TEMP:
            patty.heat(1, env)
            # time.sleep(5)
        # input('ready for next')
        # First part of array finds patty, second part flips
        # Check for Collisions 
        find_and_flip = robot.flip_patty(patty)
        
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
        for q in find_and_flip[0]:
             robot.CookMove(q)
             print("BELOWER")
            #  print(env.swift_objects)
             env.step(0.06)
             robot.robot.q = q  # Update the robot's current configuration to the last configuration in the trajectory

        for q in find_and_flip[1]:
            robot.CookMove(q)
            tr = robot.robot.fkine(q).A
            patty.setPose(tr * robot.flipoffset)
            
            env.step(0.06)
        for s in robot.PattyGravity(patty):
            patty.setPose(s)
            env.step(0.01)
        
    # Test the patty color change
    env.hold()
    # sys.exit(app.exec_())

if __name__ == "__main__":
    
    main()