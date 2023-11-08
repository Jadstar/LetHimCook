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
import time

# import mainwindow

NUM_OF_PATTIES = 4      #Default No of Patties
FLOOR_LVL = 0           #Where the floor starts (z axis) on the swift env
ESTOP = False
RESET = True
PICKUP_THRESHOLD = 0.05
TEACH_Q_VALS= [0,0,0,0,0,0,0,0,0,0]

pattylist = []
env = swift.Swift()
lock = threading.Lock()
condition = threading.Condition()
assemblyBotWaiting = False
assemblyBotCarryingPatty = False
idx = 0 #used to track the current patty being moved the assemblyBOt

#could definitely be somehwere better, but needed to make global
tomatoSaucePath = 'assets/tomatoSauce.stl'
tomatoSaucePose = SE3(1.6, 0.7, 0.47) @ SE3.Rx(pi/2)
tomatoSauce = geometry.Mesh(tomatoSaucePath, base=tomatoSaucePose, scale=(0.001,0.001,0.001))
tomatoSauce.color = (1.0,0,0,1.0)

platePath = 'assets/dinnerPlate.stl'
platePose = SE3(1.4, 0.6, 0.5)
plate1 = geometry.Mesh(platePath, base=platePose, scale=(0.0009,0.0009,0.0009))
plate1.color = (1.0,1.0,1.0,1.0)

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
        self._new_window.robotlinks.connect(self.getTeachModelChanges)
        self._new_window.reset.connect(self.resetRobot)
        self._new_window.show()
    def uploadScannedPatties(self, cropped_imgs):
        global NUM_OF_PATTIES
        NUM_OF_PATTIES = len(cropped_imgs)
        print(f"Detected {NUM_OF_PATTIES} patties.")  # This will print the number of patties

    def getTeachModelChanges(self,linkvalue):
        global TEACH_Q_VALS
        TEACH_Q_VALS[linkvalue[1]] = linkvalue[0]
        # print(TEACH_Q_VALS)
        
    def resetRobot(self,value):
        global RESET
        global TEACH_Q_VALS
        RESET = value
        TEACH_Q_VALS = [0,0,0,0,0,0,0,0,0,0]
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
    
    barrierPath = 'assets/barrier.stl'
    barrierPose = SE3(-1.4, 2.5, 0) *SE3.Rz(-pi)
    barrier = geometry.Mesh(barrierPath, base=barrierPose, scale=(0.0035,0.0035,0.0035))
    barrier.color = (1.0,1.0,1.0,1.0)
    env.add(barrier)
 
    buttonPath = 'assets/EmergencyButton.stl'
    buttonPose = SE3(-1.45, 1.9, 0.1)*SE3.Ry(pi/2)
    button = geometry.Mesh(buttonPath, base=buttonPose, scale=(0.0005,0.0005,0.0005))
    button.color = (0.0,1.0,0.0,1.0)
    env.add(button)
 
    extinguisherPath = 'assets/extinguisher.stl'
    extinguisherPose = SE3(-1.4, 1.8, 0) *SE3.Rx(pi/2)
    extinguisher = geometry.Mesh(extinguisherPath, base=extinguisherPose, scale=(1.0,1.0,1.0))
    extinguisher.color = (1.0,0.0,0.0,1.0)
    env.add(extinguisher)

    benchpath = 'assets/workingBench.stl'
    benchpose = SE3(0.8, 0.3, 0) *SE3.Rx(pi/2) *SE3.Ry(pi/2)
    bench = geometry.Mesh(benchpath, base=benchpose, scale=(0.0009,0.0009,0.0009))
    env.add(bench)


 
    # sprinklerPath = 'assets/sprinkler.stl'
    # sprinklerPose = SE3(-1.4, -0.6, 0.7)
    # sprinkler = geometry.Mesh(sprinklerPath, base=sprinklerPose, scale=(1.0,1.0,1.0))
    # sprinkler.color = (1.0,0.0,0.0,1.0)
    # env.add(sprinkler)

def CookingRobotWorkflow(robot):
    global idx
    global assemblyBotCarryingPatty
    while(True):

        #pattyindex=0
        for pattyindex in range(idx, len(pattylist)):


                while RESET:
                    print(TEACH_Q_VALS)
                    robot.CookMove(TEACH_Q_VALS)
                    stepEnvironment()
                    robot.robot.q = q 

                    idx = 0


                while pattylist[pattyindex].temperature < FLIP_TEMP:
                    pattylist[pattyindex].heat(1, env)

                pattylist[pattyindex].update_color(env)

                if not ESTOP:
                    find_and_flip = robot.flip_patty(pattylist[pattyindex])

                    for i, q in enumerate(find_and_flip[0]):
                        if not ESTOP:
                            robot.CookMove(q)
                            robot.robot.q = q  # Update the robot's current configuration
                            stepEnvironment(0.06)

                    for i, q in enumerate(find_and_flip[1]):
                        if not ESTOP:
                            robot.CookMove(q)
                            tr = robot.robot.fkine(q).A
                            # diff = np.linalg.norm(pattylist[pattyindex].getPose() - tr)
                            # if diff <= PICKUP_THRESHOLD:
                            pattylist[pattyindex].setPose(tr * robot.flipoffset)
                            stepEnvironment(0.06)

                    for s in robot.PattyGravity(pattylist[pattyindex]):
                        if not ESTOP:
                            pattylist[pattyindex].setPose(s)
                            stepEnvironment(0.01)

                    pattyindex += 1

                global assemblyBotWaiting
                if(assemblyBotWaiting):
                    while pattylist[idx].temperature < DONE_TEMP:
                        pattylist[idx].heat(1, env)
                        pattylist[idx].update_color(env)
                        # time.sleep(5)
                    # input('ready for next')
                    # First part of array finds patty, second part flips
                    # Check for Collisions 
                    movetoplate = robot.move_to_plate(pattylist[idx], plate1.T)
                    
                    # print("++++++++++++++++++")
                    # print(robot.robot.fkine(robot.robot.q).A[:3, 3])
                    # print("++++++++++++++++++")

                    #Get Plate Ready
            
                    if not ESTOP:
                        for i,q in enumerate(movetoplate[0]):
                            if not ESTOP:
                                #assemblyRobot.robot.q  = assemblyRoutine[0][i]
                                robot.CookMove(q)
                                stepEnvironment(0.06)
                                robot.robot.q = q

                        # Perform the flipping action at the plate
                        for q in movetoplate[1]:
                            if not ESTOP:
                                robot.CookMove(q)
                                tr = robot.robot.fkine(q).A
                                pattylist[idx].setPose(tr * robot.flipoffset)
                                stepEnvironment(0.06)

                        # Apply gravity effect to the patty if needed
                        for s in robot.PattyGravity(pattylist[idx]): ####   Was PattyGravity(patty), check if PattyGravity(pattylist[idx]) is actually correct
                            if not ESTOP:
                                pattylist[idx].setPose(s)
                                stepEnvironment(0.01)

                        assemblyBotCarryingPatty = True
                        with condition:
                            condition.notify() #notify assemblyBot


def AssemblyRobotWorkflow(assemblyRobot):
    global assemblyBotCarryingPatty
    global idx

    IsFirstMovement = True
    while(True):
        assemblyRoutine = assemblyRobot.generateRoutine()
        for i in range(len(assemblyRoutine)):

            if(i==2): #holding out plate to cooking bot
                with condition:
                    global assemblyBotWaiting
                    assemblyBotWaiting = True
                    condition.wait()
                    assemblyBotWaiting = False


            for q in (assemblyRoutine[i]):
                if not ESTOP:
                    assemblyRobot.robot.q = q
                    if(i==4 or i==5 or i==6): #move sauce at these steps

                        fkine = assemblyRobot.robot.fkine(q).A @ transl(-0.1,0,0.038) @ SE3.Rz(pi/2).A @ SE3.Rx(pi).A
                        tomatoSauce.T = fkine
                        
                    if not IsFirstMovement:
                        if(i==0 or i==1 or i==2 or i==8): #move plate at these steps
                            fkine = assemblyRobot.robot.fkine(q).A @ transl(0,0,0.14) @ SE3.Ry(pi/2).A
                            #print(fkine)
                            plate1.T = fkine
                            
                            if(assemblyBotCarryingPatty):
                                
                                pattylist[idx].setPose(fkine)
                        


                    stepEnvironment(0.05)
                    time.sleep(0.05)
            IsFirstMovement = False

            if(i==8):

                assemblyBotCarryingPatty=False
                idx+=1


def stepEnvironment(t=0.05):
    if(lock.acquire(blocking=False)):
        try:
            env.step(t)
            pass
        finally:
            lock.release()
            pass
    else:
        print("skipping env step")


def EnvironmentThread():
    while(True):
        env.step(0.05)
        time.sleep(1)

def main():
    global ESTOP
    global TEACH_Q_VALS
    # print(f"ESTOP IS {ESTOP}")
    #Set up opening window
    # add assembly bench
  
    # Initialize the simulation environment

    env.launch(realtime=True)
    

    

    env.add(tomatoSauce)
    env.add(plate1)
      #Generate a number of Patties on Grill


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
    assemblyRobot.setPose(SE3(1.39,0.97 , 0.485))
    assemblyRobot.robot.add_to_env(env)

    cookingThread = threading.Thread(target=CookingRobotWorkflow, args=(robot,))
    assemblyThread = threading.Thread(target=AssemblyRobotWorkflow, args=(assemblyRobot,))
    envThread = threading.Thread(target=EnvironmentThread)

    #CookingRobotWorkflow(robot, env)    

    # Burgers are ready to be plated
    
    #AssemblyRobotWorkflow(assemblyRobot, env)

    cookingThread.start()
    assemblyThread.start()
    #envThread.start()

    cookingThread.join()
    assemblyThread.join()
    #envThread.join()
    

    env.hold()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = Ui_Dialog()
    ui.show()
    app.exec_()  # This will block until Ui_Dialog is closed
    ui.robotrun.join()
    # main()