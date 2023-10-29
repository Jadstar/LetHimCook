import swift
import time
from cooking import CookingRobot, Patty, DONE_TEMP, FLIP_TEMP
from GUI import PattyVisualizer
from PyQt5.QtWidgets import QApplication
from spatialmath import SE3
import threading

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


def main():
    # Initialize the simulation environment
    env = swift.Swift()
    env.launch(realtime=True)
    patty = Patty(SE3(2,1,1),env=env)
    patty.setPose(SE3(2, 1, 1))
    patty.AddtoEnv(env)
    
    # Load the robot and cooking items
    robot = CookingRobot()
    robot.AddtoEnv(env=env)
    
    # Initialize the GUI for Patty Visualization
    app = QApplication([])
    window = PattyVisualizer()
    window.show()
    goal_test = patty.getPose()
    while True:
        # Cooking logic from testpatty
        while not window.emergency_stop_requested and patty.temperature < FLIP_TEMP:
            patty.heat(1)  # Increase temperature by 1 degree for testing
            print(f"Current Patty Temperature: {patty.temperature}°C")
            window.update_display(patty.temperature)
            time.sleep(0.2)
        # If the patty is done cooking, break out of the loop
        if patty.temperature >= FLIP_TEMP:

            fullq = robot.flip_patty(patty)

            for q in fullq[0]:
                robot.CookMove(q)
                # tr = self.robot.fkine(q).A
                # patty.setPose(tr * self.patty_offset)
                env.step(0.02)
                
            # Flipping Patty
            for q in fullq[1]:
                robot.CookMove(q)
                tr = robot.robot.fkine(q).A
                patty.setPose(tr * robot.flipoffset)
                env.step(0.02)
            #Gravity
            for s in robot.PattyGravity(patty):
                patty.setPose(s)
                env.step(0.02)

            env.hold()
        if patty.temperature >=DONE_TEMP:
            print("done")
        # If emergency stop is requested, pause and wait for it to be unset
        while window.emergency_stop_requested:
            time.sleep(0.5)
    # Start the GUI's main loop
        app.exec_()

    
if __name__ == "__main__":
    main()
