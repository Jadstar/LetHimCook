import swift
import time
from cooking import CookingRobot, Patty, DONE_TEMP
from GUI import PattyVisualizer
from PyQt5.QtWidgets import QApplication
from spatialmath import SE3
import threading
def testpatty(patty, window, env):
    
    
    while patty.temperature < DONE_TEMP:
        patty.heat(1)  # Increase temperature by 1 degree for testing
        print(f"Current Patty Temperature: {patty.temperature}°C")
        window.update_display(patty.temperature)
        time.sleep(0.2)

def main():
    # Initialize the simulation environment
    env = swift.Swift()
    env.launch(realtime=True)
    patty = Patty(env=env)
    patty.setPose(SE3(2, 1, 1))
    patty.AddtoEnv(env)
    # Load the robot and cooking items
    robot = CookingRobot()
    
    # Initialize the GUI for Patty Visualization
    app = QApplication([])
    window = PattyVisualizer()
    window.show()

    # Start the testpatty function in a separate thread
    test_thread = threading.Thread(target=testpatty, args=(patty, window, env))
    test_thread.start()


    # Start the cooking process
    # robot.cook(patty)

    # Start the GUI's main loop
    app.exec_()

    test_thread.join()

if __name__ == "__main__":
    main()
