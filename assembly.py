'''
THIS IS WHERE THE UR3 BURGER ASSSEMBLY WILL BE DONE BEFORE PUSHING IT INTO MAIN.PY
'''


import roboticstoolbox as rtb
from spatialmath import SE3
from ir_support import UR5
from math import pi
from spatialmath.base import *
from roboticstoolbox import models, jtraj, trapezoidal
import numpy as np
import time
import matplotlib.pyplot as plt


class AssemblyRobot:

    def __init__(self):
        self.robot = UR5()

    def setPose(self, position):
        self.robot.base = position