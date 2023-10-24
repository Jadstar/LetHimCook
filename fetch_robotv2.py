#!/usr/bin/env python
import swift
import numpy as np
from roboticstoolbox.robot.Robot import Robot
import roboticstoolbox as rtb
from math import pi
# from spatialmath import SE3


class Fetch(Robot):
    """
    Heavily taken from Robotics Toolbox for Python
    https://petercorke.github.io/robotics-toolbox-python/arm_erobot.html#defined-from-urdf

    As could not figure out how to define DH params, use ETS to define robot
    plus  URDF allows for ROS integration 

    ERobot: fetch (by Fetch), 10 joints (RPPRRRRRRR), 1 gripper, 6 branches, dynamics, geometry, collision
    ┌─────┬────────────────────┬───────┬────────────────────┬─────────────────────────────────────────────────────────┐
    │link │        link        │ joint │       parent       │                   ETS: parent to link                   │
    ├─────┼────────────────────┼───────┼────────────────────┼─────────────────────────────────────────────────────────┤
    │   0 │ base0              │       │ BASE               │ SE3()                                                   │
    │   1 │ base1              │     0 │ base0              │ SE3() ⊕ Rz(q0)                                          │
    │   2 │ base_link          │     1 │ base1              │ SE3() ⊕ tx(q1)                                          │
    │   3 │ torso_lift_link    │     2 │ base_link          │ SE3(-0.08687, 0, 0.3774; -3.508e-15°, -0°, 0°) ⊕ tz(q2) │
    │   4 │ shoulder_pan_link  │     3 │ torso_lift_link    │ SE3(0.1195, 0, 0.3486) ⊕ Rz(q3)                         │
    │   5 │ shoulder_lift_link │     4 │ shoulder_pan_link  │ SE3(0.117, 0, 0.06) ⊕ Ry(q4)                            │
    │   6 │ upperarm_roll_link │     5 │ shoulder_lift_link │ SE3(0.219, 0, 0) ⊕ Rx(q5)                               │
    │   7 │ elbow_flex_link    │     6 │ upperarm_roll_link │ SE3(0.133, 0, 0) ⊕ Ry(q6)                               │
    │   8 │ forearm_roll_link  │     7 │ elbow_flex_link    │ SE3(0.197, 0, 0) ⊕ Rx(q7)                               │
    │   9 │ wrist_flex_link    │     8 │ forearm_roll_link  │ SE3(0.1245, 0, 0) ⊕ Ry(q8)                              │
    │  10 │ @wrist_roll_link   │     9 │ wrist_flex_link    │ SE3(0.1385, 0, 0) ⊕ Rx(q9)                              │
    │  11 │ bellows_link       │       │ torso_lift_link    │ SE3()                                                   │
    │  12 │ bellows_link2      │       │ torso_lift_link    │ SE3()                                                   │
    │  13 │ estop_link         │       │ base_link          │ SE3(-0.1246, 0.2389, 0.3113; 90°, -0°, 0°)              │
    │  14 │ laser_link         │       │ base_link          │ SE3(0.235, 0, 0.2878; -180°, -0°, 0°)                   │
    │  15 │ torso_fixed_link   │       │ base_link          │ SE3(-0.08687, 0, 0.3774; -3.508e-15°, -0°, 0°)          │
    └─────┴────────────────────┴───────┴────────────────────┴─────────────────────────────────────────────────────────┘

    qr = arm aligned in x axis
    qz = arm tucked in 
    ┌─────┬─────┬────┬───────┬────────┬────────┬────────┬────────┬─────┬────────┬─────┐
    │name │ q0  │ q1 │ q2    │ q3     │ q4     │ q5     │ q6     │ q7  │ q8     │ q9  │
    ├─────┼─────┼────┼───────┼────────┼────────┼────────┼────────┼─────┼────────┼─────┤
    │  qr │  0° │  0 │  0.05 │  75.6° │  80.2° │ -11.5° │  98.5° │  0° │  95.1° │  0° │
    │  qz │  0° │  0 │  0    │  0°    │  0°    │  0°    │  0°    │  0° │  0°    │  0° │

    """

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(tld="",file_path="fetch_robotdata/robots/fetch.urdf")

        super().__init__(
            links,
            name=name,
            manufacturer="Fetch",
            gripper_links=links[11],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )
        self.qdlim = np.array([4.0, 4.0, 0.1, 1.25, 1.45, 1.57, 1.52, 1.57, 2.26, 2.26])
        

        self.qz = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.qr = np.array([0, 0, 0.05, 1.32, 1.4, -0.2, 1.72, 0, 1.66, 0])

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    env = swift.Swift()
    env.launch(realtime= True)

    robot = Fetch()
    qtest = [0,0,0,0,0,0,0,0,0,0]

    env.add(robot)
    for link in robot.links:
        print(link.name)
        print(link.isjoint)
        print(len(link.collision))

    print()

    for link in robot.grippers[0].links:
        print(link.name)
        print(link.isjoint)
        print(len(link.collision))

    # robot.plot(robot.q)
    print(robot.qz)
    print(robot.qr)
    tr = robot.fkine(qtest)
    tz = robot.fkine(robot.qz)
    print(robot.qlim)


    # Tep = robot.fkine([0, -0.3, 0, -2.2, 0, 2, 0.7854,0,0,0]).A
    q_goal = [robot.q[i]-pi/3 for i in range(len(robot.q))]

    finalpos = robot.ikine_LM(tz,q0=robot.qr,joint_limits=True)
    print(finalpos)
    if finalpos.success:
        qgoal = finalpos.q
        qtraj = rtb.jtraj(robot.q,qgoal,50,).q

    for q in qtraj:
        robot.q = q
        env.step(0.02)

    env.hold()
