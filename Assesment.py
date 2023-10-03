import swift
import roboticstoolbox as rtb
from spatialmath import SE3
from LinearUR3.LinearUR3 import *
from math import pi
import time
import spatialgeometry as geometry
import numpy as np

class Brick:
    def __init__(self, mesh_path, initial_pose, env):
        # Initialise the brick with its mesh and initial pose
        self.mesh = geometry.Mesh(mesh_path, pose=initial_pose)
        self.current_pose = initial_pose
        self.env = env
        self.add_to_env()

    def add_to_env(self):
        # Add the brick mesh to the environment
        self.env.add(self.mesh)

    def move(self, T):
        # Move the brick to the specified transformation matrix
        adjusted_T = T
        self.mesh.T = adjusted_T

def assesment():   
    # Setting up 
    r = LinearUR3()
    env = swift.Swift()
    env.launch(realtime=True)

    
    # height of table
    # tableheight = 1.08
    # Add props and reqs to env
    # props = geometry.Mesh('props.dae', pose=SE3(1.5,5,0))
    # table = geometry.Mesh('Table.stl', pose=SE3(0,0,0), color=(0,0,1))
    # fence = geometry.Mesh('fence.dae', pose=SE3(0,0,-0.9) * SE3.Rx(pi/2))
    # stop = geometry.Mesh('stop.dae', pose=SE3(0.5,0.5,tableheight) * SE3.Rx(pi/2), color=(1,0,0))
    # env.add(table)
    # env.add(fence)
    # env.add(stop)
    # env.add(props)
    # Brick specifications
    brick_dimensions = (0.15, 0.1, 0.04)
    initial_position = (-0.85, 0.4, brick_dimensions[2] / 2)  # Adjusted for the brick's height
    spacing = 0.01
    # 5x4 easiest for robot to pickup
    brick_rows = [5, 4]  # 5 bricks in the first row, 4 in the second

    # Load and add bricks to the environment
    bricks = []
    brick_positions = []
    for i, num_bricks in enumerate(brick_rows):
        for j in range(num_bricks):
            x_offset = j * (brick_dimensions[0] + spacing)
            y_offset = i * (brick_dimensions[1] + spacing)
            brick_position = SE3(initial_position[0] + x_offset, initial_position[1] + y_offset, initial_position[2]) * SE3.Rz(-pi/2)
            brick_positions.append(brick_position)
            bricks.append(Brick('Brick.stl', brick_position, env))

    # Set initial configurations for the robot and the gripper
    r.q = r._qtest
    r.base = r.base * SE3(0, 0, 0)
    r.add_to_env(env)
    # g.palm_mesh.T = g.palm_mesh.T * SE3(-tableheight, 0, 0)
    # g.finger1.base = g.palm_mesh.T
    # g.finger2.base = g.palm_mesh.T
    # g.finger3.base = g.palm_mesh.T
    # g.addtoEnv(env)  # Add the gripper robot to the environment


    # Pick from top
    downward_orientation = SE3.Rx(pi) * SE3.Rz(pi/2) 
    # Adjust for the gripper's fingertip offset in Z direction
    z_offset = -0.2
    # Initial position for the 3x3
    stack_position = (-0.65, -0.4, 1.112 + brick_dimensions[2] - z_offset)
    # Define gripper closing trajectory
    # qtrajclosed = g.closedhandtraj()
    # Define gripper opening trajectory
    # qtrajopen = g.openhandtraj()
    # Counter for bricks stacked in the current column
    stack_counter = 0
    max_attempts = 100
    # time.sleep(100)
    for target_brick_position in brick_positions:
        brick_index = brick_positions.index(target_brick_position) 
        #set the target to pick up brick
        end_effector_target = SE3(target_brick_position.t[0], target_brick_position.t[1], target_brick_position.t[2] + (brick_dimensions[2]) - z_offset) * downward_orientation
        q_inital= [0.07199, -2.661, -1.986, -1.358, -1.368, 1.571, 2.052]
        q_goal_solution = r.ikine_LM(end_effector_target, q0=q_inital, ilimit = 200, slimit = 1000)
        print(q_goal_solution)
        if q_goal_solution.success:
            attempt = 0
            valid_solution_found = False

            while attempt < max_attempts and not valid_solution_found:
                q_goal_solution = r.ikine_LM(end_effector_target, q0=q_inital, ilimit = 200, slimit = 1000)
                
                if q_goal_solution.success:
                    q_goal = q_goal_solution.q
                    # Trajectory generation for the robot
                    qtraj = rtb.jtraj(r.q, q_goal, 50).q
                    below_table = False

                    # Check if any part of the robot goes below the table height
                    for q in qtraj:
                        end_effector_position = r.fkine(q).t
                        if end_effector_position[2] < 0:
                            below_table = True
                            break

                    # If any joint is below the table, discard the solution
                    if below_table:
                        print(f"IK solution attempt {attempt+1} violates table constraint. Trying another initial condition.")
                        # Modify the initial condition for the next attempt
                        q_inital = q_goal_solution.q
                        attempt += 1
                    else:
                        valid_solution_found = True
                else:
                    # If IK solution was not successful, change the initial condition for the next attempt
                    q_inital = np.random.uniform(r.qlim[0, :], r.qlim[1, :])
                    attempt += 1

            if not valid_solution_found:
                print(f"Failed to find a valid IK solution in {max_attempts} attempts.")
                continue

            qtraj = rtb.jtraj(r.q, q_goal, 50).q
            prev_q = r.q 
            # Start with the robot's initial configuration
            # for q, qfc in zip(qtraj, qtrajclosed):
            #     end_effector_pose = r.fkine(prev_q)
            #     g.move(end_effector_pose.A)
            #     r.q = q
            #     # Set the joint angles for the fingers
            #     g.finger1.q = qfc
            #     g.finger2.q = qfc
            #     g.finger3.q = -qfc
            #     env.step(0.02)
            #     prev_q = q

            # Calculate position for stacking the brick
            stack_target = SE3(stack_position[0], stack_position[1], stack_position[2] + (stack_counter*(brick_dimensions[2]-.01))) * downward_orientation
            q_goal_stack = r.ikine_LM(stack_target, ilimit=200, slimit=100).q

            # Move to the stack position
            qtraj_stack = rtb.jtraj(r.q, q_goal_stack, 50).q
            prev_q = r.q 
            
            for q in qtraj_stack:
                end_effector_pose = r.fkine(prev_q)
                # g.move(end_effector_pose.A)
                brickendeffector = end_effector_pose  * SE3(0, 0, 0.2)
                bricks[brick_index].move(brickendeffector.A)
                r.q = q
                # Set the joint angles for the fingers
                env.step(0.02)
                prev_q = q
            
            # for qfo in qtrajopen:
            #     g.finger1.q = qfo
            #     g.finger2.q = qfo
            #     g.finger3.q = -qfo
            #     env.step(0.02)

            # Increment the stack counter
            stack_counter += 1
            if stack_counter == 3:
                stack_position = (stack_position[0] + brick_dimensions[0]-0.0135, stack_position[1], 1.112 + brick_dimensions[2] - z_offset)
                stack_counter = 0

        else:
            print("Failed to find IK solution for brick")
    env.hold()       
    time.sleep(3)


if __name__ == "__main__":
    assesment()
    r=LinearUR3()
    # plot_robot_reach(r)
    # reach_and_volume(r)
    # desired_pose = SE3(0.5, 0.2, 0.3) * SE3.Rx(pi/4) 
    # print(r.getjointstate(desired_pose))
    # r.teach_robot()

    # env = swift.Swift()
    # env.launch(realtime=True)
    # r.add_to_env(env)

    # desired_pose = SE3(0.5, 0.2, 0.3)  # Example desired pose
    # distance = r.goto_jointstate(desired_pose, env)
    # print(f"Distance from desired pose: {distance} mm")