#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
import time
import numpy as np
from math import pi
import sys
class JointAngles:
    def __init__(self):
        self.name = ["", "", "", "", "", ""]  #could have also done [""] * 6
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# UR3e home position
home = np.radians([156, -88.77, 92.12, -93.32, -89.73, 19.85])

# Hanoi tower location 
Q11 = [144.57*pi/180.0, -58.54*pi/180.0, 120.35*pi/180.0, -151.78*pi/180.0, -89.86*pi/180.0, 103.57*pi/180.0]
Q12 = [144.57*pi/180.0, -65.78*pi/180.0, 118.92*pi/180.0, -143.12*pi/180.0, -89.86*pi/180.0, 103.57*pi/180.0]
Q13 = [144.60*pi/180.0, -72.27*pi/180.0, 116.18*pi/180.0, -133.88*pi/180.0, -89.83*pi/180.0, 103.55*pi/180.0]

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""
Q21 = [155.93*pi/180.0, -59.80*pi/180.0, 123.08*pi/180.0, -153.25*pi/180.0, -89.90*pi/180.0, 19.90*pi/180.0]
Q22 = [155.94*pi/180.0, -67.44*pi/180.0, 121.57*pi/180.0, -144.11*pi/180.0, -89.86*pi/180.0, 19.97*pi/180.0]
Q23 = [155.95*pi/180.0, -74.09*pi/180.0, 118.79*pi/180.0, -134.67*pi/180.0, -89.83*pi/180.0, 19.96*pi/180.0]

Q31 = [182.15*pi/180.0, -54.94*pi/180.0, 112.14*pi/180.0, -147.17*pi/180.0, -89.87*pi/180.0, 46.21*pi/180.0]
Q32 = [182.16*pi/180.0, -61.31*pi/180.0, 110.82*pi/180.0, -139.48*pi/180.0, -89.84*pi/180.0, 46.19*pi/180.0]
Q33 = [182.17*pi/180.0, -67.16*pi/180.0, 108.21*pi/180.0, -131.02*pi/180.0, -89.81*pi/180.0, 46.18*pi/180.0]

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############
class UR3e(Node):
    def __init__(self):
        super().__init__('ur3e')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        ############## Your Code Start Here ##############
        # TODO: define a ROS subscriber for gripper input message and corresponding callback function
        # ROS2 gripper input topic: /io_and_status_controller/io_states
        self.io_feedback_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_state_callback, 10)

        ############### Your Code End Here ###############

        # Service clients
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IO service not available, waiting...')

        # State variables
        self.current_joint_state = None
        self.analog_in_0_value = 0
        self.current_JointAngles = JointAngles()
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ] # shoulder_pan_joint is the base rotation joint

    def joint_state_callback(self, msg):
        self.current_joint_state = msg  # Currently only used to check if messages have arrived
        index_inOrder = 0
        for name in self.joint_names:
            index_outofOrder = msg.name.index(name)
            self.current_JointAngles.name[index_inOrder] = name
            self.current_JointAngles.position[index_inOrder] = msg.position[index_outofOrder]
            index_inOrder = index_inOrder + 1 


    def io_state_callback(self, msg):
    ############## Your Code Start Here ##############
        """
        TODO: define a ROS topic callback funtion that 
        receives and stores the state of  the suction cup
        Whenever /io_and_status_controller/io_states 
        publishes this info, this callback function is
        called.
        """

        self.analog_in_0_value = msg.analog_in_states[0].state
        pass

    ############### Your Code End Here ###############

    def set_io(self, pin, state):
        req = SetIO.Request()
        req.fun = 1
        req.pin = pin
        req.state = state
        future = self.io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def move_arm(self, target):
        if self.current_joint_state is None:
            self.get_logger().error("No joint state received!")
            return False

        V_MAX = 1#2.09    # rad/s
        A_MAX = 0.8#2.79   # rad/s^2
        MIN_DURATION = 1
        MAX_DURATION = 8.0

        deltas = []
        for i in range(6):
            deltas.append(abs(self.current_JointAngles.position[i] - target[i]))


        max_delta = max(deltas)
        t_acc = V_MAX / A_MAX
        d_acc = 0.5 * A_MAX * (t_acc ** 2)
        if max_delta > 2 * d_acc:
            # trapezoidal velocity profile
            t_total = 2 * t_acc + (max_delta - 2 * d_acc) / V_MAX
        else:
            # triangular velocity profile
            t_total = 2 * (max_delta / A_MAX) ** 0.5

        duration = max(MIN_DURATION, min(t_total, MAX_DURATION))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Start immediately when the controller receives it
        trajectory_msg.header.stamp.sec = 0
        trajectory_msg.header.stamp.nanosec = 0

        # Anchor point: current measured joint state at t = 0
        p0 = JointTrajectoryPoint()
        p0.positions = self.current_JointAngles.position
        p0.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        trajectory_msg.points.append(p0)

        # Goal point
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # end at rest, 2 point trajectory
        p1.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #end at rest.
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory_msg.points.append(p1)

        self.trajectory_pub.publish(trajectory_msg)

        self.get_logger().info(f'Moving to position: {np.degrees(target)}')

        # Wait for movement completion
        start_time = time.time()
        while time.time() - start_time < duration + 2:
            rclpy.spin_once(self, timeout_sec=0.1)

            deltas = []
            for i in range(6):
                deltas.append(abs(self.current_JointAngles.position[i] - target[i]))
            if all(delta < 0.001 for delta in deltas):
                time.sleep(0.25)
                return True
        return False


    def wait(self,sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_block(self, start_tower, start_height, end_tower, end_height):
        global Q
    ############## Your Code Start Here ##############
    # TODO: add code to move block from start tower and height to end tower and height
    ### Hint: Use the Q array to map out your towers by location and "height".
        self.move_arm(home)
        self.move_arm(Q[start_tower][start_height])
        self.set_io(0, 1.0)  # Turn on suction
        self.wait(1)  # Delay to make sure suction cup has grasped
        if (self.analog_in_0_value < 2.0):
            self.get_logger().error("Suction cup failed to grasp the block!")
            self.set_io(0, 0.0)
            return 1
        self.move_arm(home) 
        self.move_arm(Q[end_tower][end_height])
        self.set_io(0, 0.0)  # Turn off suction
        
        error = 0

        return error

    ############### Your Code End Here ###############


def main(args=None):
    input("Check if the UR3e is in 'Remote' Mode?\n\
    Check if the UR3e is initialized and in 'Normal' state.\n\
    Have you run the ROS2 launch statement?\n\
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.\n\
    \n\
    Press <Enter> to Continue.")
    rclpy.init(args=args)
    node = UR3e()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input
    loop_count = 0
    # Wait for initial state updates
    while node.current_joint_state is None:
        executor.spin_once(timeout_sec=0.05)
        node.get_logger().info("Waiting for initial state updates...")
        time.sleep(0.5)

    try:
        # Get user input
        start = int(input("Enter starting position")) - 1
        end = int(input("Enter ending position")) - 1
        intermediate = 3 - start - end
        print("You entered " + str(start) + " and " + str(end) + "\n")

        ############## Your Code Start Here ##############
        # TODO: modify the code so that UR3e can move tower accordingly from user input

        
        if (node.move_block(start, 2, end, 0)):
            return
           
        if (node.move_block(start, 1, intermediate, 0)):
            return
             
        if (node.move_block(end, 0, intermediate, 1)):
            return 
         
        if (node.move_block(start, 0, end, 0)):
            return
        
        if (node.move_block(intermediate, 1, start, 0)):
            return   
          
        if (node.move_block(intermediate, 0, end, 1)):
            return
        
        if (node.move_block(start, 0, end, 2)):
            return
        

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
