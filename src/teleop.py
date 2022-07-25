#!/usr/bin/env python

import math
import numpy as np

from pynput import keyboard

from collections import defaultdict

import rospy
from std_msgs.msg import Header, Bool, Float64MultiArray
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import JointState, Joy
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPositionFK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal, JointTolerance
from actionlib_msgs.msg import GoalID
from moveit_commander.move_group import MoveGroupCommander


keybindings_arm = {
    "i": (1, 0, 0),
    "k": (-1, 0, 0),
    "j": (0, 1, 0),
    "l": (0, -1, 0),
    "u": (0, 0, -1),
    "o": (0, 0, 1)
}

keybindings_gripper = {
    ".": 1,
    ",": -1
}

keybindings_wheels = {
    "w": (1, 0),
    "s": (-1, 0),
    "a": (0, 1),
    "d": (0, -1)
}

active_keys = defaultdict(lambda: False)

class JointStateSubscriber:

    def __init__(self):
        self.subscriber = rospy.Subscriber("joint_states", JointState, self.update)

        self.joint_state = JointState()
    
    def update(self, joint_state):
        self.joint_state = joint_state

class JoySubscriber:

    def __init__(self):
        self.subscriber = rospy.Subscriber("joy", Joy, self.update)

        # Xbox mapping:

        # Axes:
        # LeftX, LeftY, LeftTrigger, RightX, RightY, RightTrigger, DPadX, DPadY
        # SticksX: 1: left, -1: right
        # SticksY: 1: up, -1: down
        # Triggers: 1: off, -1, in

        # Buttons:
        # A, B, X, Y, LeftBumper, RightBumper, View / Select, Menu / Start, Xbox, LeftStick, RightStick
        # 0: off, 1: in

        self.axes = [0, 0, 1, 0, 0, 1, 0, 0]
        self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    def update(self, joy):
        self.buttons = joy.buttons

        for i in range(len(self.axes)):
            if joy.axes[i] > 0 and joy.axes[i] < 0.2:
                self.axes[i] = 0
            elif joy.axes[i] < 0 and joy.axes[i] > -0.2:
                self.axes[i] = 0
            else:
                self.axes[i] = joy.axes[i]

class TeleopSubscriber:

    def __init__(self):
        self.subscriber = rospy.Subscriber("turtlebot_teleop", Bool, self.update)
        self.publisher = rospy.Publisher("turtlebot_teleop", Bool, queue_size=10)

        self.activated = False
    
    def update(self, message):
        self.activated = message.data
    
    def stop(self):
        self.activated = False
        
        message = Bool()
        message.data = False
        self.publisher.publish(message)


class ComputeFK:

    def __init__(self):
        rospy.wait_for_service("compute_fk")
        self.service_function = rospy.ServiceProxy("compute_fk", GetPositionFK)
        
        self.header = Header()
        self.header.frame_id = "base_footprint"

        self.fk_link_names = ["link5"]

        self.robot_state = RobotState()
    
    def apply(self, joint_state):
        try:
            self.robot_state.joint_state = joint_state
            response = self.service_function(self.header, self.fk_link_names, self.robot_state)
            return response.pose_stamped[0].pose.position 

        except rospy.ServiceException as e:
            pass

    def __call__(self, joint_state):
        return self.apply(joint_state)

class ComputeIK:

    def __init__(self):
        rospy.wait_for_service("compute_ik")
        self.service_function = rospy.ServiceProxy("compute_ik", GetPositionIK)
        
        self.request = PositionIKRequest()
        self.request.group_name = "arm"
        self.request.pose_stamped.header.frame_id = "base_footprint"
        self.request.pose_stamped.pose.orientation.w = 1
    
    def apply(self, joint_state, request_position):
        try:
            self.request.robot_state.joint_state = joint_state
            self.request.pose_stamped.pose.position = request_position
            response = self.service_function(self.request)
            return response.solution.joint_state

        except rospy.ServiceException as e:
            pass

    def __call__(self, joint_state, request_position):
        return self.apply(joint_state, request_position)

class PlanKinematicPath:

    def __init__(self):
        rospy.wait_for_service("plan_kinematic_path")
        self.service_function = rospy.ServiceProxy("plan_kinematic_path", GetMotionPlan)
        
        self.motionplanrequest = MotionPlanRequest()
        self.motionplanrequest.workspace_parameters.min_corner.x = -1
        self.motionplanrequest.workspace_parameters.min_corner.y = -1
        self.motionplanrequest.workspace_parameters.min_corner.z = -1
        self.motionplanrequest.workspace_parameters.max_corner.x = -1
        self.motionplanrequest.workspace_parameters.max_corner.y = -1
        self.motionplanrequest.workspace_parameters.max_corner.z = -1
        self.motionplanrequest.workspace_parameters.header.frame_id = "base_footprint"
        self.motionplanrequest.group_name = "arm"

        goal_constraints = Constraints()

        for i in range(4):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = "joint" + str(i + 1)
            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1
            goal_constraints.joint_constraints.append(joint_constraint)

        self.motionplanrequest.goal_constraints = [goal_constraints]

    
    def apply(self, joint_state_start, joint_state_end):
        try:
            self.motionplanrequest.start_state.joint_state = joint_state_start
            for i in range(4):
                self.motionplanrequest.goal_constraints[0].joint_constraints[i].position = joint_state_end.position[i]

            response = self.service_function(self.motionplanrequest)
            return response.motion_plan_response.trajectory.joint_trajectory

        except rospy.ServiceException as e:
            pass

    def __call__(self, joint_state_start, joint_state_end):
        return self.apply(joint_state_start, joint_state_end)


def on_press(key):
    try:
        active_keys[key.char] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        active_keys[key.char] = False
    except AttributeError:
        pass

def main():
    print("Starting")
    try:
        rospy.init_node("isar_turtlebot_teleoperation", anonymous=True)

        arm_publisher = rospy.Publisher("joint_trajectory_point", Float64MultiArray, queue_size=10, tcp_nodelay=True)
        gripper_publisher = rospy.Publisher("gripper_position", Float64MultiArray, queue_size=10)
        wheel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        compute_fk = ComputeFK()
        compute_ik = ComputeIK()
        plan_kinematic_path = PlanKinematicPath()
    
        joint_state_subscriber = JointStateSubscriber()
        joy_subscriber = JoySubscriber()
        teleop_subscriber = TeleopSubscriber()

        # Wait until a joint_state message is received
        # Necessary to compute the reference_position
        while not joint_state_subscriber.joint_state.position and not rospy.is_shutdown():
            pass

        print("Read initial joint state")

        # Constant parameters
        arm_speed_x = 0.002
        arm_speed_y = 0.002
        arm_speed_z = 0.002
        arm_motion_plan_duration = 0.05
        arm_trajectory_points = 10

        gripper_speed = 0.01
        
        wheels_linear_speed = 10
        wheels_angular_speed = 10


        goal_arm = Float64MultiArray()
        goal_gripper = Float64MultiArray()
        

        start_state = JointState()
        start_state.header = joint_state_subscriber.joint_state.header
        start_state.name = joint_state_subscriber.joint_state.name[2:6]
        start_state.position = joint_state_subscriber.joint_state.position[2:6]
        start_state.velocity = joint_state_subscriber.joint_state.velocity[2:6]

        end_state = JointState()
        end_state.header = joint_state_subscriber.joint_state.header
        end_state.name = joint_state_subscriber.joint_state.name[2:6]
        end_state.position = [0, 0, 0, 0]
        end_state.velocity = [0, 0, 0, 0]

        trajectory_home = plan_kinematic_path(start_state, end_state)

        goal_arm = Float64MultiArray()
        for point in trajectory_home.points:
            time_from_start = float(point.time_from_start.secs) + float(point.time_from_start.nsecs) / 1e9
            goal_arm.data.extend([time_from_start, *point.positions])

        arm_publisher.publish(goal_arm)

        reference_position = compute_fk(end_state)
        reference_position.y = 0

        reference_position_backup = Point()
        reference_position_backup.x = 0
        reference_position_backup.y = 0
        reference_position_backup.z = 0

        joint_state_ik_backup = JointState()
        
        reference_angle = 0

        joint_state_ik = JointState()
        joint_state_ik.header = joint_state_subscriber.joint_state.header
        joint_state_ik.name = joint_state_subscriber.joint_state.name[2:6]
        joint_state_ik.position = [0, 0, 0, 0]
        joint_state_ik.velocity = [0, 0, 0, 0]

        print("Computed initial position")

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()


        wheels_twist = Twist()
        wheels_twist.linear.x = 0
        wheels_twist.linear.y = 0
        wheels_twist.linear.z = 0
        wheels_twist.angular.x = 0
        wheels_twist.angular.y = 0
        wheels_twist.angular.z = 0

        goal_gripper.data = [0]
        gripper_publisher.publish(goal_gripper)



        rate = rospy.Rate(100) # 100hz

        print("Ready")
        
        while not rospy.is_shutdown():

            rate.sleep()

            #if not teleop_subscriber.activated:
            #    continue

            #if joy_subscriber.buttons[7] == 1:
            #    teleop_subscriber.stop()
            #    continue

            wheels_twist.linear.x = 0
            wheels_twist.angular.z = 0
            
            if active_keys["w"] and not active_keys["s"]:
                wheels_twist.linear.x = wheels_linear_speed
            elif active_keys["s"] and not active_keys["w"]:
                wheels_twist.linear.x = - wheels_linear_speed
            elif joy_subscriber.axes[1] != 0:
                wheels_twist.linear.x = joy_subscriber.axes[1] * wheels_linear_speed

            if active_keys["a"] and not active_keys["d"]:
                wheels_twist.angular.z += wheels_angular_speed
            elif active_keys["d"] and not active_keys["a"]:
                wheels_twist.angular.z += - wheels_angular_speed
            elif joy_subscriber.axes[0] != 0:
                wheels_twist.angular.z = joy_subscriber.axes[0] * wheels_angular_speed

            wheel_publisher.publish(wheels_twist)

            arm_rotate = False
            arm_translate = False

            if active_keys["l"] and not active_keys["j"] and reference_angle > -3:
                reference_angle -= arm_speed_y
                arm_rotate = True
            elif active_keys["j"] and not active_keys["l"] and reference_angle < 3:
                reference_angle += arm_speed_y
                arm_rotate = True
            elif joy_subscriber.axes[3] != 0:
                reference_angle += 0.005 * joy_subscriber.axes[3]
                arm_rotate = True


            reference_position_backup.x = reference_position.x
            reference_position_backup.z = reference_position.z

            if active_keys["i"] and not active_keys["k"]:
                reference_position.x += arm_speed_x
                arm_translate = True
            elif active_keys["k"] and not active_keys["i"]:
                reference_position.x -= arm_speed_x
                arm_translate = True
            elif joy_subscriber.axes[4] != 0:
                reference_position.x += arm_speed_x / 2 * joy_subscriber.axes[4]
                arm_translate = True

            if active_keys["o"] and not active_keys["u"]:
                reference_position.z += arm_speed_z
                arm_translate = True
            elif active_keys["u"] and not active_keys["o"]:
                reference_position.z -= arm_speed_z
                arm_translate = True
            elif joy_subscriber.axes[2] - joy_subscriber.axes[5] != 0:
                reference_position.z += arm_speed_z / 2 * (joy_subscriber.axes[2] - joy_subscriber.axes[5]) / 2
                arm_translate = True

            
            if arm_translate:
                joint_state_ik_backup.header = joint_state_ik.header
                joint_state_ik_backup.name = joint_state_ik.name
                joint_state_ik_backup.position = joint_state_ik.position
                joint_state_ik_backup.velocity = joint_state_ik.velocity

                joint_state_ik = compute_ik(joint_state_subscriber.joint_state, reference_position)

                # No solution
                # Revert the changes to the reference position to unstuck it
                # Then escape the try block by raising an exception
                if len(joint_state_ik.position) == 0 or joint_state_ik.position[1] < -0.1 or joint_state_ik.position[3] < -0.1:
                    reference_position.x = reference_position_backup.x
                    reference_position.z = reference_position_backup.z
                    joint_state_ik = joint_state_ik_backup
                    arm_rotate = False
                    arm_translate = False

            if arm_rotate or arm_translate:
                goal_arm.data = [0, *joint_state_subscriber.joint_state.position[2:6]]

                end_positions = [reference_angle] + list(joint_state_ik.position[1:4])

                for i in range(1, arm_trajectory_points + 1):
                    goal_arm.data.append(arm_motion_plan_duration * i)
                    for j in range(4):
                        goal_arm.data.append(end_positions[j] * (i - 1) / (arm_trajectory_points - 1) + joint_state_subscriber.joint_state.position[j + 2] * (arm_trajectory_points - i) / (arm_trajectory_points - 1))
                    
                arm_publisher.publish(goal_arm)
            
            if active_keys["h"] or joy_subscriber.buttons[7] == 1:
                start_state = JointState()
                start_state.header = joint_state_subscriber.joint_state.header
                start_state.name = joint_state_subscriber.joint_state.name[2:6]
                start_state.position = joint_state_subscriber.joint_state.position[2:6]
                start_state.velocity = joint_state_subscriber.joint_state.velocity[2:6]

                end_state = JointState()
                end_state.header = joint_state_subscriber.joint_state.header
                end_state.name = joint_state_subscriber.joint_state.name[2:6]
                end_state.position = [0, 0, 0, 0]
                end_state.velocity = [0, 0, 0, 0]

                trajectory_home = plan_kinematic_path(start_state, end_state)

                goal_arm.data = []
                for point in trajectory_home.points:
                    time_from_start = float(point.time_from_start.secs) + float(point.time_from_start.nsecs) / 1e9
                    goal_arm.data.extend([time_from_start, *point.positions])

                arm_publisher.publish(goal_arm)

                reference_position = compute_fk(end_state)
                reference_position.y = 0
                reference_angle = 0

            gripper_action = None
            
            if active_keys["."] and not active_keys[","] or joy_subscriber.buttons[4] == 1 and joy_subscriber.buttons[5] == 0:
                gripper_action = -1
            elif active_keys[","] and not active_keys["."] or joy_subscriber.buttons[5] == 1 and joy_subscriber.buttons[4] == 0:
                gripper_action = 1
            
            if gripper_action:
                gripper_end = joint_state_subscriber.joint_state.position[6] + gripper_action * gripper_speed
                if gripper_end > 0.017:
                    gripper_end = 0.017

                goal_gripper.data = [gripper_end]
                gripper_publisher.publish(goal_gripper)
        
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
