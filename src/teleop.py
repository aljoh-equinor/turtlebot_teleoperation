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

        time_previous = rospy.Time.now()

        arm_publisher = rospy.Publisher("arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10)
        gripper_publisher = rospy.Publisher("gripper_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10)
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
        arm_speed_x = 0.2
        arm_speed_y = 1
        arm_speed_z = 0.2
        arm_motion_plan_duration = 1
        arm_trajectory_points = 10

        gripper_speed = 0.01
        gripper_motion_plan_duration = 0.1
        gripper_trajectory_points = 100

        wheels_linear_speed = 20
        wheels_angular_speed = 20

        trajectory_arm = JointTrajectory()
        trajectory_arm.header.frame_id = "base_footprint"
        trajectory_arm.joint_names = [f"joint{i}" for i in range(1, 5)]
        trajectory_arm.points = []

        for i in range(arm_trajectory_points + 1):
            point = JointTrajectoryPoint()
            point.accelerations = [0 for _ in range(4)]
            point.time_from_start = rospy.Duration(arm_motion_plan_duration * i)
            trajectory_arm.points.append(point)

        goal_arm = FollowJointTrajectoryActionGoal()
        #goal_arm.goal.trajectory = trajectory_arm
        """goal_arm.goal.goal_time_tolerance = rospy.Duration(5)
        for i in range(4):
            goal_arm.goal.goal_tolerance.append(JointTolerance())
            goal_arm.goal.goal_tolerance[i].name = "joint" + str(i + 1)
            goal_arm.goal.goal_tolerance[i].position = 6
            goal_arm.goal.goal_tolerance[i].velocity = 6
            goal_arm.goal.goal_tolerance[i].acceleration = 6"""
        

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

        goal_arm.header.stamp = rospy.Time.now()
        goal_arm.goal_id.stamp = rospy.Time.now()
        goal_arm.goal_id.id = f"turtlebot_teleoperation_arm-{goal_arm.goal_id.stamp.secs}.{goal_arm.goal_id.stamp.nsecs}"
        goal_arm.goal.trajectory = trajectory_home

        arm_publisher.publish(goal_arm)

        # fiks at home på startup ikke virker
        # dette fikser også initialisering av referansepunktet
        # legg til timer i inputlesning
        # tuning



        reference_position = compute_fk(end_state)
        reference_position.y = 0

        reference_position_backup = Point()
        reference_position_backup.x = 0
        reference_position_backup.y = 0
        reference_position_backup.z = 0

        joint_state_ik_backup = JointState()
        
        reference_angle = 0

        #joint_state_ik = compute_ik(joint_state_subscriber.joint_state, reference_position)
        joint_state_ik = JointState()
        joint_state_ik.header = joint_state_subscriber.joint_state.header
        joint_state_ik.name = joint_state_subscriber.joint_state.name[2:6]
        joint_state_ik.position = [0, 0, 0, 0]
        joint_state_ik.velocity = [0, 0, 0, 0]

        print("Computed initial position")

        print("Ready")

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()


        wheels_twist = Twist()
        wheels_twist.linear.x = 0
        wheels_twist.linear.y = 0
        wheels_twist.linear.z = 0
        wheels_twist.angular.x = 0
        wheels_twist.angular.y = 0
        wheels_twist.angular.z = 0


        start_point_gripper = JointTrajectoryPoint()
        start_point_gripper.accelerations = [0]
        start_point_gripper.time_from_start = rospy.Duration(0)
        
        end_point_gripper = JointTrajectoryPoint()
        end_point_gripper.velocities = [0]
        end_point_gripper.accelerations = [0]
        end_point_gripper.time_from_start = rospy.Duration(gripper_motion_plan_duration)

        trajectory_gripper = JointTrajectory()
        trajectory_gripper.header.frame_id = "base_footprint"
        trajectory_gripper.joint_names = ["gripper"]
        trajectory_gripper.points = [start_point_gripper, end_point_gripper]

        goal_gripper = FollowJointTrajectoryActionGoal()
        goal_gripper.goal.trajectory = trajectory_gripper

        rate = rospy.Rate(100) # 100hz

        active_keys_prev = active_keys
        
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

            if active_keys["l"] and not active_keys_prev["l"] and not active_keys["j"] and reference_angle > -3:
                reference_angle -= 0.1
                arm_rotate = True
            elif active_keys["j"] and not active_keys_prev["j"] and not active_keys["l"] and reference_angle < 3:
                reference_angle += 0.1
                arm_rotate = True
            elif joy_subscriber.axes[3] != 0:
                reference_angle -= 0.1 * joy_subscriber.axes[3]
                arm_rotate = True

            if active_keys["i"] and not active_keys_prev["i"] and not active_keys["k"]:
                reference_position.x += 0.01
                arm_translate = True
            elif active_keys["k"] and not active_keys_prev["k"] and not active_keys["i"]:
                reference_position.x -= 0.01
                arm_translate = True
            elif joy_subscriber.axes[4] != 0:
                reference_position.x += 0.1 * joy_subscriber.axes[4]
                arm_translate = True

            if active_keys["o"] and not active_keys_prev["o"] and not active_keys["u"]:
                reference_position.z += 0.01
                arm_translate = True
            elif active_keys["u"] and not active_keys_prev["u"] and not active_keys["o"]:
                reference_position.z -= 0.01
                arm_translate = True
            elif joy_subscriber.axes[2] - joy_subscriber.axes[5] != 0:
                reference_position.z += 0.001 * (joy_subscriber.axes[2] - joy_subscriber.axes[5]) / 2
                arm_translate = True

            
            if arm_translate:
                try:
                    reference_position_backup.x = reference_position.x
                    reference_position_backup.z = reference_position.z

                    joint_state_ik_backup.header = joint_state_ik.header
                    joint_state_ik_backup.name = joint_state_ik.name
                    joint_state_ik_backup.position = joint_state_ik.position
                    joint_state_ik_backup.velocity = joint_state_ik.velocity

                    joint_state_ik = compute_ik(joint_state_subscriber.joint_state, reference_position)

                    # No solution
                    # Revert the changes to the reference position to unstuck it
                    # Then escape the try block by raising an exception
                    if len(joint_state_ik.position) == 0:
                        raise IndexError
                
                except IndexError:
                    reference_position = reference_position_backup
                    joint_state_ik = joint_state_ik_backup
                    arm_rotate = False
                    arm_translate = False

            if arm_rotate or arm_translate:
                trajectory_arm.points[0].positions = joint_state_subscriber.joint_state.position[2:6]
                trajectory_arm.points[0].velocities = joint_state_subscriber.joint_state.velocity[2:6]

                end_positions = [reference_angle] + list(joint_state_ik.position[1:4])

                arm_time = 0.1
                arm_motion_plan_duration = abs(reference_angle - joint_state_subscriber.joint_state.position[2]) / arm_trajectory_points * arm_time

                for i in range(1, arm_trajectory_points + 1):
                    #trajectory_arm.points[i].positions = end_positions
                    trajectory_arm.points[i].positions = [0, 0, 0, 0]
                    for j in range(4):
                        trajectory_arm.points[i].positions[j] = end_positions[j] * (i - 1) / (arm_trajectory_points - 1) + joint_state_subscriber.joint_state.position[j + 2] * (arm_trajectory_points - i) / (arm_trajectory_points - 1)
                    trajectory_arm.points[i].velocities = [0, 0, 0, 0]
                    trajectory_arm.points[i].velocities[0] = -i * (i - arm_trajectory_points)
                    trajectory_arm.points[i].time_from_start = rospy.Duration(arm_motion_plan_duration * i)
                
                goal_arm.goal.trajectory = trajectory_arm
                goal_arm.header.stamp = rospy.Time.now()
                goal_arm.goal_id.stamp = rospy.Time.now()
                goal_arm.goal_id.id = f"turtlebot_teleoperation_arm-{goal_arm.goal_id.stamp.secs}.{goal_arm.goal_id.stamp.nsecs}"

                arm_publisher.publish(goal_arm)
            
            if active_keys["h"] and not active_keys_prev["h"]:
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

                goal_arm.header.stamp = rospy.Time.now()
                goal_arm.goal_id.stamp = rospy.Time.now()
                goal_arm.goal_id.id = f"turtlebot_teleoperation_arm-{goal_arm.goal_id.stamp.secs}.{goal_arm.goal_id.stamp.nsecs}"
                goal_arm.goal.trajectory = trajectory_home

                arm_publisher.publish(goal_arm)

                reference_position = compute_fk(end_state)
                reference_position.y = 0
                reference_angle = 0

            gripper_action = None
            
            if active_keys["."] and not active_keys[","] or joy_subscriber.buttons[4] == 1 and joy_subscriber.buttons[5] == 0:
                gripper_action = 1
            elif active_keys[","] and not active_keys["."] or joy_subscriber.buttons[5] == 1 and joy_subscriber.buttons[4] == 0:
                gripper_action = - 1
            
            if gripper_action:
                try:
                    goal_gripper.goal.trajectory.points[0].positions = [joint_state_subscriber.joint_state.position[6]]
                    goal_gripper.goal.trajectory.points[0].velocities = [joint_state_subscriber.joint_state.velocity[6]]

                    gripper_end = joint_state_subscriber.joint_state.position[6] + gripper_action * gripper_speed
                    if gripper_end < 0.024:
                        gripper_end = 0.024

                    goal_gripper.goal.trajectory.points[1].positions = [gripper_end]

                    goal_gripper.header.stamp = rospy.Time.now()
                    goal_gripper.goal_id.stamp = rospy.Time.now()
                    goal_gripper.goal_id.id = f"turtlebot_teleoperation_gripper-{goal_gripper.goal_id.stamp.secs}.{goal_gripper.goal_id.stamp.nsecs}"
                    
                    gripper_publisher.publish(goal_gripper)
                
                except IndexError:
                    pass
            
            active_keys_prev = active_keys.copy()
        
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
