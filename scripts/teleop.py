#!/usr/bin/env python

from pynput import keyboard

from collections import defaultdict

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Joy
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPositionFK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal


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

        self.axes = []
        self.buttons = []
    
    def update(self, joy):
        self.axes = joy.axes
        self.buttons = joy.buttons

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
            joint_constraint.tolerance_above = 0.0001
            joint_constraint.tolerance_below = 0.0001
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

if __name__ == "__main__":
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

        # Wait until a joint_state message is received
        # Necessary to compute the reference_position
        while not joint_state_subscriber.joint_state.position and not rospy.is_shutdown():
            pass

        print("Read initial joint state")
    
        reference_position = compute_fk(joint_state_subscriber.joint_state)

        print("Computed initial position")

        # Constant parameters
        arm_speed = 0.2
        arm_motion_plan_duration = 0.01
        gripper_speed = 0.2
        gripper_motion_plan_duration = 0.01
        wheels_linear_speed = 10
        wheels_angular_speed = 100

        print("Ready")
        
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        rate = rospy.Rate(100) # 100hz

        while not rospy.is_shutdown():
        
            time_now = rospy.Time.now()
            delta_time = (time_now - time_previous).to_sec()

            publish_arm = False
            for key, value in keybindings_arm.items():
                if active_keys[key]:
                    publish_arm = True
                    vx, vy, vz = value
                    reference_position.x += vx * arm_speed * delta_time
                    reference_position.y += vy * arm_speed * delta_time
                    reference_position.z += vz * arm_speed * delta_time
            
            if publish_arm:
                try:
                    joint_state_ik = compute_ik(joint_state_subscriber.joint_state, reference_position)

                    # No solution
                    if len(joint_state_ik.position) == 0:
                        raise IndexError

                    start_point = JointTrajectoryPoint()
                    start_point.positions = joint_state_subscriber.joint_state.position[2:6]
                    start_point.velocities = joint_state_subscriber.joint_state.velocity[2:6]
                    start_point.accelerations = [0 for _ in range(4)]
                    start_point.time_from_start = rospy.Duration(0)

                    end_point = JointTrajectoryPoint()
                    end_point.positions = joint_state_ik.position[0:4]
                    end_point.velocities = joint_state_ik.velocity[0:4]
                    end_point.accelerations = [0 for _ in range(4)]
                    end_point.time_from_start = rospy.Duration(arm_motion_plan_duration)

                    trajectory = JointTrajectory()
                    trajectory.header.frame_id = "base_footprint"
                    trajectory.joint_names = [f"joint{i}" for i in range(1, 5)]
                    trajectory.points = [start_point, end_point]

                    #trajectory = plan_kinematic_path(joint_state_subscriber.joint_state, joint_state_ik)

                    goal = FollowJointTrajectoryActionGoal()
                    goal.header.stamp = time_now
                    goal.goal_id.stamp = time_now
                    goal.goal_id.id = f"turtlebot_teleoperation_arm-{goal.goal_id.stamp.secs}.{goal.goal_id.stamp.nsecs}"
                    goal.goal.trajectory = trajectory
                    
                    arm_publisher.publish(goal)
                
                except IndexError:
                    pass
            
            action = None
            gripper_key0, gripper_key1 = keybindings_gripper.keys()
            if active_keys[gripper_key0] and not active_keys[gripper_key1]:
                action = keybindings_gripper[gripper_key0]
            elif not active_keys[gripper_key0] and active_keys[gripper_key1]:
                action = keybindings_gripper[gripper_key1]

            if action is not None:
                try:
                    start_point = JointTrajectoryPoint()
                    start_point.positions = [joint_state_subscriber.joint_state.position[0]]
                    start_point.velocities = [joint_state_subscriber.joint_state.velocity[0]]
                    start_point.accelerations = [0]
                    start_point.time_from_start = rospy.Duration(0)

                    end_point = JointTrajectoryPoint()
                    end_point.positions = [joint_state_subscriber.joint_state.position[0] + action * gripper_speed * delta_time]
                    end_point.velocities = [0]
                    end_point.accelerations = [0]
                    end_point.time_from_start = rospy.Duration(gripper_motion_plan_duration)

                    trajectory = JointTrajectory()
                    trajectory.header.frame_id = "base_footprint"
                    trajectory.joint_names = ["gripper"]
                    trajectory.points = [start_point, end_point]

                    goal = FollowJointTrajectoryActionGoal()

                    goal.header.stamp = rospy.Time.now()
                    goal.goal_id.stamp = rospy.Time.now()
                    goal.goal_id.id = f"turtlebot_teleoperation_gripper-{goal.goal_id.stamp.secs}.{goal.goal_id.stamp.nsecs}"
                    goal.goal.trajectory = trajectory

                    gripper_publisher.publish(goal)
                
                except IndexError:
                    pass

            wheels_twist = Twist()
            wheels_twist.linear.x = 0
            wheels_twist.linear.y = 0
            wheels_twist.linear.z = 0
            wheels_twist.angular.x = 0
            wheels_twist.angular.y = 0
            wheels_twist.angular.z = 0
            
            for key, value in keybindings_wheels.items():
                if active_keys[key]:
                    linear, angular = keybindings_wheels[key]

                    wheels_twist.linear.x += linear * wheels_linear_speed * delta_time
                    wheels_twist.angular.z += angular * wheels_angular_speed * delta_time

            wheel_publisher.publish(wheels_twist)

            rate.sleep()
        
            time_previous = time_now

    except rospy.ROSInterruptException:
        pass

