#!/usr/bin/env python

import math
import numpy as np
import cv2

from pynput import keyboard

from collections import defaultdict

import rospy
from std_msgs.msg import Header, Bool, Float64MultiArray
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState, Joy, Image, CompressedImage
from control_msgs.msg import FollowJointTrajectoryActionGoal
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPositionFK
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge


class JointStateSubscriber:

    def __init__(self):
        self.subscriber = rospy.Subscriber("joint_states", JointState, self.update)

        self.joint_state = JointState()
    
    def update(self, joint_state):
        self.joint_state = joint_state
    
    def get_current_state(self):
        current_state = JointState()
        current_state.header = self.joint_state.header
        current_state.name = self.joint_state.name[2:6]
        current_state.position = self.joint_state.position[2:6]
        current_state.velocity = self.joint_state.velocity[2:6]
        return current_state

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
        self.buttons = list(joy.buttons)

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


class CameraSubscriber:

    def __init__(self):
        #self.color_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.update_color)
        self.color_subscriber = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.update_color)
        self.depth_subscriber = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.update_depth)
        #self.depth_subscriber = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw/compressed", CompressedImage, self.update_depth)

        self.bridge = CvBridge()

        self.color = None
        self.depth = None
    
    def update_color(self, message):
        #self.color = self.bridge.imgmsg_to_cv2(message)
        self.color = self.bridge.compressed_imgmsg_to_cv2(message)
    
    def update_depth(self, message):
        self.depth = np.array(self.bridge.imgmsg_to_cv2(message), dtype=np.float32)
        #self.depth = self.bridge.compressed_imgmsg_to_cv2(message)


class OdomSubscriber:

    def __init__(self):
        self.subscriber = rospy.Subscriber("/odom", Odometry, self.update)

        self.position = None
        self.orientation = None
    
    def update(self, message):
        self.position = message.pose.pose.position
        self.orientation = message.pose.pose.orientation


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


active_keys = defaultdict(lambda: False)

def on_press(key):
    if key == keyboard.Key.shift:
        active_keys["shift"] = True
        return
    elif key == keyboard.Key.enter:
        active_keys["enter"] = True
        return
    try:
        active_keys[key.char] = True
    except AttributeError:
        pass

def on_release(key):
    if key == keyboard.Key.shift:
        active_keys["shift"] = False
        return
    elif key == keyboard.Key.enter:
        active_keys["enter"] = False
        return
    try:
        active_keys[key.char] = False
    except AttributeError:
        pass


def goto(joint_state_position, joint_state_subscriber, plan_kinematic_path, arm_publisher, rate, goto_attempt_counter=0, max_goto_attempts=10, goto_timeout=2, goto_offset=0.1, trajectory_points=20):
    joint_state = JointState()
    joint_state.header = joint_state_subscriber.joint_state.header
    joint_state.name = joint_state_subscriber.joint_state.name[2:6]
    joint_state.position = joint_state_position
    joint_state.velocity = [0, 0, 0, 0]

    while np.max(np.abs(np.array(joint_state_subscriber.joint_state.position[2:6]) - np.array(joint_state_position))) > goto_offset and goto_attempt_counter < max_goto_attempts:
        trajectory = plan_kinematic_path(joint_state_subscriber.get_current_state(), joint_state)
        trajectory_home_length = len(trajectory.points)

        if trajectory_home_length == 0:
            print("No path found")
            return

        end_index = min(trajectory_points, trajectory_home_length - 1)

        goal_arm = Float64MultiArray()
        for point in trajectory.points[:end_index + 1]:
            goal_arm.data.extend([point.time_from_start.to_sec(), *point.positions])

        arm_publisher.publish(goal_arm)

        end_point = np.array(trajectory.points[end_index].positions)

        start_time = rospy.Time.now().to_sec()
        while (np.max(np.abs(np.array(joint_state_subscriber.joint_state.position[2:6] - end_point))) > goto_offset) and (rospy.Time.now().to_sec() - start_time < goto_timeout):
            rate.sleep()
        
        goto_attempt_counter += 1


def main():

    # Constant parameters
    arm_speed_x = 0.001
    arm_speed_y = 0.006
    arm_speed_z = 0.001
    arm_motion_plan_duration = 0.1
    arm_trajectory_points = 10

    gripper_speed = 0.01
    gripper_max = 0.017
    
    wheels_linear_speed = 5
    wheels_angular_speed = 5

    print("Starting")

    try:
        rospy.init_node("isar_turtlebot_teleoperation", anonymous=True)

        arm_publisher = rospy.Publisher("joint_trajectory_point", Float64MultiArray, queue_size=10)
        gripper_publisher = rospy.Publisher("gripper_position", Float64MultiArray, queue_size=10)
        wheel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        pose_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)

        #compute_fk = ComputeFK()
        #compute_ik = ComputeIK()
        #plan_kinematic_path = PlanKinematicPath()
    
        joint_state_subscriber = JointStateSubscriber()
        joy_subscriber = JoySubscriber()
        teleop_subscriber = TeleopSubscriber()
        camera_subscriber = CameraSubscriber()
        odom_subscriber = OdomSubscriber()

        rate = rospy.Rate(100) # 100hz

        # Wait until a joint_state message is received
        # Necessary to compute the reference_position
        # not (len(joint_state_subscriber.joint_state.position[2:6]) == 4)
        """while not joint_state_subscriber.joint_state.position and not rospy.is_shutdown():
            rate.sleep()"""

        print("Read initial joint state")

        while (camera_subscriber.color is None or camera_subscriber.depth is None) and not rospy.is_shutdown():
        #while camera_subscriber.color is None and not rospy.is_shutdown():
            rate.sleep()
        
        """img_d = camera_subscriber.depth
        img_d = cv2.rotate(img_d, cv2.ROTATE_180)
        print(img_d[200:280, 280:360])
        print(np.unique(img_d))
        cv2.imwrite("img3.jpg", img_d)
        exit()"""

        home_state_position = [0, 0, 0, 0]
        throw_state_position = [0, -np.pi * 0.6, -np.pi * 0.5, -np.pi * 0.5]
        pickup_state_position = [0, np.pi * 0.36, -np.pi * 0.4, np.pi * 0.3]

        print("Started home trajectory")
        #goto(home_state_position, joint_state_subscriber, plan_kinematic_path, arm_publisher, rate)

        joint_state_ik = JointState()
        joint_state_ik.header = joint_state_subscriber.joint_state.header
        joint_state_ik.name = joint_state_subscriber.joint_state.name[2:6]
        joint_state_ik.position = home_state_position
        joint_state_ik.velocity = [0, 0, 0, 0]

        #reference_position = compute_fk(joint_state_ik)
        #reference_position.y = 0

        reference_angle = 0

        reference_position_backup_x = 0
        reference_position_backup_z = 0

        joint_state_ik_backup = JointState()

        print("Computed initial position")

        wheels_twist = Twist()
        wheels_twist.linear.x = 0
        wheels_twist.linear.y = 0
        wheels_twist.linear.z = 0
        wheels_twist.angular.x = 0
        wheels_twist.angular.y = 0
        wheels_twist.angular.z = 0

        goal_gripper = Float64MultiArray()
        goal_gripper.data = [0]
        gripper_publisher.publish(goal_gripper)

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        print("Ready")

        while not rospy.is_shutdown():
            rate.sleep()

            """if not teleop_subscriber.activated:
                continue

            if active_keys["enter"] or joy_subscriber.buttons[7] == 1:
                teleop_subscriber.stop()
                continue"""

            if active_keys["shift"] or joy_subscriber.buttons[0] == 1:
                active_keys["shift"] = False
                joy_subscriber.buttons[0] = 0

                #image_run = True
                #while image_run:

                img = camera_subscriber.color
                img = cv2.rotate(img, cv2.ROTATE_180)

                #cv2.imwrite("img.jpg", img)

                img_t = img.copy()
                img_t = cv2.cvtColor(img_t, cv2.COLOR_BGR2HSV)

                reference_color_rgb = [150, 80, 250]
                reference_color_hsv = list(cv2.cvtColor(np.array([[reference_color_rgb]], dtype=np.uint8), cv2.COLOR_BGR2HSV)[0, 0, :])
                reference_threshold = [5, 100, 100]

                for i in range(3):
                    img_t[img_t[:, :, i] < reference_color_hsv[i] - reference_threshold[i]] = [0, 0, 0]
                    img_t[img_t[:, :, i] > reference_color_hsv[i] + reference_threshold[i]] = [0, 0, 0]
                
                img_t = cv2.cvtColor(img_t, cv2.COLOR_HSV2BGR)
                img_t = cv2.cvtColor(img_t, cv2.COLOR_BGR2GRAY)
                (thresh, img_t) = cv2.threshold(img_t, 10, 255, cv2.THRESH_BINARY)

                #cv2.imwrite("img2.jpg", img_t)

                # https://en.wikipedia.org/wiki/Mathematical_morphology

                # Closing: Fill holes and connect components
                kernel = np.ones((5, 5), np.uint8)
                img_t = cv2.dilate(img_t, kernel)
                img_t = cv2.erode(img_t, kernel)
                
                # Opening: remove small noise
                kernel = np.ones((5, 5), np.uint8)
                img_t = cv2.erode(img_t, kernel)
                img_t = cv2.dilate(img_t, kernel)

                #cv2.imwrite("img3.jpg", img_t)

                totalLabels, label_ids, values, centroid = cv2.connectedComponentsWithStats(img_t, 4, cv2.CV_8U)

                """for i in range(1, totalLabels):
                    if values[i, cv2.CC_STAT_AREA] > 100:
                        print(values[i, cv2.CC_STAT_AREA])
                        if centroid[i, 0] > 350:
                            wheels_twist.linear.x = wheels_linear_speed
                            wheels_twist.angular.z = 0
                            wheel_publisher.publish(wheels_twist)
                            break
                        elif centroid[i, 0] < 290:
                            wheels_twist.linear.x = - wheels_linear_speed
                            wheels_twist.angular.z = 0
                            wheel_publisher.publish(wheels_twist)
                            break
                        elif values[i, cv2.CC_STAT_AREA] < 10000:
                            wheels_twist.linear.x = wheels_linear_speed
                            wheels_twist.angular.z = 0
                            wheel_publisher.publish(wheels_twist)
                            break
                        else:
                            wheels_twist.linear.x = 0
                            wheels_twist.angular.z = 0
                            wheel_publisher.publish(wheels_twist)
                            image_run = False
                            break"""
            
                #continue

                dst = cv2.cornerHarris(img_t, 4, 3, 0.04)
                object_corners = np.where(dst > 0.1 * dst.max())

                img_tt = img_t.copy()
                img_tt = cv2.cvtColor(img_tt, cv2.COLOR_GRAY2BGR)
                
                """object_bounding_boxes = {}
                for y, x in zip(*object_corners):
                    label = label_ids[y, x]
                    if label != 0 and values[label, cv2.CC_STAT_AREA] > 500:
                        if label not in object_bounding_boxes:
                            object_bounding_boxes[label] = {"x": [10000, -1], "y": [10000, -1]}

                        if x < object_bounding_boxes[label]["x"][0]:
                            object_bounding_boxes[label]["x"][0] = x
                        if x > object_bounding_boxes[label]["x"][1]:
                            object_bounding_boxes[label]["x"][1] = x

                        if y < object_bounding_boxes[label]["y"][0]:
                            object_bounding_boxes[label]["y"][0] = y
                        if y > object_bounding_boxes[label]["y"][1]:
                            object_bounding_boxes[label]["y"][1] = y

                        #img_tt[y, x] = [0, 0, 255]
                        img_tt = cv2.circle(img_tt, (x, y), 5, [0, 0, 255], -1)
                
                cv2.imwrite("img4.jpg", img_tt)

                img_tt = img_t.copy()
                img_tt = cv2.cvtColor(img_tt, cv2.COLOR_GRAY2BGR)
                
                for key, bb in object_bounding_boxes.items():
                    minX = bb["x"][0]
                    maxX = bb["x"][1]
                    minY = bb["y"][0]
                    maxY = bb["y"][1]

                    img_tt = cv2.line(img_tt, (minX, minY), (minX, maxY), [0, 0, 255], 5)
                    img_tt = cv2.line(img_tt, (minX, maxY), (maxX, maxY), [0, 0, 255], 5)
                    img_tt = cv2.line(img_tt, (maxX, maxY), (maxX, minY), [0, 0, 255], 5)
                    img_tt = cv2.line(img_tt, (maxX, minY), (minX, minY), [0, 0, 255], 5)
                
                cv2.imwrite("img5.jpg", img_tt)"""
                
                object_bounding_boxes = {}
                for y, x in zip(*object_corners):
                    label = label_ids[y, x]
                    if label != 0 and values[label, cv2.CC_STAT_AREA] > 100:
                        if label not in object_bounding_boxes:
                            object_bounding_boxes[label] = {"minX": [10000, 0], "maxX": [-1, -0], "maxY": [0, -1]}
                        
                        if x < object_bounding_boxes[label]["minX"][0]:
                            object_bounding_boxes[label]["minX"][0] = x
                            object_bounding_boxes[label]["minX"][1] = y
                        
                        if x > object_bounding_boxes[label]["maxX"][0]:
                            object_bounding_boxes[label]["maxX"][0] = x
                            object_bounding_boxes[label]["maxX"][1] = y
                        
                        if y > object_bounding_boxes[label]["maxY"][1]:
                            object_bounding_boxes[label]["maxY"][0] = x
                            object_bounding_boxes[label]["maxY"][1] = y


                img_tt = img_t.copy()
                img_tt = cv2.cvtColor(img_tt, cv2.COLOR_GRAY2BGR)
                
                for key, bb in object_bounding_boxes.items():
                    minX_x = bb["minX"][0]
                    minX_y = bb["minX"][1]

                    maxX_x = bb["maxX"][0]
                    maxX_y = bb["maxX"][1]

                    maxY_x = bb["maxY"][0]
                    maxY_y = bb["maxY"][1]

                    if (maxY_x - minX_x)**2 + (maxY_y - minX_y)**2 > (maxY_x - maxX_x)**2 + (maxY_y - maxX_y)**2:
                        object_angle = np.arctan((maxY_x - minX_x) / (maxY_y - minX_y))
                    else:
                        object_angle = np.arctan((maxY_x - maxX_x) / (maxY_y - maxX_y))
                    
                    pdx = (minX_x + maxX_x + maxY_x) // 3
                    pdy = (minX_y + maxX_y + maxY_y) // 3
                    

                    img_d = cv2.rotate(camera_subscriber.depth, cv2.ROTATE_180)
                    #img_d = camera_subscriber.depth
                    total_distance = img_d[pdy, pdx] / 1000
                    if total_distance == 0:
                        print("Distance 0")
                        break

                    distance = np.sqrt(total_distance**2 - 0.05**2)
                    
                    travel_angle = np.arctan((pdx - 320) / (480 - pdy))
                    distance_x = distance * np.sin(travel_angle)
                    distance_y = distance * np.cos(travel_angle)

                    """print(f"Object angle: {object_angle * 180 / 3.14:.1f} degrees")
                    print(f"Total distance: {total_distance * 100:.1f} cm")
                    print(f"Travel angle: {travel_angle * 180 / 3.14:.1f} degrees")
                    print(f"x distance: {distance_x * 100:.1f} cm")
                    print(f"y distance: {distance_y * 100:.1f} cm")"""
                    
                    """img_tt = cv2.circle(img_tt, (minX_x, minX_y), 5, [0, 0, 255], -1)
                    img_tt = cv2.circle(img_tt, (maxX_x, maxX_y), 5, [0, 0, 255], -1)
                    img_tt = cv2.circle(img_tt, (maxY_x, maxY_y), 5, [0, 0, 255], -1)

                    img_tt = cv2.line(img_tt, (minX_x, minX_y), (maxY_x, maxY_y), [0, 0, 255], 5)
                    img_tt = cv2.line(img_tt, (maxX_x, maxX_y), (maxY_x, maxY_y), [0, 0, 255], 5)

                    cv2.imwrite("img6.jpg", img_tt)"""


                    orientation = odom_subscriber.orientation

                    angle = 2 * np.arccos(orientation.w)
                    z_axis = orientation.z / np.sqrt(1 - orientation.w * orientation.w)
                    angle *= z_axis

                    dx = distance_y - np.cos(object_angle) * 0.1
                    dy = - distance_x - np.sin(object_angle) * 0.1

                    dx_transformed = np.cos(angle) * dx - np.sin(angle) * dy
                    dy_transformed = np.sin(angle) * dx + np.cos(angle) * dy

                    position_x = odom_subscriber.position.x + dx_transformed
                    position_y = odom_subscriber.position.y + dy_transformed

                    angle += object_angle

                    qz = np.sin(angle / 2)
                    qw = np.cos(angle / 2)

                    target_pose = PoseStamped()
                    target_pose.header.frame_id = "odom"
                    target_pose.pose.position.x = position_x
                    target_pose.pose.position.y = position_y
                    target_pose.pose.position.z = 0
                    target_pose.pose.orientation.x = 0
                    target_pose.pose.orientation.y = 0
                    target_pose.pose.orientation.z = qz
                    target_pose.pose.orientation.w = qw

                    pose_publisher.publish(target_pose)

                
                    """angle = np.arctan2(target_pose.pose.position.y - odom_subscriber.position.y, target_pose.pose.position.x - odom_subscriber.position.x)
                    initial_target_rotation_z = qz = np.sin(angle / 2)
                    initial_target_rotation_w = np.cos(angle / 2)

                    wheels_twist.linear.x = 0
                    wheels_twist.angular.z = wheels_angular_speed
                    wheel_publisher.publish(wheels_twist)

                    start_time = rospy.Time.now().to_sec()
                    while (abs(odom_subscriber.orientation.z - initial_target_rotation_z) > 0.1 or abs(odom_subscriber.orientation.w - initial_target_rotation_w) > 0.1) and rospy.Time.now().to_sec() - start_time < 3:
                        rate.sleep()
                    
                    wheels_twist.linear.x = wheels_linear_speed
                    wheels_twist.angular.z = 0
                    wheel_publisher.publish(wheels_twist)
                    
                    start_time = rospy.Time.now().to_sec()
                    while (abs(odom_subscriber.position.x - target_pose.pose.position.x) > 0.1 or abs(odom_subscriber.position.y - target_pose.pose.position.y) > 0.1) and rospy.Time.now().to_sec() - start_time < 3:
                        rate.sleep()
                    
                    wheels_twist.linear.x = 0
                    wheels_twist.angular.z = wheels_angular_speed
                    wheel_publisher.publish(wheels_twist)

                    start_time = rospy.Time.now().to_sec()
                    while (abs(odom_subscriber.orientation.z - target_pose.pose.orientation.z) > 0.1 or abs(odom_subscriber.orientation.w - target_pose.pose.orientation.w) > 0.1) and rospy.Time.now().to_sec() - start_time < 3:
                        rate.sleep()
                    
                    wheels_twist.linear.x = 0
                    wheels_twist.angular.z = 0
                    wheel_publisher.publish(wheels_twist)"""

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

            #wheel_publisher.publish(wheels_twist)


            arm_rotate = False
            arm_translate = False

            if active_keys["l"] and not active_keys["j"] and reference_angle > -3:
                reference_angle -= arm_speed_y
                arm_rotate = True
            elif active_keys["j"] and not active_keys["l"] and reference_angle < 3:
                reference_angle += arm_speed_y
                arm_rotate = True
            elif joy_subscriber.axes[3] != 0:
                reference_angle += arm_speed_y * joy_subscriber.axes[3]
                arm_rotate = True


            reference_position_backup_x = reference_position.x
            reference_position_backup_z = reference_position.z

            if active_keys["i"] and not active_keys["k"]:
                reference_position.x += arm_speed_x
                arm_translate = True
            elif active_keys["k"] and not active_keys["i"]:
                reference_position.x -= arm_speed_x
                arm_translate = True
            elif joy_subscriber.axes[4] != 0:
                reference_position.x += arm_speed_x * joy_subscriber.axes[4]
                arm_translate = True

            if active_keys["o"] and not active_keys["u"]:
                reference_position.z += arm_speed_z
                arm_translate = True
            elif active_keys["u"] and not active_keys["o"]:
                reference_position.z -= arm_speed_z
                arm_translate = True
            elif joy_subscriber.axes[2] - joy_subscriber.axes[5] != 0:
                reference_position.z += arm_speed_z * (joy_subscriber.axes[2] - joy_subscriber.axes[5]) / 2
                arm_translate = True

            
            # Only do IK if there was a translation
            if arm_translate:
                joint_state_ik_backup.header = joint_state_ik.header
                joint_state_ik_backup.name = joint_state_ik.name
                joint_state_ik_backup.position = joint_state_ik.position
                joint_state_ik_backup.velocity = joint_state_ik.velocity

                joint_state_ik = compute_ik(joint_state_subscriber.joint_state, reference_position)

                # No solution
                # Revert the changes to the reference position to unstuck it
                # Then escape the try block by raising an exception
                if len(joint_state_ik.position) == 0 or joint_state_ik.position[1] < -0.5 or joint_state_ik.position[3] < -0.5 or joint_state_ik.position[3] > 0.5:
                    reference_position.x = reference_position_backup_x
                    reference_position.z = reference_position_backup_z
                    joint_state_ik = joint_state_ik_backup
                    arm_rotate = False
                    arm_translate = False


            if arm_rotate or arm_translate:
                goal_arm = Float64MultiArray()
                goal_arm.data = [0, *joint_state_subscriber.joint_state.position[2:6]]

                end_position = [reference_angle] + list(joint_state_ik.position[1:4])

                # Create trajectory by linearly interpolating between current state and end_position
                for i in range(1, arm_trajectory_points + 1):
                    goal_arm.data.append(arm_motion_plan_duration * i)
                    for j in range(4):
                        goal_arm.data.append(end_position[j] * (i - 1) / (arm_trajectory_points - 1) + joint_state_subscriber.joint_state.position[j + 2] * (arm_trajectory_points - i) / (arm_trajectory_points - 1))
                    
                arm_publisher.publish(goal_arm)
            

            # Return to home position
            if active_keys["h"] or joy_subscriber.buttons[3] == 1:
                print("Started home trajectory")
                
                goto(home_state_position, joint_state_subscriber, plan_kinematic_path, arm_publisher, rate)

                joint_state_ik.position = home_state_position
                joint_state_ik.velocity = [0, 0, 0, 0]

                reference_position = compute_fk(joint_state_ik)
                reference_position.y = 0

                reference_angle = 0
                
                print("Ready")
            

            # Pickup object
            if active_keys["p"] or joy_subscriber.buttons[1] == 1:
                print("Started pickup")

                active_keys["p"] = False
                joy_subscriber.buttons[1] = 0

                goto(home_state_position, joint_state_subscriber, plan_kinematic_path, arm_publisher, rate)
                
                goal_gripper.data = [gripper_max]
                gripper_publisher.publish(goal_gripper)

                goto(pickup_state_position, joint_state_subscriber, plan_kinematic_path, arm_publisher, rate)

                start_time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - start_time < 1:
                    rate.sleep()

                goal_gripper.data = [-1]
                gripper_publisher.publish(goal_gripper)

                start_time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - start_time < 1:
                    rate.sleep()

                goto(home_state_position, joint_state_subscriber, plan_kinematic_path, arm_publisher, rate)

                print("Ready")
            

            # Place object
            if active_keys["v"] or joy_subscriber.buttons[2] == 1:
                print("Started place")

                active_keys["v"] = False
                joy_subscriber.buttons[2] = 0

                goto(home_state_position, joint_state_subscriber, plan_kinematic_path, arm_publisher, rate)

                goal_gripper.data = [-1]
                gripper_publisher.publish(goal_gripper)

                goto(pickup_state_position, joint_state_subscriber, plan_kinematic_path, arm_publisher, rate)

                start_time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - start_time < 1:
                    rate.sleep()

                goal_gripper.data = [gripper_max]
                gripper_publisher.publish(goal_gripper)

                start_time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - start_time < 1:
                    rate.sleep()

                goto(home_state_position, joint_state_subscriber, plan_kinematic_path, arm_publisher, rate)

                print("Ready")
            

            # Throw object
            if active_keys["t"] or joy_subscriber.buttons[6] == 1:
                print("Started throw")

                active_keys["t"] = False
                joy_subscriber.buttons[6] = 0

                goal_gripper.data = [-1]
                gripper_publisher.publish(goal_gripper)

                goto(home_state_position, joint_state_subscriber, plan_kinematic_path, arm_publisher, rate)

                goto(throw_state_position, joint_state_subscriber, plan_kinematic_path, arm_publisher, rate)

                goal_arm = Float64MultiArray()
                for i in range(20):
                    goal_arm.data.extend([0.1 * (i + 1), *pickup_state_position])
                arm_publisher.publish(goal_arm)

                start_time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - start_time < 0.7:
                    rate.sleep()

                goal_gripper.data = [gripper_max]
                gripper_publisher.publish(goal_gripper)

                start_time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - start_time < 2:
                    rate.sleep()

                goal_arm.data = [0, *pickup_state_position]
                for i in range(1, arm_trajectory_points + 1):
                    goal_arm.data.append(arm_motion_plan_duration * i)
                    for j in range(4):
                        goal_arm.data.append(home_state_position[j] * (i - 1) / (arm_trajectory_points - 1) + joint_state_subscriber.joint_state.position[j + 2] * (arm_trajectory_points - i) / (arm_trajectory_points - 1))
                    
                arm_publisher.publish(goal_arm)

                print("Ready")


            gripper_action = None
            
            if active_keys["."] and not active_keys[","] or active_keys[":"] and not active_keys[";"] or joy_subscriber.buttons[5] == 1 and joy_subscriber.buttons[4] == 0:
                gripper_action = -1
            elif active_keys[","] and not active_keys["."] or active_keys[";"] and not active_keys[":"] or joy_subscriber.buttons[4] == 1 and joy_subscriber.buttons[5] == 0:
                gripper_action = 1
            
            if gripper_action:
                gripper_end = joint_state_subscriber.joint_state.position[6] + gripper_action * gripper_speed
                if gripper_end > gripper_max:
                    gripper_end = gripper_max

                goal_gripper.data = [gripper_end]
                gripper_publisher.publish(goal_gripper)
        
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
