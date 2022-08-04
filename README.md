# turtlebot_teleoperation

ROS node that performs real-time teleoperation of turtlebot3 waffle_pi. Wheels, arm and gripper can be controlled in parallel and respond directly to inputs.

The arm is controlled by moving a reference point for the end effector based on control inputs, and then using the moveit package for performing inverse kinematics at every change. The joint states are sent directly to the control system on the turtlebot.

Currently requires multiple ROS nodes to be run prior. Start by running:

```
roscore
```

on the remote computer. Then run the following command on the turtlebot:

```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

Then launch the following two launch files on the remote computer:

```
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch

roslaunch isar_turtlebot_moveit_config move_group.launch
```

The final move_group launch file is from [isar-turtlebot](https://github.com/equinor/isar-turtlebot).


Then run the teleoperation node:

```
rosrun turtlebot_teleoperation teleop.py
```

## Control mappings:

### Keyboard:
* `WASD`: Wheel velocities
* `IJKL`: Move reference of arm end effector in the horizontal plane
* `UO`: Move reference of arm end effector along the vertical axis
* `,.`: Close and open the gripper
* `h`: Reset to home position
* `p`: Pick up object
* `v`: Place down object
* `t`: Throw held object
* `SHIFT`: Detect object and move to position
* `ENTER`: Deactivate teleoperation

Uses the package `pynput` to handle keyboard inputs. Can be installed with `pip`.

### Xbox one controller:
* `Left-stick`: Wheel velocities
* `Right-stick` Move reference of arm end effector in the horizontal plane
* `Triggers`: Move reference of arm end effector along the vertical axis
* `Bumpers`: Close and open the gripper
* `Y`: Reset to home position
* `B`: Pick up object
* `X`: Place down object
* `View`: Throw held object
* `A`: Detect object and move to position
* `Menu`: Deactivate teleoperation

Xbox controller requires the ROS node joy/joy_node to be run:

```
rosrun joy joy_node
```