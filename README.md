# turtlebot_teleoperation

ROS node that performs real-time teleoperation of turtlebot3 waffle_pi. Wheels, arm and gripper can be controlled in parallel and respond directly to inputs without any planning.

The arm is controlled by moving a reference point for the end effector based on control inputs, and then using the moveit package for performing inverse kinematics at every change. The joint states are sent directly to the control system on the turtlebot.

Currently requires multiple nodes from [isar-turtlebot](https://github.com/equinor/isar-turtlebot). Easiest approach is to run:

```
roslaunch isar_turtlebot turtlebot_manipulator.launch
```

which also starts a simulator and planning windows. The turthebot3_manipulation_navigation.rviz must be kept open for the nodes to run, but Gazebo and moveit.rviz can be closed if not needed.

Run the node:

```
rosrun turtlebot_teleoperation teleop.py
```

## Control mappings:

### Keyboard:
* `WASD`: Wheel velocities
* `IJKL`: Move reference of arm end effector in the horizontal plane
* `UO`: Move reference of arm end effector along the vertical axis
* `,.`: Close and open the gripper

Uses the package `pynput` to handle keyboard inputs. Can be installed with `pip`.

### Xbox controller:
* `Left-stick`: Wheel velocities
* `Right-stick` Move reference of arm end effector in the horizontal plane
* `Triggers`: Move reference of arm end effector along the vertical axis
* `Bumpers`: Close and open the gripper

Xbox controller requires the ROS node joy/joy_node to be run:

```
rosrun joy joy_node
```