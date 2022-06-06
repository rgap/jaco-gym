OpenAI Gym environment for the Jaco2 robotic arm by Kinova.

This was tested on ROS Melodic, Ubuntu 18.04, and Python 3.

You could follow the [original repository](https://github.com/PierreExeter/jaco-gym/) to install both ROS Melodic and [Kinova-ROS](https://github.com/Kinovarobotics/kinova-ros).

If you are using Pycharm, then run it from a terminal.


## How to run the sample code

In terminal 1

```
roscore
```

In terminal 2

```
cd ~/catkin_ws; catkin_make; source devel/setup.bash; cd "$OLDPWD";
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300
```

In terminal 3

Assuming you are inside jaco-gym/test

```
cd ~/catkin_ws; catkin_make; source devel/setup.bash; cd "$OLDPWD";
python test_physicalrobot.py
```

## Useful ROS commands

This shows the current state (joint angles)
```
rostopic echo /j2n6s300_driver/out/joint_state
```

This is used for running gazebo. I couldn't get it to work correctly.

```
roslaunch kinova_gazebo robot_launch_render.launch kinova_robotType:=j2n6s300
```

## Useful links

```
https://usermanual.wiki/Pdf/JACOC2B26DOFAdvancedSpecificationGuide.1434949807.pdf
https://assistive.kinovarobotics.com/uploads/EN-UG-007-Jaco-user-guide-R05.pdf
```