# Install ROS Car on Raspberry Pi

## Target

- target hw:

  Raspberry Pi 3B

- target os:

  ```sh
  lsb_release -a
  No LSB modules are available.
  Distributor ID: Raspbian
  Description:    Raspbian GNU/Linux 10 (buster)
  Release:        10
  Codename:       buster
  ```

## setup environment

### install ROS

Install from source ([http://wiki.ros.org/kinetic/Installation/Source](http://wiki.ros.org/kinetic/Installation/Source))

```sh
apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential

mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

#----------------------------------------------------------------
# # Desktop-Full Install: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception
#
#     $ rosinstall_generator desktop_full --rosdistro kinetic --deps --wet-only --tar > kinetic-desktop-full-wet.rosinstall
#     $ wstool init -j8 src kinetic-desktop-full-wet.rosinstall
#
#----------------------------------------------------------------
# # Desktop Install (recommended): ROS, rqt, rviz, and robot-generic libraries
#
#     $ rosinstall_generator desktop --rosdistro kinetic --deps --wet-only --tar > kinetic-desktop-wet.rosinstall
#     $ wstool init -j8 src kinetic-desktop-wet.rosinstall
#
#----------------------------------------------------------------
# # install ROS-Comm: (Bare Bones)
rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init -j8 src kinetic-ros_comm-wet.rosinstall
```
