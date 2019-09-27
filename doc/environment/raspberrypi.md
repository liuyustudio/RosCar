# Install ROS Car on Raspberry Pi

<!-- TOC -->

- [1. Basic Info](#1-basic-info)
    - [1.1. Reference](#11-reference)
    - [1.2. Target](#12-target)
- [2. Config Environment](#2-config-environment)
    - [2.1. Install ROS](#21-install-ros)
        - [2.1.1. Prerequisites](#211-prerequisites)
        - [2.1.2. Installation](#212-installation)
        - [2.1.3. Resolv Dependencies](#213-resolv-dependencies)
        - [2.1.4. Building the catkin Workspace](#214-building-the-catkin-workspace)
    - [2.2. install 3rd party](#22-install-3rd-party)
- [3. Install ROS Car](#3-install-ros-car)
- [start ROS APPs](#start-ros-apps)
    - [environment](#environment)

<!-- /TOC -->

## 1. Basic Info

### 1.1. Reference

- Installing ROS Kinetic on the Raspberry Pi ([http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi))
- Install Boost 1.58 [https://www.boost.org/doc/libs/1_58_0/more/getting_started/unix-variants.html](https://www.boost.org/doc/libs/1_58_0/more/getting_started/unix-variants.html)

### 1.2. Target

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

## 2. Config Environment

### 2.1. Install ROS

#### 2.1.1. Prerequisites

Setup ROS Repositories

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt update
sudo apt upgrade -y
```

Install Bootstrap Dependencies

```sh
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
```

Initializing rosdep

```sh
sudo rosdep init
rosdep update
```

#### 2.1.2. Installation

Create a catkin Workspace

```sh
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
```

fetch the core packages

```sh
rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init src kinetic-ros_comm-wet.rosinstall
```

#### 2.1.3. Resolv Dependencies

Unavailable Dependencies

```sh
mkdir -p ~/ros_catkin_ws/external_src
cd ~/ros_catkin_ws/external_src
wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
unzip assimp-3.1.1_no_test_models.zip
cd assimp-3.1.1
cmake .
make
sudo make install
```

Resolving Dependencies with rosdep

```sh
# Raspbian Buster:
cd ~/ros_catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:buster
```

#### 2.1.4. Building the catkin Workspace

Install Boost 1.58

```sh
mkdir -p /tmp/boost
cd /tmp/boost
wget http://sourceforge.net/projects/boost/files/boost/1.58.0/boost_1_58_0.tar.bz2
tar jxf boost_1_58_0.tar.bz2
cd boost_1_58_0
./bootstrap.sh
./b2 install
```

Invoke catkin_make_isolated

```sh
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic
```

setup.bash in the ~/.bashrc

```sh
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```

### 2.2. install 3rd party

```sh
# rapidjson
apt install -y rapidjson-dev

# install GUN's readline library
apt install -y libreadline-dev
```

## 3. Install ROS Car

download source code(from master brance).

```sh
mkdir -p ~/workspace/rosCar
git clone https://github.com/liuyustudio/RosCar.git ~/workspace/rosCar
```

compile

```sh
cd ~/workspace/rosCar/car
catkin_make
```

## start ROS APPs

### environment

- boost library

if start roscore fail and error message looks like this:

```txt
process[rosout-1]: started with pid [16014]
/opt/ros/kinetic/lib/rosout/rosout: error while loading shared libraries: libboost_filesystem.so.1.58.0: cannot open shared object file: No such file or directory
[rosout-1] process has died [pid 16014, exit code 127, cmd /opt/ros/kinetic/lib/rosout/rosout __name:=rosout __log:=/home/liuyu/.ros/log/8ee223fa-e135-11e9-b98a-b827eb5a735d/rosout-1.log].
log file: /home/liuyu/.ros/log/8ee223fa-e135-11e9-b98a-b827eb5a735d/rosout-1*.log
[rosout-1] restarting process
```

that's means roscore can't fild libboost_filesystem.so.1.58.0

solution:

```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```
