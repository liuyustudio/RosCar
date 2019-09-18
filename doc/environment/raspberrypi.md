# Install ROS Car on Raspberry Pi

Reference:

- Installing ROS Kinetic on the Raspberry Pi ([http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi))
- Install Boost 1.58 [https://www.boost.org/doc/libs/1_58_0/more/getting_started/unix-variants.html](https://www.boost.org/doc/libs/1_58_0/more/getting_started/unix-variants.html)

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

## Install ROS

### Prerequisites

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

### Installation

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

### Resolv Dependencies

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

### Building the catkin Workspace

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
