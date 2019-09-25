# Frequently Asked Questions

<!-- TOC -->

- [1. build/compile](#1-buildcompile)
    - [1.1. Hot to put headers into devel/include filder with catkin_make](#11-hot-to-put-headers-into-develinclude-filder-with-catkin_make)
    - [1.2. build debug version](#12-build-debug-version)
- [2. start node](#2-start-node)
    - [2.1. docker container](#21-docker-container)
        - [2.1.1. create docker container](#211-create-docker-container)
        - [2.1.2. debug in docker container](#212-debug-in-docker-container)

<!-- /TOC -->

## 1. build/compile

### 1.1. Hot to put headers into devel/include filder with catkin_make

[https://answers.ros.org/question/150306/how-to-put-headers-into-develinclude-folder-with-catkin_make/](https://answers.ros.org/question/150306/how-to-put-headers-into-develinclude-folder-with-catkin_make/)

- CMakeLists.txt

  - library package:

    ```cmake
    install(DIRECTORY include/${PROJECT_NAME}/
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

    catkin_package(INCLUDE_DIRS include)
    include_directories(include)
    ```

  - downstream packages:

    ```cmake
    find_package(catkin REQUIRED COMPONENTS your_package)
    include_directories(${catkin_INCLUDE_DIRS})
    ```

- include folder

  put all header files into 'include' folder, as shown below:

  ```txt
  .
  ├── CMakeLists.txt
  ├── include
  │   └── roscar_common
  │       ├── const.h -> ../../src/const.h
  │       ├── error.h -> ../../src/error.h
  │       └── rcmp.h -> ../../src/rcmp.h
  ├── package.xml
  └── src
      ├── const.h
      ├── error.h
      ├── rcmp.cpp
      └── rcmp.h
  ```

### 1.2. build debug version

ref: [http://www.liuyu.com/linux/ssh/2019/08/10/debug-ros-node-by-gdb.html](http://www.liuyu.com/linux/ssh/2019/08/10/debug-ros-node-by-gdb.html)

```sh
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

## 2. start node

### 2.1. docker container

#### 2.1.1. create docker container

- enable debug version

    ```sh
    docker run -it --hostname rosCar --name rosCar --cap-add=SYS_PTRACE --security-opt seccomp=unconfined -v /workspace/project/rosCar/car:/car roscar:ros-kinetic-base bash
    ```

- normal version

    ```sh
    docker run -it --hostname rosCar --name rosCar -v /workspace/project/rosCar/car:/car roscar:ros-kinetic-base bash
    ```

#### 2.1.2. debug in docker container

```sh
rosrun --prefix 'gdb -ex run --args' interface interface
```
