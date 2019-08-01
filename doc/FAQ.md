# Frequently Asked Questions

<!-- TOC -->

- [1. build/compile](#1-buildcompile)
    - [1.1. Hot to put headers into devel/include filder with catkin_make](#11-hot-to-put-headers-into-develinclude-filder-with-catkin_make)

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
