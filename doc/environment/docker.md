# ROS Car in Docker

在项目测试及仿真阶段， Docker 容器无疑是最好的测试环境。

## Setup

- pull docker image

    - pull newest stable version

    ```sh
    docker pull ros
    ```

    pull 'kinetic' version

    ```sh
    docker pull ros:kinetic
    ```

- launch docker container

    ```sh
    docker run -it --rm -v /workspace/project/rosCar/car:/car ros:kinetic bash
    ```
