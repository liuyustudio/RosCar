FROM ros:kinetic-ros-base-xenial

# add user
RUN groupadd -g 1000 liuyu \
    && useradd -m -u 1000 -g liuyu liuyu

# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-tutorials \
    ros-kinetic-common-tutorials \
    && rm -rf /var/lib/apt/lists/

USER liuyu
WORKDIR /home/liuyu
