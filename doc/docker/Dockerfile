FROM ros:kinetic-ros-base-xenial

# install ros tutorials packages
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y ros-kinetic-common-tutorials \
        ros-kinetic-ros-tutorials

# install GUN's readline library
RUN apt-get install libreadline-dev -y

# install rapid json
ADD https://github.com/Tencent/rapidjson/archive/v1.1.0.tar.gz /tmp
RUN cd /tmp && \
    tar zxf v1.1.0.tar.gz && \
    cd rapidjson-1.1.0 && mkdir build && cd build && \
    cmake .. && \
    make -j && make install

# add user
RUN groupadd -g 1000 liuyu \
    && useradd -m -u 1000 -g liuyu liuyu

# set default environment
USER liuyu
WORKDIR /home/liuyu
