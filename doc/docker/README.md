# docker environment for ROS Car

## create docker image

```sh
docker build -t roscar:ros-kinetic-base .
docker tag roscar:ros-kinetic-base roscar:latest
```

## run

```sh
docker run -it --hostname rosCar --name rosCar  -v /workspace/project/rosCar/car:/car roscar:ros-kinetic-base bash
```

### for GDB

reference from: [https://stackoverflow.com/a/46676907](https://stackoverflow.com/a/46676907)


```sh
docker run -it --hostname rosCar --name rosCar --cap-add=SYS_PTRACE --security-opt seccomp=unconfined -v /workspace/project/rosCar/car:/car roscar:ros-kinetic-base bash
```
