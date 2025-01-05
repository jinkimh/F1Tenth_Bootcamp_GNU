#! /bin/bash

xhost +local:docker

docker run -it \
  --privileged \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd)/f1tenth_ws/src/f1tenth_gym_ros:/sim_ws/src/f1tenth_gym_ros"\
  --volume="$(pwd)/f1tenth_ws/src/f1tenth-software-stack:/sim_ws/src/f1tenth-software-stack"\
  --name f110_gym_docker \
  f1tenth_gym_ros:latest
