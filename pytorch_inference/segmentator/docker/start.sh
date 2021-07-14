#!/bin/bash

orange=`tput setaf 3`
reset_color=`tput sgr0`

if command -v nvidia-smi &> /dev/null
then
    echo "Running on ${orange}nvidia${reset_color} hardware"
    ARGS="--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all"
else
    echo "Running on ${orange}intel${reset_color} hardware: nvidia driver not found"
    ARGS="--device=/dev/dri:/dev/dri"
fi

xhost +
docker run -itd --rm \
    $ARGS \
    --net host \
    --ipc host \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v `pwd`/../:/home/docker_segmentator/catkin_ws/src/segmentator \
    --name segmentator \
    segmentator:latest
xhost -

docker exec --user root \
    segmentator bash -c "/etc/init.d/ssh start"
