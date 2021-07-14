#!/bin/bash

export ARCH=`uname -m`

xhost +
docker run -itd --rm \
    --ipc host \
    --net host \
    -e "DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device=/dev/dri:/dev/dri \
    -p ${PORT}:22 \
    -v `pwd`/../:/home/docker_segmentator/catkin_ws/src/segmentator \
    -v /home/${USER}:/home/${USER}:rw \
    --name segmentator \
    ${ARCH}melodic/segmentator:latest
xhost -

docker exec --user root \
    segmentator bash -c "/etc/init.d/ssh start"
