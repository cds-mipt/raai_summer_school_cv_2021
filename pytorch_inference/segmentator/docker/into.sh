#!/bin/bash
docker exec --user "docker_segmentator" -it segmentator \
        /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /home/docker_segmentator; /bin/bash"
