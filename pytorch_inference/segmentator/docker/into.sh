#!/bin/bash
docker exec --user "docker_segmentator" -it segmentator \
        /bin/bash -c ". /ros_entrypoint.sh; cd /home/docker_segmentator; /bin/bash"
