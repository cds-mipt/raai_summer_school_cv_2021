#!/bin/bash

export ARCH=`uname -m`

docker build . \
    -f Dockerfile.cpu \
    --build-arg UID=${UID} \
    --build-arg GID=${UID} \
    -t ${ARCH}melodic/segmentator:latest
