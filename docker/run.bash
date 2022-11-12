#!/bin/bash

export ARM_GAZEBO_DIR=`pwd | gawk '{ print gensub(/\/docker/, "", 1) }'`

DC=dc_x11.yml

if [ "$1" == "nvidia" ]; then
  DC=dc_nvidia.yml
elif [ "$1" == "vnc" ]; then
  DC=dc_vnc.yml
fi

docker-compose -f $DC  up


