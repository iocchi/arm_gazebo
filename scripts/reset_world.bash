#!/bin/bash

python gazebo_objects.py -d ../config/ARMd.txt
python gazebo_objects.py -a $1
