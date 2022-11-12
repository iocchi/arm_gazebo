# ARM Gazebo

This package contains files for Gazebo simulation for RoboCup ARM challenge.

# Setup

* Install docker engine (not docker Desktop!!!)  (tested on v. 19.03, 20.10) 

    Usually, this should work
    
        sudo apt install docker.io

    or install from binaries

        https://docs.docker.com/engine/install/binaries/

    See also 
    [Post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall/).
    In particular, add your user to the `docker` group and log out and in again, before proceeding.

        sudo usermod -aG docker $USER


* Install docker-compose (tested on v. 1.28)

    First remove any other `docker-compose` file, if present (check with `which docker-compose`)

    Download binay file for v. 1.28.5

        cd /usr/local/bin
        sudo wget https://github.com/docker/compose/releases/download/1.28.5/docker-compose-Linux-x86_64
        sudo mv docker-compose-Linux-x86_64 docker-compose
        sudo chmod a+x docker-compose
        docker-compose -v

* Nvidia driver

    Install nvidia-docker2

    [NVidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)



# Build

Build docker image

    cd docker
    ./build.bash

# Run

    cd docker
    ./run.bash [x11|nvidia|vnc]

`x11` default mode uses X11 server, `nvidia` if you have NVidia graphic card and you installed nvidia-docker, `vnc` if you have problems with other modes uses a virtual xserver accessible through a web browser at `http://localhost:3000/`


# Configure and spawn objects

Create a config file in `config` folder with objects described in this format

    <object_name> <type> <x> <y> <z> <yaw> <pitch> <roll>


Spawn objects from the docker container

    docker exec -it gazebo tmux a
    cd ~/src/arm_gazebo/scripts
    python gazebo_objects.py -a <object config file>

For example:

    python gazebo_objects.py -a ../config/ARM1.txt

For other interactions with gazebo objects, see a list of options with

    python gazebo_objects.py -h


