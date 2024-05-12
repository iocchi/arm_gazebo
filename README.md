# ARM Gazebo

This package contains files for Gazebo simulation for RoboCup ARM challenge.

# Setup

> [!NOTE]
> Skip this section if you downloaded the ARM Gazebo Virtual Machine.

* Install docker engine (not docker Desktop!!!)  (tested on v. 19.03, 20.10) 

    Usually, this should work

    ```bash    
    sudo apt install docker.io
    ```

    or install from binaries https://docs.docker.com/engine/install/binaries/

    See also 
    [Post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall/).
    In particular, add your user to the `docker` group and log out and in again, before proceeding.

    ```bash
    sudo usermod -aG docker $USER
    ```

* Install docker-compose (tested on v. 1.28)

    First remove any other `docker-compose` file, if present (check with `which docker-compose`)

    Download binary file for v. 1.28.5

    ```bash
    cd /usr/local/bin
    sudo wget https://github.com/docker/compose/releases/download/1.28.5/docker-compose-Linux-x86_64
    sudo mv docker-compose-Linux-x86_64 docker-compose
    sudo chmod a+x docker-compose
    docker-compose -v
    ```

* Nvidia driver

    Install nvidia-docker2

    [NVidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)


* Additional Tools
    ```bash
    sudo apt install gawk
    ```


# Build

> [!NOTE]
> Skip this section if you downloaded the ARM Gazebo Virtual Machine.

Build docker image

```bash	
cd docker
./build.bash
```

# Run

```bash
cd docker
./run.bash [x11|nvidia|vnc]
```

`x11` default mode uses X11 server, `nvidia` if you have NVidia graphic card and you installed nvidia-docker, `vnc` if you have problems with other modes uses a virtual xserver accessible through a web browser at `http://localhost:3000/`

> [!NOTE]
> if you are using the ARM Gazebo Virtual Machine, run in standard mode `./run.bash`


## Explanation of the startup procedure

The script `run.bash` use `docker-compose` to start the docker containers with the configuration given in the files `docker/dc_<*>.yml`

To see the running docker containers, use the command

```bash
docker ps
```

The container `armgazebo` is the one that is running the ARM gazebo simulation

The container is running a [tmux](https://github.com/tmux/tmux/wiki) session. You can attach to this session with the following command

```bash
docker exec -it armgazebo tmux a
```

The container initialize the gazebo simulation by running the script `bin/init_sim.bash`.
In this script you can find the commands sent to the tmux server, in particular the following windows are created: 

0. gazebo simulator
1. moveit
2. rviz (command prepared but not launched),
3. scripts to spawn objects.

> [!NOTE]
> You can create additional tmux windows with <kbd>Ctrl</kbd>+<kbd>b</kbd> <kbd>c</kbd>. You can detach from tmux with <kbd>Ctrl</kbd>+<kbd>b</kbd> <kbd>d</kbd>






# Configure and spawn objects

Create a config file in `config` folder with objects described in this format

```
 <object_name> <type> <x> <y> <z> <yaw> <pitch> <roll>
```

Spawn objects from the docker container

First enter the docker container

```bash
docker exec -it armgazebo tmux a
```

Then, issue the following commands to manage objects in gazebo simulator

```bash
cd ~/src/arm_gazebo/scripts
python gazebo_objects.py -a <object config file>
```

For example:

```bash
python gazebo_objects.py -a ../config/ARM1.txt
```

You can add more config files in the `config` folder

To reset a world, you can use the script

```bash
./reset_world.bash <object config file>
```

This script will first delete all the objects according to the patterns
in file `config/ARMd.txt` and then spawn the new objects in the specified
config file.

For example, this command will reset the world with the configuration `ARM1.txt`

```bash
./reset_world.bash ../config/ARM1.txt
```

For other interactions with gazebo objects, see a list of options with

```bash
python gazebo_objects.py -h

Options
-h	help
-l	list objects
-m	list available models
-a <obj> <type> <x> <y> <z> <yaw> <pitch> <roll>|<filename>	add an object or all objects in config file
-d <obj>|<filename>	delete one object or all objects in config file
-w	world properties
-s <obj>	object properties and state
```

# Rviz visualization

From the docker container

```bash
docker exec -it armgazebo tmux a
```

you can create a new tmux windows with <kbd>Ctrl</kbd>+<kbd>b</kbd> <kbd>c</kbd> and then run

```bash
cd ~/src/arm_gazebo/config
rosrun rviz rviz -d robot_camera.rviz
```

You can detach from tmux with <kbd>Ctrl</kbd>+<kbd>b</kbd> <kbd>d</kbd>

# Troubleshooting

* In case of errors due to existing containers, use this command to clean them

    ```bash
    docker container prune -f
    ```

