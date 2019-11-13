- [Dependency Installation](#dependencies)
- [Building](#building)
- [Usage](#usage)

### Dependencies
#### Local Docker engine setup
Install [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) (you may want to configure your user permissions so you don't have to use sudo with every docker command -- see [here](https://docs.docker.com/install/linux/linux-postinstall/))
```
sudo apt-get remove docker docker-engine docker.io containerd runc
sudo apt-get update
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
```

Install [nvidia-container-runtime](https://github.com/nvidia/nvidia-container-runtime#docker-engine-setup) (main instructions from the link shown below)

```
sudo apt-get install nvidia-container-runtime
```
Follow the steps for creating a drop in file to register the runtime with docker

```
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo tee /etc/systemd/system/docker.service.d/override.conf <<EOF
[Service]
ExecStart=
ExecStart=/usr/bin/dockerd --host=fd:// --add-runtime=nvidia=/usr/bin/nvidia-container-runtime
EOF
sudo systemctl daemon-reload
sudo systemctl restart docker
```

## Building
### Pull the image from dockerhub
```
docker pull anthonysimeonov/airobot-dev:latest
```

### Building the image locally
From within this directory (```/path/to/airobot/docker/```), run the following command to build the image (this will take quite a bit of time if you have not done it locally before)
```
python docker_build.py
```

(if you are building the image locally you will need to set up your machine with public keys linked to your github account for cloning private repositories required during building)

## Usage
### Running the container
Open two terminals

#### Terminal 1
In the first terminal, from within this directory (```/path/to/airobot/docker/```), run the ```run_image.bash``` script to launch the container. You will enter a terminal session from within the container after launching.
```
./run_image.bash
```

then setup ```airobot``` from within the container
```
cd /home/improbable/airobot/
python setup.py install
```

#### Termainal 2
From the second terminal, run the following bash script to allow the container access to your local X server for visualization (there are better ways to do this, but we keep this relatively secure by only providing access to xhost to the specific system we want)

```
./visualize_access.bash
```

You can now run programs from within the container that are GUI-based, and have the GUI appear
on your host screen (i.e. Gazebo, RViz, PyBullet)

First, enter the container by starting a new interactive terminal session (```$CONTAINER_ID``` can usually be found by tab completing the below command, or viewing the information about the running containers by running ```docker ps```) and source the catkin workspace that has been built with the following commands
```
docker exec -it $CONTAINER_ID bash
```
(now inside of the container)
```
cd /root/catkin_ws/
source devel/setup.bash
```

Then, for instance, you can launch a simulated UR5e in Gazebo, and start MoveIt! with it, with the following command

```
roslaunch ur5e_bringup ur5e_start.launch sim:=true
```

Or you can launch a simulated UR5e in PyBullet with ```airobot``` with the following commands
```
cd /home/improbable/airobot/examples/ur5e/sim/
python joint_position_control.py
```

You can also connect to the real robot and control it via the ```airobot``` API functions (```<robot_ip>``` can be viewed from the PolyScope interface on the real robot's teach pendant. First bringup the connection to the robot with the following command, after connecting to the local network via ethernet cable.

```
roslaunch ur5e_bringup ur5e_start.launch robot_ip:=<robot_ip>
```
Then run the following command to run the ```joint_position_control``` example (from a new terminal, either using ```tmux``` or by entering the container from a new local terminal with the same ```docker exec``` command above)
```
cd /home/improbable/airobot/examples/ur5e/real/
python joint_position_control.py
```
