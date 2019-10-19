Enter a terminal from within this directory, and run the ```run_image.bash``` script to launch the container
```
./run_image.bash
```

Within another terminal, run the following bash script to allow the container access to your local X server for visualization (there are better ways to do this, but we keep this relatively secure by only providing access to xhost to the specific system we want)

```
./visualize_access.bash
```

Then, from within the first terminal, which has now entered the container, we can use ROS normally and programs will be visualized on our local system (note -- currently, we mount the repo locally using a volume so the workspace in the container has to be built upon the first time launching the container. this behavior will be modified upon release versions of the code)

```
catkin build
source devel/setup.bash
roslaunch ur5e_2f140_gazebo ur5e_2f140_world.launch
```

Then in a new local terminal, run the ```bash``` command inside the docker to open a new terminal inside the container to interact with the robot (```$CONTAINER_ID``` can usually be found by tap completing the below command, or viewing the information about the running containers by running ```docker ps```)

```
docker exec -it $CONTAINER_ID bash
source devel/setup.bash
cd src/ur5e_2f140_gazebo/test
python test_arm_control.py
```
