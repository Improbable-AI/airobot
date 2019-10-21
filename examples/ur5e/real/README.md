## Launch the UR5e driver

If you haven't installed it, you can follow the instructions [here](https://github.com/Improbable-AI/ur5e_robotiq_2f140)

```bash
roslaunch ur5e_bringup ur5e_start robot_ip:=<robot_ip>
```

## Launch the realsense camera driver if needed

If you haven't installed it, you can find instructions [here](https://github.com/Improbable-AI/camera_calibration/tree/qa) or [here](https://github.com/IntelRealSense/realsense-ros)

```bash
roslaunch realsense2_camera rs_rgbd.launch
```

After these two steps, you can run the example files.