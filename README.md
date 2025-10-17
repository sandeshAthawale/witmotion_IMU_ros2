# IMU Workspace Guide

This document captures the exact steps used to bring up the WITMOTION BWT901C(L) IMU on ROS 2 Jazzy inside the `~/imu_ws` workspace. Follow them in order on a fresh machine to reproduce the setup.

## 1. Prepare the ROS 2 workspace

```bash
mkdir -p ~/imu_ws/src
cd ~/imu_ws/src
git clone https://github.com/Arcanain/bwt901cl_pkg.git

# replace these two files with the one present in this github repo
# src/bwt901cl_pkg/bwt901cl_pkg/src/bwt901cl.py
# src/bwt901cl_pkg/bwt901cl_pkg/imu_bwt901cl.py
```

## 2. Source ROS 2 and install dependencies

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

## 3. Build the package

```bash
cd ~/imu_ws
colcon build --packages-select bwt901cl_pkg
source install/setup.bash
```

> Tip: add the `source install/setup.bash` line to your shell profile if you launch the node often.

## 4. Grant serial-port permissions

Identify the port (usually `/dev/ttyUSB0`) and grant access:

```bash
sudo usermod -a -G dialout $USER     # log out/in once after running
sudo chmod a+rw /dev/ttyUSB0        # quick test without relogin
```

## 5. Run the IMU driver

```bash
ros2 run bwt901cl_pkg imu_bwt901cl --ros-args -p port:=/dev/ttyUSB0
```

The node prints current roll/pitch/yaw in degrees. Each run also writes a CSV log to `log/<timestamp>.csv` that captures the remaining telemetry (angular velocity, acceleration, temperature, magnetic field and quaternion).

## 6. Inspect the published data

With the node running, open a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/imu_ws/install/setup.bash
ros2 topic list
```

Useful topics to inspect:

- `/imu_data_raw` – `sensor_msgs/msg/Imu`
- `/imu_angle_deg` – `geometry_msgs/msg/Vector3Stamped` (Euler angles in degrees)
- `/imu_magnetic_field` – `sensor_msgs/msg/MagneticField`
- `/imu_temperature` – `std_msgs/msg/Float32`

Example echoes:

```bash
ros2 topic echo /imu_data_raw
ros2 topic echo /imu_angle_deg
ros2 topic echo /imu_magnetic_field
ros2 topic echo /imu_temperature
```

## 7. Video Walkthrough

Prefer a live demo? Watch the short terminal walkthrough that covers port setup, launch commands, and log inspection:  
[![IMU terminal demo thumbnail](images/imu_video_thumbnail_with_play.svg)](https://drive.google.com/file/d/1VAqRX4KtbbWUm0-9MyCw4_mVqH4EXK9D/view?usp=sharing)


## 8. Visualise in RViz

```bash
rviz2
```

In RViz (Terminal 2):

1. Set the **Fixed Frame** to `base_link`.
2. Add a **TF** display to view the `base_link → imu_link` transform.
3. Add an **Imu** display and target `/imu_data_raw`.

## 9. Recorded logs

All CSV logs land in the `log/` folder (configurable with `--ros-args -p log_dir:=...`). The filename follows `YYYYMMDD-HHMMSS.csv`. Use your preferred tools (e.g. Python pandas) to post-process those logs.

---

You can now integrate the IMU into downstream pipelines (robot localisation, navigation, etc.). When you’re ready to share this workspace, remember to commit `src/` along with this README. Git instructions will follow in the next step.


## Author

Sandesh Athawale<br>
Currently based in Tokyo, Japan 🇯🇵 and working as a System engineer.
