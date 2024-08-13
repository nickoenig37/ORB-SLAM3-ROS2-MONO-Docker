# ORB-SLAM3 ROS2 Wrapper Docker

This repository contains a dockerized comprehensive wrapper for ORB-SLAM3 using a monocular camera on ROS 2 Humble for Ubuntu 22.04. This repository was adapted from a dockerized rgbd implementation of ORB-SLAM3 at <https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker>

# Demo GIF

![ORBSLAM3-GIF](ORBSLAM_Moving.gif)

# Steps to use this wrapper

## 1. Clone this repository

```bash
    git clone <copy https or ssh repository location>
    cd ORB-SLAM3-ROS2-MONO-Docker
    git submodule update --init --recursive --remote
```

## 2. Install Docker on your system (if not already installed)

```bash
cd ORB-SLAM3-ROS2-MONO-Docker
sudo chmod +x container_root/shell_scripts/docker_install.sh
./container_root/shell_scripts/docker_install.sh
```

## 3. Build the image with ORB_SLAM3

1. Build the image: ```sudo docker build -t orb-slam3-humble:22.04 .```
1. Add ```xhost +``` to your ```.bashrc``` to support correct x11-forwarding using ```echo "xhost +" >> ~/.bashrc```
1. ```source ~/.bashrc```
1. You can see the built images on your machine by running ```sudo docker images```.

## 4. Running the container

1. ```cd ORB-SLAM3-ROS2-Docker``` (ignore if you are already in the folder)
2. ```sudo docker compose run orb_slam3_22_humble```
3. This should take you inside the container. Once you are inside, run the command ```xeyes``` and a pair of eyes should pop-up. If they do, x11 forwarding has correctly been setup on your computer. (Ignore the message:

```bash
bash: /root/colcon_ws/install/setup.bash: No such file or directory
```

It should go away once you complete the next step.)

## 5. Building the ORB-SLAM3 Wrapper

Launch the container using steps in (4).

```bash
cd /root/colcon_ws/
colcon build --symlink-install
```

The build process may emit warnings associated with `-Wreorder`, `-Wunused-parameter` and similar compiler flags.
You can ignore them.

If there are no errors (in addition to warnings above) then you can source the install script"

```bash
source install/setup.bash
```

# Container Information (Editing and Launching Files)

## Launching ORB-SLAM3

Launch the container using steps in (4).
If you are inside the container, run the following:

```bash
ros2 launch orb_slam3_ros2_wrapper rgb_unirobot.launch.py
```

## Interfacing ORB-SLAM3 nodes with external ROS2 processes

To make ORB-SLAM3 read from an external ROS2 simulation (or hardware teleoperation), one needs to ensure that the Docker container running ORB-SLAM3 and the external ROS2 are communicating correctly.

Presently, this has been configured using Cyclone DDS only.

1. Make sure that both the ORB-SLAM3 container and the external ROS2 have the same `ROS_DOMAIN_ID` set-up.

1. ORB-SLAM3 container is already configured to use Cyclone DDS.
    The external ROS2 has to have the following steps configured, as detailed in <https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker/issues/8#issuecomment-2187977113>

1. Install Cyclone DDS:

  ```bash
  sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp 
  ```

1. Create the Cyclone DDS configuration file in user's home folder (`~/.ros/cyclonedds.xml`) containing the following configuration:

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config"
      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
      <Domain id="any">
          <General>
              <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
              <AllowMulticast>false</AllowMulticast>
          </General>
          <Discovery>
              <ParticipantIndex>auto</ParticipantIndex>
              <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
              <Peers>
                  <Peer Address="localhost" />
              </Peers>
          </Discovery>
      </Domain>
  </CycloneDDS>
  ```

1. Set up the following environment variables, e.g., in `.bashrc` add:

  ```bash

  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=~/.ros/cyclonedds.xml

  ```

1. Setup the ORB-SLAM3 ROS2 Docker using the steps above. Once you do (1) step in the ```Launching ORB-SLAM3``` section, you should see a window popup which is waiting for images. This is partially indicative of the setup correctly done. However, I recommend you run your simulation first and then run the ORB-SLAM3 wrapper for monocular.

- NOTE: for a monocular camera, you need multiple keyframes to begin tracking. So, you may need to move the camera around a bit to get the ORB-SLAM3 to start tracking.

If you're having issues interfacing between your simulation and the ROS2 communication side of things see the potential issues section below!

## Making Changes & Managing the Workspace

- For starters, all changed code is in the ```orb_slam3_ros2_wrapper``` directory. This is where you will be making changes to the ORB-SLAM3 wrapper, the ```ORB_SLAM3``` directory is the original codebase and should not be modified.
- For managing sensors being used by the algorithm, see the "ROS Parameter descriptions" section below. These paramater files are initialized in the `reg_rgb.launch.py` file.
- For changing topic names to subscribe to, go to orb_slam3_ros2_wrapper/src/monocular/rgb-slam-node.cpp
- The algorithm is much better when using your own camera calibration file. I recommend following these instructions to calibrate the camera: `https://docs.ros.org/en/rolling/p/camera_calibration/tutorial_mono.html`
  - Take the values this calibration gave you and make the corresponding changes to the format of an existing file. If you don't understand the format value differences between files, I recommend giving both of them to GPT and asking it to write your values to the correct file format.
  - For calibrating your camera in gazebo I recommend following this link: `https://medium.com/@arshad.mehmood/camera-calibration-in-gazebo-ros2-6bed2620a652`

### Potential issues you may face

The simulation and the wrapper both have their ```ROS_DOMAIN_ID``` set to 55 so they are meant to work out of the box. However, you may face issues if this environment variable is not set properly. Before you start the wrapper, run ```ros2 topic list``` and make sure the topics namespaced with ```scout_2``` are visible inside the ORB-SLAM3 container provided the simulation is running along the side.

## ROS Parameter descriptions

| Parameter Name          | Default Value | Description                                                                 |
|-------------------------|---------------|-----------------------------------------------------------------------------|
| `robot_base_frame`      | `base_footprint` | The name of the frame attached to the robot's base. |
| `global_frame`          | `map`         | The name of the global frame of reference. It represents a fixed world coordinate frame in which the robot navigates.|
| `odom_frame`            | `odom`        | The name of the odometry frame. |
| `robot_x`               | `0.0`         | The robot's initial x-coordinate in the global frame. Specifies the starting position along the x-axis. The SLAM Wrapper will assume this to be the initial x position|
| `robot_y`               | `0.0`         | The robot's initial y-coordinate in the global frame. Specifies the starting position along the y-axis. The SLAM Wrapper will assume this to be the initial y position|
| `visualization`         | `true`        | A boolean flag to enable or disable visualization. When set to `true`, the ORB-SLAM3 viewer will show up with the tracked points and the keyframe trajectories.|
| `ros_visualization`     | `false`       | A boolean flag to control ROS-based visualization. If set to `true`, it enables ROS tools like RViz to visualize the robot's data. (3D position of the tracked points etc.)  **This feature is unstable and not tested as of now**|
| `no_odometry_mode`      | `false`       | A boolean flag to toggle odometry mode. When `true`, the system operates without relying on odometry data, which might be used in scenarios where odometry information is unavailable or unreliable. In this case, it publishes the transform directly between the ```global_frame``` and the ```robot_base_frame```|
