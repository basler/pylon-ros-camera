Magazino PylonCamera powered by Basler
=====

This package offers many functions of the Basler-PylonAPI in the ROS-Framwork.
Images can continiously beeing published over /image_raw or /image_rect topics.
Furthermore an Action-based image grabbing with desired exposure/brightness, gain and gamma is provided.

The default node operates in Software-Trigger Mode.

The package opens either a predefined camera or if no camera is defined the first camera device it can find.

Installation
---

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
    wget --quiet -O - https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

    sudo apt-get update
    sudo apt-get install git ros-indigo-ros-base python-catkin-tools htop tmux searchmonkey openssh-server pgadmin3 gdmap

# Parameter
 - camera_frame: frame to which the camera is attached
 - start_exposure: exposure which is set to the camera by starting
 - desired_framerate: if the desired framerate can't be reached, this value will be shrinked
 - target_type: Chose between 'BRIGHTNESS' or 'EXPOSURE'

# Using HDR
 - Install OpenCV 3
 - Use these parameters in launch-file:

```
#!bash

<param name="output_hdr_img" value="true" type="bool" />
<param name="use_sequencer" value="true" type="bool" />
<rosparam param="desired_seq_exp_times">[1000, 2000, 3000]</rosparam> <!-- 1000,2000,3000 exposure times for HDR -->
```



Usage
---
     roslaunch pylon_camera default_camera.launch