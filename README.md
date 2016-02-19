Magazino PylonCamera powered by Basler
=======



This package offers many functions of the Basler-PylonAPI inside the ROS-Framwork.

The package supports Baslers USB 3.0, GigE as well as the DART cameras.
Images can continuously be published over /image_raw topic. The camera-characteristic parameter such as hight, width and camera_frame were published over the /camera_info topic.
Furthermore an action-based image grabbing with desired exposure/brightness, gain and gamma is provided.
Hence one can grab a sequence of images with target exposure times each as well as a single image.
The default node operates in Software-Trigger Mode.
This means that the image acquisition is triggered with a certain rate and the camera is not running in the continuous mode.

The package opens either a predefined camera (using a given 'device_user_id' parameter) or, if no camera id is predefined the first camera device it can find.

# Installation
Download and install the latest Version of the PylonAPI from

     http://www.baslerweb.com/de/support/download-uebersicht/downloads-software

Build the PylonCamera package as you would build a standard ROS-package unsing p.e.

     catkin_make

# Parameter

Default parameter in the config file
---

 - device_user_id: name of camera (written to its EPROM e.g. via /opt/pylon5/bin/PylonViewerApp). If empty, a random camera is opened. 
 - camera_frame: frame to which the camera is attached
 - start_exposure: exposure which is set to the camera by starting
 - desired_framerate: if the desired framerate can't be reached, this value will be shrinked
 - [Deprecated?] target_type: Chose between 'BRIGHTNESS' or 'EXPOSURE'
 - [HOW TO ACTIVATE AutoExposure?]

Optional and device specific parameter
---

 - gige/mtu_size: only available for GigE-Cameras.


# Usage

The Pylon Camera Node can be started over the launch file which includes a config file with desired Parameter as framerate or exposrue time

     roslaunch pylon_camera pylon_camera.launch

Images were only published if another node connects to the image topic. The published images can be seen using the image_view node from the image_pipeline stack:

     rosrun image_view image_view image:=/pylon_camera/image_raw