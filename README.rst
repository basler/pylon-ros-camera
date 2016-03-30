ROS-driver for Basler Cameras
====
developed by Magazino GmbH, using the pylon Software Camera Suite by Basler AG
---
.. image:: https://bytebucket.org/Magazino/pylon_camera/raw/b195c6579b446757fffa674954f357d9fc95686e/wiki_imgs/basler.png?token=715219a25dc1261c139eef9ec36c42469cbce048
   :scale: 10 %
   :align: left

.. image:: https://bytebucket.org/Magazino/pylon_camera/raw/b195c6579b446757fffa674954f357d9fc95686e/wiki_imgs/basler.png?token=715219a25dc1261c139eef9ec36c42469cbce048
   :scale: 10 %
   :align: left

.. image:: ../wiki_imgs/basler.png
   :scale: 50 %
   :align: center

.. image:: ../wiki_imgs/pylon.png
   :scale: 50 %
   :align: right
   
   

This package offers many functions of the Basler-PylonAPI inside the ROS-Framwork.

The package supports Baslers USB 3.0, GigE as well as the DART cameras.
Images can continuously be published over _/image\_raw_ topic. The camera-characteristic parameter such as hight, width and camera_frame were published over the _/camera\_info topic.
Furthermore an action-based image grabbing with desired exposure, gain, gamma and / or brightness is provided.
Hence one can grab a sequence of images with above target settings as well as a single image.
Adapting camera's settings regarding binning (in x and y direction), exposure, gain, gamma and brightness can be done using provided 'set_*' services.
These changements effect the continuous image acqusistion and hence the images provided through the image topics.
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

 - **camera_frame**
 
 The tf frame under which the images were published
 
 - **device_user_id**
 
 The DeviceUserID of the camera. If empty, the first camera found in the device list will be used
 
 - **binning_x & binning_y**
 
 Binning factor to get downsampled images. It refers here to any camera setting which combines rectangular neighborhoods of pixels into larger "super-pixels." It reduces the resolution of the output image to (width / binning_x) x (height / binning_y). The default values binning_x = binning_y = 0 are considered the same as binning_x = binning_y = 1 (no subsampling).
 
Image Intensity Settings
----
The following settings do **NOT** have to be set. Each camera has default values which provide an automatic image adjustment resulting in valid images

 - **exposure**: The exposure time in microseconds to be set after opening the camera.
 - **gain**: The target gain in percent of the maximal value the camera supports. For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so called 'device specific units'.
 - **gamma**: Gamma correction of pixel intensity. Adjusts the brightness of the pixel values output by the camera's sensor to account for a non-linearity in the human perception of brightness or of the display system (such as CRT).
 - **brightness**: The average intensity value of the images. It depends the exposure time as well as the gain setting. If 'exposure' is provided, the interface will try to reach the desired brightness by only varying the gain. (What may often fail, because the range of possible exposure vaules is many times higher than the gain range). If 'gain' is provided, the interface will try to reach the desired brightness by only varying the exposure time. If gain AND exposure are given, it is not possible to reach the brightness, because both are assumed to be fix.
 - **brightness_continuous**: Only relevant, if 'brightness' is set: The brightness_continuous flag controls the auto brightness function. If it is set to false, the brightness will only be reached once. Hence changing light conditions lead to changing brightness values. If it is set to true, the given brightness will be reached continuously, trying to adapt to changing light conditions. This is only possible for values in the possible auto range of the pylon API which is e.g. [50 - 205] for acA2500-14um and acA1920-40gm
 - **exposure_auto & gain_auto**: Only relevant, if '**brightness**' is set: If the camera should try to reach and / or keep the brightness, hence adapting to changing light conditions, at least one of the following flags must be set. If both are set, the interface will use the profile that tries to keep the gain at minimum to reduce white noise. The exposure_auto flag indicates, that the desired brightness will be reached by adapting the exposure time. The gain_auto flag indicates, that the desired brightness will be reached by adapting the gain.
 - **frame_rate**: The desired publisher frame rate if listening to the topics. This paramter can only be set once at startup. Calling the GrabImages-Action can result in a higher framerate
 
Optional and device specific parameter
---

 - **gige/mtu_size**: The MTU size. Only used for GigE cameras. To prevent lost frames configure the camera has to be configured with the MTU size the network card supports. A value greater 3000 should be good (1500 for RaspberryPI)


# Usage

The Pylon Camera Node can be started over the launch file which includes a config file with desired Parameter as framerate or exposrue time

     roslaunch pylon_camera pylon_camera.launch

Images were only published if another node connects to the image topic. The published images can be seen using the image_view node from the image_pipeline stack:

     rosrun image_view image_view image:=/pylon_camera/image_raw
     
     
 Authors:
     Marcel Debout

:Version: 1.0 of 2001/08/08