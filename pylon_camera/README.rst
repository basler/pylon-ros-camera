====
**ROS-Driver for Basler Cameras**
====

This package offers many functions of the Basler [pylon Camera Software Suite](https://www.baslerweb.com/en/products/software/basler-pylon-camera-software-suite/) C++ API inside the ROS-Framework.

Images can continuously be published over *\/image\_raw* or the *\/image\_rect* topic.
The latter just in case the intrinsic calibration matrices are provided through the **camera_info_url** parameter.

The camera-characteristic parameter such as hight, width, projection matrices and camera_frame were published over the *\/camera\_info* topic.
Furthermore an action-based image grabbing with desired exposure, gain, gamma and / or brightness is provided.
Hence one can grab a sequence of images with above target settings as well as a single image.

Adapting camera's settings regarding binning (in x and y direction), exposure, gain, gamma and brightness can be done using provided 'set_*' services.
These changes effect the continuous image acquisition and hence the images provided through the image topics.

The default node operates in Software-Trigger Mode.
This means that the image acquisition is triggered with a certain rate and the camera is not running in the continuous mode.

The package opens either a predefined camera (using a given 'device_user_id' parameter) or, if no camera id is predefined the first camera device it can find.

This driver was improved by [drag and bot GmbH](www.dragandbot.com) from the version originally released by [Magazino GmbH](https://github.com/magazino/pylon_camera).

|

******
**Installation**
******

The pylon_camera-pkg requires the pylon Camera Software Suite for Linux to be installed on your system. This package will be automatically installed through rosdep script for x86_64 architecture from [drag&bot fileserver](https://dnb-public-downloads-misc.s3.eu-central-1.amazonaws.com/pylon/pylon_5.2.0.13457-deb0_amd64.deb). If this install process fails, it is still possible to install the API libraries manually. In that case, please download and install the latest pylon Camera Software Suite Linux Debian Installer Package for your architecture from:

``https://www.baslerweb.com/de/support/downloads/downloads-software/``

First, clone this repository in your catkin workspace (e.g. catkin_ws) and the drag&bot public common messages, which is required for publishing status information of the hardware component to the drag&bot component monitoring:

``cd ~/catkin_ws && git clone https://github.com/basler/pylon-ros-camera && git clone https://github.com/dragandbot/dragandbot_common.git``

In order to build the package, you need to configure rosdep (i.e. the ROS command-line tool for checking and installing system dependencies for ROS packages) such that
it knows how to resolve this dependency. This can be achieved by executing the following commands:

``sudo sh -c 'echo "yaml https://raw.githubusercontent.com/basler/pylon-ros-camera/master/pylon_camera/rosdep/pylon_sdk.yaml" > /etc/ros/rosdep/sources.list.d/30-pylon_camera.list' && rosdep update && sudo rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y``

Compile the workspace using catkin build or catkin make: 
``cd ~/catkin_ws && catkin clean -y && catkin build && source ~/.bashrc``
``cd ~/catkin_ws && catkin_make clean && catkin_make && source ~/.bashrc``

GigE Cameras IP Configuration can be done using the command: 
`roslaunch pylon_camera pylon_camera_ip_configuration.launch`
|

******
**Parameters**
******

All parameters are listed in the default config file:  ``config/default.yaml``

**Common parameters**

- **camera_frame**
  The tf frame under which the images were published

- **device_user_id**
  The DeviceUserID of the camera. If empty, the first camera found in the device list will be used

- **camera_info_url**
  The CameraInfo URL (Uniform Resource Locator) where the optional intrinsic camera calibration parameters are stored. This URL string will be parsed from the CameraInfoManager:
  http://docs.ros.org/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html#details

- **image_encoding**
  The encoding of the pixels -- channel meaning, ordering, size taken from the list of strings in include/sensor_msgs/image_encodings.h. The supported encodings are 'mono8', 'bgr8', 'rgb8', 'bayer_bggr8', 'bayer_gbrg8' and 'bayer_rggb8'.
  Default values are 'mono8' and 'rgb8'

- **binning_x & binning_y**
  Binning factor to get downsampled images. It refers here to any camera setting which combines rectangular neighborhoods of pixels into larger "super-pixels." It reduces the resolution of the output image to (width / binning_x) x (height / binning_y). The default values binning_x = binning_y = 0 are considered the same as binning_x = binning_y = 1 (no subsampling).

- **downsampling_factor_exposure_search**
  To speed up the exposure search, the mean brightness is not calculated on the entire image, but on a subset instead. The image is downsampled until a desired window hight is reached. The window hight is calculated out of the image height divided by the downsampling_factor_exposure search

- **frame_rate**
  The desired publisher frame rate if listening to the topics. This parameter can only be set once at start-up. Calling the GrabImages-Action can result in a higher frame rate.

- **shutter_mode**
  Set mode of camera's shutter if the value is not empty. The supported modes are 'rolling', 'global' and 'global_reset'.
  Default value is '' (empty)

**Image Intensity Settings**

The following settings do **NOT** have to be set. Each camera has default values which provide an automatic image adjustment resulting in valid images

- **exposure**
  The exposure time in microseconds to be set after opening the camera.

- **gain**
  The target gain in percent of the maximal value the camera supports. For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so called 'device specific units'.

- **gamma**
  Gamma correction of pixel intensity. Adjusts the brightness of the pixel values output by the camera's sensor to account for a non-linearity in the human perception of brightness or of the display system (such as CRT).

- **brightness**
  The average intensity value of the images. It depends the exposure time as well as the gain setting. If '**exposure**' is provided, the interface will try to reach the desired brightness by only varying the gain. (What may often fail, because the range of possible exposure values is many times higher than the gain range). If '**gain**' is provided, the interface will try to reach the desired brightness by only varying the exposure time. If '**gain**' AND '**exposure**' are given, it is not possible to reach the brightness, because both are assumed to be fix.

- **brightness_continuous**
  Only relevant, if '**brightness**' is set: The brightness_continuous flag controls the auto brightness function. If it is set to false, the brightness will only be reached once. Hence changing light conditions lead to changing brightness values. If it is set to true, the given brightness will be reached continuously, trying to adapt to changing light conditions. This is only possible for values in the possible auto range of the pylon API which is e.g. [50 - 205] for acA2500-14um and acA1920-40gm

- **exposure_auto & gain_auto**
  Only relevant, if '**brightness**' is set: If the camera should try to reach and / or keep the brightness, hence adapting to changing light conditions, at least one of the following flags must be set. If both are set, the interface will use the profile that tries to keep the gain at minimum to reduce white noise. The exposure_auto flag indicates, that the desired brightness will be reached by adapting the exposure time. The gain_auto flag indicates, that the desired brightness will be reached by adapting the gain.

**Optional and device specific parameter**

- **gige/mtu_size**
  The MTU size. Only used for GigE cameras. To prevent lost frames configure the camera has to be configured with the MTU size the network card supports. A value greater 3000 should be good (1500 for RaspberryPI)

- **gige/inter_pkg_delay**
  The inter-package delay in ticks. Only used for GigE cameras. To prevent lost frames it should be greater 0. For most of GigE-Cameras, a value of 1000 is reasonable. For GigE-Cameras used on a RaspberryPI this value should be set to 11772.

- **trigger_timeout**
  The camera trigger timeout in ms

- **grab_timeout**
  The camera grab timeout in ms

- **white_balance_auto**
  Automatically corrects color shifts in images acquired. (0 = Off, 1 = Once, 2 = Continuous).

- **white_balance_ratio_red**
  The Balance White camera feature allows you to manually correct color shifts so that white objects appear white in images acquired.
  For this purpose, a digital gain correction can be applied per color channel (red, green, blue).
  The increase or decrease in intensity is proportional. For example, if the balance ratio for a color is set to 1.2, the intensity of that color is increased by 20 %.

  NOTE: This parameter will have an effect only in case that white_balance_auto set to off.

- **white_balance_ratio_green**
  Refer to description of white_balance_ratio_red.

- **white_balance_ratio_blue**
  Refer to description of white_balance_ratio_red.

- **grab_strategy**
  The camera grabbing strategy.


******
**Usage**
******

The pylon_camera_node can be started over the launch file which includes a config file with desired parameters as frame rate or exposure time

``roslaunch pylon_camera pylon_camera_node.launch``     or     ``rosrun pylon_camera pylon_camera_node``

Anyhow running above launch commands will run the camera with the current set parameters on the camera. To launch the pylon node use specific user set you should run one of the below commands:

``roslaunch pylon_camera pylon_camera_node.launch startup_user_set:=Default``  or ``roslaunch pylon_camera pylon_camera_node.launch startup_user_set:=UserSet1`` or ``roslaunch pylon_camera pylon_camera_node.launch startup_user_set:=UserSet2`` or ``roslaunch pylon_camera pylon_camera_node.launch startup_user_set:=UserSet3``

Images were only published if another node connects to the image topic. The published images can be seen using the image_view node from the image_pipeline stack:

``rosrun image_view image_view image:=/pylon_camera_node/image_raw``


******
**ROS Service Commands**
******

Some of the ROS service use integer values for as a commands, below are a list of these services and their commands:

- **set_demosaicing_mode** ROS Service: 0 = Simple, 1 = Basler PGI

- **set_light_source_preset** ROS Service: 0 = Off, 1 = Daylight5000K, 2 = Daylight6500K, 3 = Tungsten2800K

- **set_balance_white_auto** ROS Service: 0 = Off, 1 = Once, 2 = Continuous

- **set_sensor_readout_mode** ROS Service: 0 = Normal, 1 = Fast

- **set_trigger_selector** ROS Service: 0 = Frame start, 1 = Frame burst start (ace USB cameras) / Acquisition Start (ace GigE cameras)

- **set_trigger_source** ROS Service: 0 = Software, 1 = Line1, 2 = Line3, 3 = Line4, 4 = Action1 (only selected GigE Camera)

- **set_trigger_activation** ROS Service: 0 = RisingEdge, 1 = FallingEdge

- **set_line_selector** ROS Service: 0 = Line1, 1 = Line2, 2 = Line3, 3 = Line4, 

- **set_line_mode** ROS Service: 0 = Input, 1 = Output

- **set_line_source** ROS Service: 0 = Exposure Active, 1 = FrameTriggerWait , 2 = UserOutput1, 3 = Timer1Active, 4 = FlashWindow(only with rolling shutter)

- **select_user_set** ROS Service: 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw, 

- **select_default_user_set** ROS Service: 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw

- **set_gamma_selector**  ROS Service: 0 = User, 1 = sRGB

- **set_grabbing_strategy**  ROS Service: 0 = GrabStrategy_OneByOne, 1 = GrabStrategy_LatestImageOnly, 2 = GrabStrategy_LatestImages

******
**Questions**
******

Please provide your questions via http://answers.ros.org/questions/ and tag them with **pylon_camera**