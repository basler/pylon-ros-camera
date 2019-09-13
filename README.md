# pylon-ROS-camera

The official pylon ROS driver for [Basler](http://www.baslerweb.com/) GigE Vision and USB3 Vision cameras

**Please Note:**
This project is offered with no technical support by Basler AG.
You are welcome to post any questions or issues on [GitHub](https://github.com/basler/pylon-ros-camera)

This driver was improved by [drag and bot GmbH](www.dragandbot.com) from the version originally released by [Magazino GmbH](https://github.com/magazino/pylon_camera).

**ROS packages included:**

- **pylon_camera**: the driver itself
- **camera_control_msgs**: message and service definitions for interacting with the camera driver

Please check the README file of each package for more details and help.

## For the Impatient
 * Clone this repository in your catkin workspace (e.g. catkin_ws): `cd ~/catkin_ws/src && git clone https://github.com/basler/pylon-ros-camera`
 * Clone drag&bot public commom messages: `git clone https://github.com/dragandbot/dragandbot_common.git`
 * Install ROS dependencies: `sudo sh -c 'echo "yaml https://raw.githubusercontent.com/basler/pylon-ros-camera/master/pylon_camera/rosdep/pylon_sdk.yaml" > /etc/ros/rosdep/sources.list.d/30-pylon_camera.list' && rosdep update && sudo rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y`
 * Compile the workspace using catkin build or catkin make: `cd ~/catkin_ws && catkin clean -y && catkin build && source ~/.bashrc` or `cd ~/catkin_ws && catkin_make clean && catkin_make && source ~/.bashrc`

Pylon SDK API is automatically installed through rosdep installation.

## Available functionalities:

This package offers many functions of the Basler [pylon Camera Software Suite](https://www.baslerweb.com/en/products/software/basler-pylon-camera-software-suite/) C++ API inside the ROS-Framework.

This is a list of the supported functionality accesible through ROS services, which may depend on the exact camera model you are using:

### Image Format Control
 * Offset X
 * Offset Y
 * Reverse X
 * Reverse Y
 * Pixel Format
 * Binning Control

### Analog Control
 * Black Level
 * Black Level Raw

### Image Quality Control
 * PGI Control
 * Demosaicing Mode
 * Noise Reduction
 * Sharpness Enhancement
 * Light Source Preset
 * Balance White Auto
 * Brightness Control
 * Gain Control
 * Gamma Control
 * ROI Control

### Acquisition Control
 * Sensor Readout Mode
 * Acquisition Burst Frame
 * Acquisition Frame Count
 * Trigger Selector
 * Trigger Mode
 * Generate Software Trigger
 * Trigger Source
 * Trigger Activation
 * Trigger Delay

### Digital I/O Control
 * Line Selector
 * Line Mode
 * Line Source
 * Line Inverter
 * Line Debouncer Time

### User Set Control
 * User Set Selector
 * User Set Load
 * User Set Save
 * User Set Default

### Device Control
 * Device Link Throughput Limit Mode
 * Device Link Throughput Limit
 * Device Reset

## ROS Service list
Service Name  | Notes
------------- | -------------
/pylon_camera_node/get_loggers  | -
/pylon_camera_node/set_binning  | -
/pylon_camera_node/set_brightness | -
/pylon_camera_node/set_camera_info | -
/pylon_camera_node/set_exposure | -
/pylon_camera_node/set_gain | -
/pylon_camera_node/set_gamma | -
/pylon_camera_node/set_logger_level | -
/pylon_camera_node/set_roi | -
/pylon_camera_node/set_sleeping | -
/pylon_camera_node/execute_software_trigger | -
/pylon_camera_node/load_user_set | -
/pylon_camera_node/reset_device | -
/pylon_camera_node/save_user_set | -
/pylon_camera_node/select_default_user_set | value : 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
/pylon_camera_node/select_user_set | value : 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
/pylon_camera_node/set_acquisition_frame_count | value = new targeted frame count
/pylon_camera_node/set_balance_white_auto | value : 0 = Off, 1 = Once, 2 = Continuous
/pylon_camera_node/set_black_level | value = new targeted black level
/pylon_camera_node/set_demosaicing_mode | value : 0 = Simple, 1 = Basler PGI
/pylon_camera_node/set_device_link_throughput_limit | value = new targeted throughput limit in Bytes/sec.
/pylon_camera_node/set_device_link_throughput_limit_mode | data : false = deactivate, true = activate
/pylon_camera_node/set_image_encoding | value = mono8, mono16, bgr8, rgb8, bayer_bggr8, bayer_gbrg8, bayer_rggb8, bayer_grbg8, bayer_rggb16, bayer_bggr16, bayer_gbrg16, bayer_grbg16
/pylon_camera_node/set_light_source_preset | value : 0 = Off, 1 = Daylight5000K, 2 = Daylight6500K, 3 = Tungsten2800K
/pylon_camera_node/set_line_debouncer_time | value = delay in micro sec.
/pylon_camera_node/set_line_inverter | data : false = deactivate, true = activate
/pylon_camera_node/set_line_mode | value : 0 = Input, 1 = Output
/pylon_camera_node/set_line_selector | value : 0 = Line1, 1 = Line2, 2 = Line3, 3 = Line4
/pylon_camera_node/set_line_source | value : 0 = Exposure Active, 1 = FrameTriggerWait, 2 = UserOutput1, 3 = Timer1Active, 4 = FlashWindow
/pylon_camera_node/set_noise_reduction | value = reduction value
/pylon_camera_node/set_offset_x | value = targeted offset in x-axis
/pylon_camera_node/set_offset_y | value = targeted offset in y-axis
/pylon_camera_node/set_pgi_mode | data : false = deactivate, true = activate
/pylon_camera_node/set_reverse_x | data : false = deactivate, true = activate
/pylon_camera_node/set_reverse_y | data : false = deactivate, true = activate
/pylon_camera_node/set_sensor_readout_mode | value : 0 = Normal, 1 = Fast
/pylon_camera_node/set_sharpness_enhancement | value = sharpness value
/pylon_camera_node/set_trigger_activation | value : 0 = RigingEdge, 1 = FallingEdge
/pylon_camera_node/set_trigger_delay | value = delay in micro sec.
/pylon_camera_node/set_trigger_mode | data : false = deactivate, true = activate
/pylon_camera_node/set_trigger_selector | value : 0 = Frame start, 1 = Frame burst start (ace USB cameras) / Acquisition Start (ace GigE cameras)
/pylon_camera_node/set_trigger_source | value : 0 = Software, 1 = Line1, 2 = Line3, 3 = Line4, 4 = Action1 (only selected GigE Camera)
/pylon_camera_node/start_grabbing | -
/pylon_camera_node/stop_grabbing  | -

## Image pixel encoding

This package currently support below ROS image pixel formats :

	* mono8	        (Basler Format : Mono8)
	* mono16	(Basler Format : Mono16, Mono12)       (Notes 1&2)
	* bgr8 		(Basler Format : BGR8)
	* rgb8 		(Basler Format : RGB8)
	* bayer_bggr8 	(Basler Format : BayerBG8)
	* bayer_gbrg8 	(Basler Format : BayerGB8)
	* bayer_rggb8 	(Basler Format : BayerRG8)
	* bayer_grbg8 	(Basler Format : BayerRG8)
	* bayer_rggb16	(Basler Format : BayerRG16, BayerRG12) (Notes 1&2)
	* bayer_bggr16 	(Basler Format : BayerBG16, BayerBG12) (Notes 1&2)
	* bayer_gbrg16 	(Basler Format : BayerGB16, BayerGB12) (Notes 1&2)
	* bayer_grbg16 	(Basler Format : BayerGR16, BayerGR12) (Notes 1&2)

<u>NOTES: </u>

1 : 12-bits image will be remapped to 16-bits using bits shifting to make it work with the ROS 16-bits sensor standard message.

2 : When the user call the /pylon_camera_node/set_image_encoding to use 16-bits encoding, the driver will check first for the availability of the requested 16-bits encoding to set it, when the requested 16-bits image encoding is not available, then the driver will check the availability of the equivalent 12-bits encoding to set it. When both 16-bits and 12-bits image encoding are not available then an error message will be returned.
