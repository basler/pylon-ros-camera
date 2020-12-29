# pylon-ROS-camera

The official pylon ROS driver for [Basler](http://www.baslerweb.com/) GigE Vision and USB3 Vision cameras

**Please Note:**
This project is offered with no technical support by Basler AG.
You are welcome to post any questions or issues on [GitHub](https://github.com/basler/pylon-ros-camera)

This driver was improved by [drag and bot GmbH](https://www.dragandbot.com) from the version originally released by [Magazino GmbH](https://github.com/magazino/pylon_camera).

**ROS packages included:**

- **pylon_camera**: the driver itself
- **camera_control_msgs**: message and service definitions for interacting with the camera driver

Please check the README file of each package for more details and help.

## For the Impatient
 * Clone this repository in your catkin workspace (e.g. catkin_ws): `cd ~/catkin_ws/src && git clone https://github.com/basler/pylon-ros-camera`
 * Clone drag&bot public common messages: `git clone https://github.com/dragandbot/dragandbot_common.git`
 * Install ROS dependencies: `sudo sh -c 'echo "yaml https://raw.githubusercontent.com/basler/pylon-ros-camera/master/pylon_camera/rosdep/pylon_sdk.yaml" > /etc/ros/rosdep/sources.list.d/30-pylon_camera.list' && rosdep update && sudo rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y`
 * Compile the workspace using catkin build or catkin make: `cd ~/catkin_ws && catkin clean -y && catkin build && source ~/.bashrc` or `cd ~/catkin_ws && catkin_make clean && catkin_make && source ~/.bashrc`
 * Start the driver: `roslaunch pylon_camera pylon_camera_node.launch`
 * GigE Cameras IP Configuration can be done using the command: `roslaunch pylon_camera pylon_camera_ip_configuration.launch`

The pylon Camera Software Suite is automatically installed through rosdep installation.

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
 * ROI Control

### Analog Control
 * Black Level
 * Black Level Raw
 * Gain Control
 * Gamma Control
 * Gain Auto

### Image Quality Control
 * PGI Control
 * Demosaicing Mode
 * Noise Reduction
 * Sharpness Enhancement
 * Light Source Preset
 * Balance White Auto
 * Brightness Control
 * Balance White

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
 * Exposure Time
 * Exposure Auto
 * Auto Exposure Time Upper Limit
 * Acquisition Frame Rate
 * Resulting Frame Rate
 * Trigger Timeout
 * Grabbing Timeout
 * Grabbing Strategy
 * Output Queue Size

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
 * Device User ID
 
### Transport Layer
 * (GigE only) GevSCPSPacketSize (Packet Size)
 * (GigE only) GevSCPD (Inter-Packet Delay)
 * (USB only) MaxTransferSize 

## ROS Service list

The ROS interface with the camera was extended with new functionality. Here is presented a list of current available services.

Service Name  | Notes
------------- | -------------
/pylon_camera_node/get_loggers  | -
/pylon_camera_node/gamma_enable | (For GigE Cameras)
/pylon_camera_node/set_binning  | -
/pylon_camera_node/set_brightness | -
/pylon_camera_node/set_camera_info | -
/pylon_camera_node/set_exposure | -
/pylon_camera_node/set_gain | -
/pylon_camera_node/set_gamma | -
/pylon_camera_node/set_gamma_selector | value : 0 = User, 1 = sRGB (For GigE Cameras)
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
/pylon_camera_node/set_max_transfer_size | Maximum USB data transfer size in bytes
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
/pylon_camera_node/set_grab_timeout  | -
/pylon_camera_node/set_trigger_timeout  | -
/pylon_camera_node/set_white_balance  | Triggering this service will turn off the white balance auto 
/pylon_camera_node/set_grabbing_strategy | value : 0 = GrabStrategy_OneByOne, 1 = GrabStrategy_LatestImageOnly, 2 = GrabStrategy_LatestImages
/pylon_camera_node/set_output_queue_size | -

## Image pixel encoding

This package currently support the following ROS image pixel formats :

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

1 : 12-bits image will be remapped to 16-bits using bit shifting to make it work with the ROS 16-bits sensor standard message.

2 : When the user call the /pylon_camera_node/set_image_encoding to use 16-bits encoding, the driver will check first for the availability of the requested 16-bits encoding to set it, when the requested 16-bits image encoding is not available, then the driver will check the availability of the equivalent 12-bits encoding to set it. When both 16-bits and 12-bits image encoding are not available then an error message will be returned.

## Usage

Start the driver with command: `roslaunch pylon_camera pylon_camera_node.launch`. Then the driver will try to connect to the available cameras automatically.

To test if the driver is correctly working we recommend to use the rqt ROS tool (http://wiki.ros.org/rqt). You will need to add an image viewer through the the contextual menu RQT Plugin --> Visualization --> Image View. Then please select the `pylon_camera_node/image_raw` to display the current camera picture. If the intrinsic calibration file was configured, `pylon_camera_node/image_rect` will also appear. Please check Intrinsic calibration section for further information.

This drivers offers different ROS services to change the camera parameters. To see the list of available services please use `rosservice list` command. Once you have located the desired service you can call it by using the `rosservice call /service_name {...parameters...}` (with the corresponding service and parameters). E.g.:

```
~/workspace/dnb_docs$ rosservice call /pylon_camera_node/set_reverse_x "data: true" 
success: True
message: "done"
```
To auto-fill the parameters you can use Tab after writing the service name. Please refer to http://wiki.ros.org/rosservice for ros service usage.

### Intrinsic calibration

ROS includes a standardised camera intrinsic calibration process through **camera_calibration** package (http://wiki.ros.org/camera_calibration). This calibration process generates a file which can be read by the **pylon-ros-camera** driver by setting the **camera_info_url** parameter of the **config/default.yaml** file to the correct URI (e.g. file:///home/user/data/calibrations/my_calibration.yaml)

## Troubleshooting

To increase performance and to minimize CPU usage when grabbing images, the following settings should be considered:

### Camera hot-swapping

If you hot-swap the camera with a different camera with a non-compatible pixel encoding format (e.g. mono and color cameras), you need to restart the ROS system to replace the encoding value or replace the rosparam directly by setting the image_encoding parameter. E.g.:
`rosparam set /pylon_camera_node/image_encoding "mono8"`

### GigE Devices

#### Maximum UDP Socket Buffer Size

The system's maximum UDP receive buffer size should be increased to ensure a stable image acquisition. A maximum size of 2 MB is recommended. This can be achieved by issuing the sudo sysctl net.core.rmem_max=2097152 command. To make this setting persistent, you can add the net.core.rmem_max setting to the /etc/sysctl.conf file.

#### Enable Jumbo Frames.

Many GigE network adapters support so-called jumbo frames, i.e., network packets larger than the usual 1500 bytes. To enable jumbo frames, the maximum transfer unit (MTU) size of the PC's network adapter must be set to a high value. We recommend using a value of 8192.

#### Increase the packet size.

If your network adapter supports jumbo frames, you set the adapter's MTU to 8192 as described above. In order to take advantage of the adapter's jumbo frame capability, you must also set the packet size used by the camera to 8192.

If you are working with the pylon Viewer application, you can set the packet size by first selecting a camera from the tree in the "Device" pane. In the "Features" pane, expand the features group that shows the camera's name, expand the "Transport Layer" parameters group, and set the "Packet Size" parameter to 8192. If you write your own application, use the camera API to set the PacketSize parameter to 8192.

#### Real-time Priority

The GigE Vision implementation of Basler pylon software uses a thread for receiving image data. Basler pylon tries to set the thread priority for the receive thread to real-time thread priority. This requires certain permissions. The 'Permissions for Real-time Thread Priorities' section of the pylon INSTALL document describes how to grant the required permissions.

### U3V Devices

#### Increasing Packet Size

For faster USB transfers you should increase the packet size. You can do this by changing the "Stream Parameters" -> "Maximum Transfer Size" value from inside the pylon Viewer or by setting the corresponding value via the API. After increasing the package size you will likely run out of kernel space and see corresponding error messages on the console. The default value set by the kernel is 16 MB. To set the value (in this example to 1000 MB) you can execute as root:
`echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb`
This would assign a maximum of 1000 MB to the USB stack.
