# ROS2-Driver for Basler Cameras

The official pylon ROS2 driver for [Basler](http://www.baslerweb.com/) GigE Vision, Basler USB3 Vision and Basler blaze 3D cameras (branch: `humble_incl_blaze`)

This driver provides many functionalities available through the Basler [pylon Camera Software Suite](https://www.baslerweb.com/en/products/software/basler-pylon-camera-software-suite/) C++ API.

**Please Note:**
This project is offered with no technical support by Basler AG.
You are welcome to post any questions or issues on [GitHub](https://github.com/basler/pylon-ros-camera/issues)


## Installation

### Prerequisites

- From [Ubuntu 22.04 Jammy Jellyfish](https://releases.ubuntu.com/jammy/)
- From [Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Your ROS2 environment must be [configured](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html), your workspace [created](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html), and colcon, used to build the packages, [installed](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).
- [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html). rosdep must be installed as a debian package (`sudo apt update && sudo apt install python3-rosdep2 && sudo rosdep init && rosdep update`).
- From [pylon Camera Software Suite](https://www.baslerweb.com/de/support/downloads/downloads-software/) version 7.2 or newer. The latest APi libraries must be installed manually. Download and install the latest pylon Camera Software Suite Linux Debian Installer Package for your architecture. You may be experiencing some problems with the codemeter debian package installation. Just drop it for now and install only the pylon debian package in this case.
- [Git](https://git-scm.com/). Git must be installed as a debian package (`sudo apt update && sudo apt install git`).
- [xterm](https://invisible-island.net/xterm/). The xterm terminal emulator must be installed (refer to the *Know Issues* section below) as a debian package (`sudo apt update && sudo apt install xterm`).

### Install and build the packages

This repository including the pylon ROS2 packages must be cloned in your workspace (e.g., `dev_ws`):  
``cd ~/dev_ws/src && git clone -b humble https://github.com/basler/pylon-ros-camera pylon_ros2_camera``  
Due to a known issue with ROS2 (see the dedicated section below), the latest version of the `image_common` package must be installed from sources:  
``cd ~/dev_ws/src/pylon_ros2_camera && git clone https://github.com/ros-perception/image_common.git -b humble``  

Install the ROS2 dependencies required by the pylon ROS2 packages:  
``cd ~/dev_ws && rosdep install --from-paths src --ignore-src -r -y``  
You may experience some problems with the `diagnostic_updater` dependencies. In this case, install them by executing the following commands:  
``sudo apt install ros-humble-diagnostic-updater``  

Compile the workspace using `colcon`:  
``cd ~/dev_ws && colcon build``  

**Note**: The --symlink-install flag can be added to the `colcon build` command. This allows the installed files to be changed by changing the files in the source space (e.g., Python files or other not compiled resourced) for faster iteration (refer to [the ROS2 documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)).

**Note**: The packages are built in Release by default. The build type can be modfied by using the `--cmake-args` flag (for instance `colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Debug`).

Source the environment:  
``cd ~/dev_ws && . install/setup.bash``  

**Note**: This step can be skipped if the `setup.bash` file is sourced in your `.bashrc`.

Start the driver:  
``ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py``  


## Usage in a nutshell

Starting the *pylon_ros2_camera_node* starts the acquisition from a given Basler camera. The nodes allow as well to access many camera parameters and parameters related to the grabbing process itself.

The *pylon_ros2_camera_node* can be started thanks to a dedicated launch file thanks to the command:  
``ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py``  
Several parameters can be set through the launch file and the user parameter file loaded through it (the `pylon_ros2_camera_wrapper/config/default.yaml` user parameter file is loaded by default).

Acquisition from a specific camera is possible by setting the `device_user_id` parameter. If no specific camera is specified, the first available camera is connected automatically.  

The pylon node defines the different interface names according to the following convention:  
``[Camera name (= my_camera by default)]/[Node name (= pylon_ros2_camera_node)]/[Interface name]``  
The camera and the node names can be set thanks respectively to the `camera_name` and `node_name` parameters.  

Acquisition images are published through the `[Camera name]/[Node name]/[image_raw]` topic, only if a subscriber to this topic has been registered.  
To visualize the images, [rqt](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#install-rqt) can be used. Add an image viewer plugin through thanks to the contextual menu (Plugin -> Visualization -> Image View) and select the `[Camera name]/[Node name]/[image_raw]` topic to display the acquired and published images.  

Specific user set can be specified thanks to the `startup_user_set` parameter.  
``ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py --ros-args -p startup_user_set:=Default``  or ``ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py --ros-args -p startup_user_set:=UserSet1`` or ``ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py --ros-args -p startup_user_set:=UserSet2`` or ``ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py --ros-args -p startup_user_set:=UserSet3``  

The default trigger mode is set to software trigger. This means that the image acquisition is triggered with a certain frame rate, which may be lower than the maximum camera frame rate. The maximum camera frame rate can be reached when running a camera in a free-run or a hardware trigger mode.

Beware that some parameters implemented by the driver, like for instance the parameter `startup_user_set`, can be set through both the ROS2 parameter server and the driver launch file `pylon_ros2_camera.launch.py`, and that the latter has the priority over the ROS2 parameter server. For instance, if `startup_user_set` is set to `Default` in the `pylon_ros2_camera_wrapper/config/default.yaml` user parameter file and if it is set to `CurrentSetting` in the driver launch file (and if the driver is started thanks to it), then `startup_user_set` will be set to `CurrentSetting`.    

### Image pixel encoding

The pylon ROS2 driver support currently the following ROS2 image pixel formats :

	* mono8	        (Basler Format : Mono8)
	* mono16	(Basler Format : Mono16, Mono12)        (Notes 1&2)
	* bgr8 		(Basler Format : BGR8)
	* rgb8 		(Basler Format : RGB8)
	* bayer_bggr8 	(Basler Format : BayerBG8)
	* bayer_gbrg8 	(Basler Format : BayerGB8)
	* bayer_rggb8 	(Basler Format : BayerRG8)
	* bayer_grbg8 	(Basler Format : BayerRG8)
	* bayer_rggb16	(Basler Format : BayerRG16, BayerRG12)  (Notes 1&2)
	* bayer_bggr16 	(Basler Format : BayerBG16, BayerBG12)  (Notes 1&2)
	* bayer_gbrg16 	(Basler Format : BayerGB16, BayerGB12)  (Notes 1&2)
	* bayer_grbg16 	(Basler Format : BayerGR16, BayerGR12)  (Notes 1&2)

**NOTES:**

1 : 12-bits image will be remapped to 16-bits using bit shifting to make it work with the ROS2 16-bits sensor standard message.

2 : When the user calls the `set_image_encoding` service to use 16-bits encoding, the driver will check first for the availability of the requested 16-bits encoding to set it, when the requested 16-bits image encoding is not available, then the driver will check the availability of the equivalent 12-bits encoding to set it. When both 16-bits and 12-bits image encoding are not available then an error message will be returned.

### Intrinsic calibration and rectified images

ROS2 includes a standardised camera intrinsic calibration process through the *camera_calibration* package. This calibration process generates a file, which can be processed by the pylon ROS2 driver by setting the `camera_info_url` parameter in the `pylon_ros2_camera_wrapper/config/default.yaml` file (it is the user parameter file loaded by default through the driver main launch file) to the correct URI (e.g., file:///home/user/data/calibrations/my_calibration.yaml).

If the calibration is valid, the rectified images are published through the `[Camera name]/[Node name]/[image_rect]` topic, only if a subscriber to this topic has been registered.

### Setting device user id

It is easily possible to connect to a specific camera through its user id. This user id can be set through the parameter `device_user_id` listed in the .yaml user parameter file loaded at launch time (by default `pylon_ros2_camera_wrapper/config/default.yaml`). It is up to the user to create specific launch files, loading specific .yaml user parameter files, which would specify the user ids of the cameras that need to be connected. If no specific camera is specified, either because the `device_user_id` parameter is not set or no .yaml user parameter file is loaded, the first available camera is connected automatically.  

In addition to being able to do so through the pylon Viewer provided by Basler, it is possible to set the device user id with the command: `ros2 run pylon_ros2_camera_component set_device_user_id [-sn SERIAL_NB] your_device_user_id`. If no serial number is specified thanks to the option `-sn`, the specified device user id `your_device_user_id` will be assigned to the first available camera.
USB cameras must be disconnected and then reconnected after setting a new device user id. USB cameras keep their old user id otherwise.


## Packages

- **pylon_ros2_camera_component**: the driver itself. The package includes the main *pylon_ros2_camera_node* developed as a component.
- **pylon_ros2_camera_wrapper**: wrapper creating the main component `pylon_ros2_camera::PylonROS2CameraNode` implemented in the *pylon_ros2_camera_component* package. The wrapper starts the driver in a single process.
- **pylon_ros2_camera_interfaces**: package implementing *pylon_ros2_camera_node* interfaces (messages, services and actions).


## Parameters

**Common parameters**

- **camera_frame**  
  The tf2 frame under which the images were published.  
  ROS2 provides a library called [tf2](https://docs.ros.org/en/humble/Concepts/About-Tf2.html) (*TransForm* version 2) to manage the coordinate transformations between the different frames (coordinate systems) defined by the user and assigned to the components of a robotics system.

- **device_user_id**  
  The DeviceUserID of the camera. If empty, the first camera found in the device list will be used.

- **camera_info_url**  
  The CameraInfo URL (Uniform Resource Locator) where the optional intrinsic camera calibration parameters are stored. This URL string will be parsed from the CameraInfoManager.

- **image_encoding**  
  The encoding of the pixels -- channel meaning, ordering, size taken from the list of strings in include file *sensor_msgs/image_encodings.h*. The supported encodings are 'mono8', 'bgr8', 'rgb8', 'bayer_bggr8', 'bayer_gbrg8' and 'bayer_rggb8'. Default values are 'mono8' and 'rgb8'.

- **binning_x & binning_y**  
  Binning factor to get downsampled images. It refers here to any camera setting which combines rectangular neighborhoods of pixels into larger "super-pixels." It reduces the resolution of the output image to (width / binning_x) x (height / binning_y). The default values binning_x = binning_y = 0 are considered the same as binning_x = binning_y = 1 (no subsampling).

- **downsampling_factor_exposure_search**  
  To speed up the exposure search, the mean brightness is not calculated on the entire image, but on a subset instead. The image is downsampled until a desired window hight is reached. The window hight is calculated out of the image height divided by the downsampling_factor_exposure search.

- **frame_rate**  
  The desired publisher frame rate if listening to the topics. This parameter can only be set once at start-up. Calling the GrabImages-Action can result in a higher frame rate.

- **shutter_mode**  
  Set mode of camera's shutter if the value is not empty. The supported modes are 'rolling', 'global' and 'global_reset'. Default value is '' (empty)

- **white_balance_auto**  
  Camera white balance auto.

- **white_balance_ratio_red & white_balance_ratio_green & white_balance_ratio_blue**  
  Camera white balance ratio.

- **trigger_timeout**  
  Camera trigger timeout in ms.

- **grab_timeout**  
  Camera grab timeout in ms.

- **grab_strategy**  
  Camera grab strategy: 0 = GrabStrategy_OneByOne / 1 = GrabStrategy_LatestImageOnly / 2 = GrabStrategy_LatestImages

**Image Intensity Settings**

The following settings do **NOT** have to be set. Each camera has default values which provide an automatic image adjustment resulting in valid images.

- **exposure**  
  The exposure time in microseconds to be set after opening the camera.

- **gain**  
  The target gain in percent of the maximal value the camera supports. For USB cameras, the gain is in dB, for GigE cameras it is given in so called 'device specific units'.

- **gamma**  
  Gamma correction of pixel intensity. Adjusts the brightness of the pixel values output by the camera's sensor to account for a non-linearity in the human perception of brightness or of the display system (such as CRT).

- **brightness**  
  The average intensity value of the images. It depends the exposure time as well as the gain setting. If '**exposure**' is provided, the interface will try to reach the desired brightness by only varying the gain. (What may often fail, because the range of possible exposure values is many times higher than the gain range). If '**gain**' is provided, the interface will try to reach the desired brightness by only varying the exposure time. If '**gain**' AND '**exposure**' are given, it is not possible to reach the brightness, because both are assumed to be fix.

- **brightness_continuous**  
  Only relevant, if '**brightness**' is set: The brightness_continuous flag controls the auto brightness function. If it is set to false, the brightness will only be reached once. Hence changing light conditions lead to changing brightness values. If it is set to true, the given brightness will be reached continuously, trying to adapt to changing light conditions. This is only possible for values in the possible auto range of the pylon API which is e.g., [50 - 205] for acA2500-14um and acA1920-40gm.

- **exposure_auto & gain_auto**  
  Only relevant, if '**brightness**' is set: If the camera should try to reach and / or keep the brightness, hence adapting to changing light conditions, at least one of the following flags must be set. If both are set, the interface will use the profile that tries to keep the gain at minimum to reduce white noise. The exposure_auto flag indicates, that the desired brightness will be reached by adapting the exposure time. The gain_auto flag indicates, that the desired brightness will be reached by adapting the gain.

**Optional and device specific parameter**

- **exposure_search_timeout**  
  The timeout while searching the exposure which is connected to the desired brightness. For slow system this has to be increased.

- **auto_exposure_upper_limit**  
  The exposure search can be limited with an upper bound. This is to prevent very high exposure times and resulting timeouts. A typical value for this upper bound is ~2000000us. Beware that this upper limit is only set if `startup_user_set` is set to `Default`.  

- **gige/mtu_size**  
  The MTU size. Only used for GigE cameras. To prevent lost frames configure the camera has to be configured with the MTU size the network card supports. A value greater 3000 should be good (1500 for single-board computer)

- **gige/inter_pkg_delay**  
  The inter-packet delay in ticks. Only used for GigE cameras. To prevent lost frames it should be greater than 0. For most of GigE cameras, a value of 1000 is reasonable. For GigE cameras used on single-board computer, this value should be set to 11772.

- **auto_flash**  
  Flag that indicates if the camera has a flash connected, which should be on exposure. Only supported for GigE cameras. Default: false.

- **auto_flash_line_2**  
  Flag that indicates if the camera has a flash connected on line 2, which should be on exposure. Only supported for GigE cameras. Default: true.

- **auto_flash_line_3**  
  Flag that indicates if the camera has a flash connected on line 3, which should be on exposure. Only supported for GigE cameras. Default: true.

**ROS2 pylon node specific parameter**

- **startup_user_set**  
  Flag specifying if a given user set is used when starting the camera. Can be set to `Default`, `UserSet1`, `UserSet2`, `UserSet3`, and `CurrentSetting`.  

- **enable_status_publisher**  
  Flag used to enable/disable the node status publisher.

- **enable_current_params_publisher**  
  Flag used to enable/disable the current camera publisher.


## PTP synchronization

The Precision Time Protocol (PTP) camera feature allows you to synchronize multiple GigE cameras in the same network. It enables a camera to use the following features, if available:
- **Scheduled Action Commands** & **Action Commands**
- **Synchronous Free Run** (applies to ace 1 cameras)
- **Periodic Signal** (applies to ace 2 cameras)  

Refer to [the documentation](https://docs.baslerweb.com/precision-time-protocol) for more info about these features, with multiple code samples.  

The pylon driver gives accordingly access through ROS2 services to the following parameters and commands:

### ACE 1

**PTP configuration & activation**
- *GevIEEE1588*                    -> /my_camera/pylon_ros2_camera_node/enable_ptp [std_srvs/srv/SetBool]

**Scheduled Action Commands** & **Action Commands**
- *ActionDeviceKey*                -> /my_camera/pylon_ros2_camera_node/set_action_trigger_configuration [pylon_ros2_camera_interfaces/srv/SetActionTriggerConfiguration]
- *ActionGroupKey*                 -> /my_camera/pylon_ros2_camera_node/set_action_trigger_configuration [pylon_ros2_camera_interfaces/srv/SetActionTriggerConfiguration]
- *ActionGroupMask*                -> /my_camera/pylon_ros2_camera_node/set_action_trigger_configuration [pylon_ros2_camera_interfaces/srv/SetActionTriggerConfiguration]
- *IssueScheduledActionCommand*    -> /my_camera/pylon_ros2_camera_node/issue_scheduled_action_command [pylon_ros2_camera_interfaces/srv/IssueScheduledActionCommand]
- *IssueActionCommand*             -> /my_camera/pylon_ros2_camera_node/issue_action_command [pylon_ros2_camera_interfaces/srv/IssueActionCommand]

**Synchronous Free Run**
- *SyncFreeRunTimerStartTimeLow*   -> /my_camera/pylon_ros2_camera_node/set_sync_free_run_timer_start_time_low [pylon_ros2_camera_interfaces/srv/SetIntegerValue]
- *SyncFreeRunTimerStartTimeHigh*  -> /my_camera/pylon_ros2_camera_node/set_sync_free_run_timer_start_time_high [pylon_ros2_camera_interfaces/srv/SetIntegerValue]
- *SyncFreeRunTimerTriggerRateAbs* -> /my_camera/pylon_ros2_camera_node/set_sync_free_run_timer_trigger_rate_abs [pylon_ros2_camera_interfaces/srv/SetFloatValue]
- *SyncFreeRunTimerUpdate*         -> /my_camera/pylon_ros2_camera_node/update_sync_free_run_timer [std_srvs/srv/Trigger]
- *SyncFreeRunTimerEnable*         -> //my_camera/pylon_ros2_camera_node/enable_sync_free_run_timer [std_srvs/srv/SetBool]

### ACE 2

**PTP configuration & activation**
- *BslPtpPriority1*                -> /my_camera/pylon_ros2_camera_node/set_ptp_priority [pylon_ros2_camera_interfaces/srv/SetIntegerValue]
- *BslPtpProfile*                  -> /my_camera/pylon_ros2_camera_node/set_ptp_profile [pylon_ros2_camera_interfaces/srv/SetIntegerValue]
- *BslPtpNetworkMode*              -> /my_camera/pylon_ros2_camera_node/set_ptp_network_mode [pylon_ros2_camera_interfaces/srv/SetIntegerValue]
- *BslPtpUcPortAddrIndex*          -> /my_camera/pylon_ros2_camera_node/set_ptp_uc_port_address_index [pylon_ros2_camera_interfaces/srv/SetIntegerValue]
- *BslPtpUcPortAddr*               -> /my_camera/pylon_ros2_camera_node/set_ptp_uc_port_address [pylon_ros2_camera_interfaces/srv/SetIntegerValue]
- *BslPtpManagementEnable*         -> /my_camera/pylon_ros2_camera_node/enable_ptp_management_protocol [std_srvs/srv/SetBool]
- *BslTwoStep*                     -> /my_camera/pylon_ros2_camera_node/enable_two_step_operation [std_srvs/srv/SetBool]
- *PtpEnable*                      -> /my_camera/pylon_ros2_camera_node/enable_ptp [std_srvs/srv/SetBool]

**Scheduled Action Commands** & **Action Commands**
- *ActionDeviceKey*                -> /my_camera/pylon_ros2_camera_node/set_action_trigger_configuration [pylon_ros2_camera_interfaces/srv/SetActionTriggerConfiguration]
- *ActionGroupKey*                 -> /my_camera/pylon_ros2_camera_node/set_action_trigger_configuration [pylon_ros2_camera_interfaces/srv/SetActionTriggerConfiguration]
- *ActionGroupMask*                -> /my_camera/pylon_ros2_camera_node/set_action_trigger_configuration [pylon_ros2_camera_interfaces/srv/SetActionTriggerConfiguration]
- *IssueScheduledActionCommand*    -> /my_camera/pylon_ros2_camera_node/issue_scheduled_action_command [pylon_ros2_camera_interfaces/srv/IssueScheduledActionCommand]
- *IssueActionCommand*             -> /my_camera/pylon_ros2_camera_node/issue_action_command [pylon_ros2_camera_interfaces/srv/IssueActionCommand]

**Periodic Signal**
- *BslPeriodicSignalDelay*         -> /my_camera/pylon_ros2_camera_node/set_periodic_signal_delay [pylon_ros2_camera_interfaces/srv/SetFloatValue]
- *BslPeriodicSignalPeriod*        -> /my_camera/pylon_ros2_camera_node/set_periodic_signal_period [pylon_ros2_camera_interfaces/srv/SetFloatValue]


## Publishers

Name          | Notes
------------- | -------------
/my_camera/pylon_ros2_camera_node/camera_info  | sensor_msgs/msg/CameraInfo
/my_camera/pylon_ros2_camera_node/current_params  | current camera parameter
/my_camera/pylon_ros2_camera_node/image_raw  | acquired images
/my_camera/pylon_ros2_camera_node/image_rect  | rectified images if the camera is calibrated
/my_camera/pylon_ros2_camera_node/status  | camera status


## Service servers

Name          | Notes
------------- | -------------
/my_camera/pylon_ros2_camera_node/activate_autoflash_output_[index]  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/describe_parameters  | -
/my_camera/pylon_ros2_camera_node/enable_ptp  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/enable_ptp_management_protocol  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/enable_sync_free_run_timer  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/enable_two_step_operation  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/execute_software_trigger  | -
/my_camera/pylon_ros2_camera_node/get_chunk_counter_value  | -
/my_camera/pylon_ros2_camera_node/get_chunk_enable  | -
/my_camera/pylon_ros2_camera_node/get_chunk_exposure_time  | -
/my_camera/pylon_ros2_camera_node/get_chunk_frame_counter  | -
/my_camera/pylon_ros2_camera_node/get_chunk_line_status_all  | -
/my_camera/pylon_ros2_camera_node/get_chunk_mode_active  | -
/my_camera/pylon_ros2_camera_node/get_chunk_selector  | -
/my_camera/pylon_ros2_camera_node/get_chunk_timestamp  | -
/my_camera/pylon_ros2_camera_node/get_max_num_buffer  | -
/my_camera/pylon_ros2_camera_node/get_parameter_types  | -
/my_camera/pylon_ros2_camera_node/get_parameters  | -
/my_camera/pylon_ros2_camera_node/get_statistic_buffer_underrun_count  | -
/my_camera/pylon_ros2_camera_node/get_statistic_failed_buffer_count  | -
/my_camera/pylon_ros2_camera_node/get_statistic_failed_packet_count  | -
/my_camera/pylon_ros2_camera_node/get_statistic_missed_frame_count  | -
/my_camera/pylon_ros2_camera_node/get_statistic_resend_request_count  | -
/my_camera/pylon_ros2_camera_node/get_statistic_resynchronization_count  | -
/my_camera/pylon_ros2_camera_node/get_statistic_total_buffer_count  | -
/my_camera/pylon_ros2_camera_node/issue_action_command  | -
/my_camera/pylon_ros2_camera_node/issue_scheduled_action_command  | -
/my_camera/pylon_ros2_camera_node/list_parameters  | -
/my_camera/pylon_ros2_camera_node/load_user_set  | -
/my_camera/pylon_ros2_camera_node/reset_device  | -
/my_camera/pylon_ros2_camera_node/save_user_set  | -
/my_camera/pylon_ros2_camera_node/set_PGI_mode  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/set_acquisition_frame_count  | value = new targeted frame count
/my_camera/pylon_ros2_camera_node/set_action_trigger_configuration  | -
/my_camera/pylon_ros2_camera_node/set_binning  | -
/my_camera/pylon_ros2_camera_node/set_black_level  | value = new targeted black level
/my_camera/pylon_ros2_camera_node/set_brightness  | -
/my_camera/pylon_ros2_camera_node/set_chunk_enable  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/set_chunk_exposure_time  | -
/my_camera/pylon_ros2_camera_node/set_chunk_mode_active  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/set_chunk_selector  | -
/my_camera/pylon_ros2_camera_node/set_demosaicing_mode  | value : 0 = Simple, 1 = Basler PGI
/my_camera/pylon_ros2_camera_node/set_device_link_throughput_limit  | value = new targeted throughput limit in Bytes/sec.
/my_camera/pylon_ros2_camera_node/set_device_link_throughput_limit_mode  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/set_exposure  | -
/my_camera/pylon_ros2_camera_node/set_gain  | -
/my_camera/pylon_ros2_camera_node/set_gamma  | value: 0 = User, 1 = sRGB
/my_camera/pylon_ros2_camera_node/set_gamma_activation  | (For GigE Cameras)
/my_camera/pylon_ros2_camera_node/set_gamma_selector  | value : 0 = User, 1 = sRGB (For GigE Cameras)
/my_camera/pylon_ros2_camera_node/set_grab_timeout  | -
/my_camera/pylon_ros2_camera_node/set_grabbing_strategy  | -
/my_camera/pylon_ros2_camera_node/set_image_encoding  | value = mono8, mono16, bgr8, rgb8, bayer_bggr8, bayer_gbrg8, bayer_rggb8, bayer_grbg8, bayer_rggb16, bayer_bggr16, bayer_gbrg16, bayer_grbg16
/my_camera/pylon_ros2_camera_node/set_light_source_preset  | value : 0 = Off, 1 = Daylight5000K, 2 = Daylight6500K, 3 = Tungsten2800K
/my_camera/pylon_ros2_camera_node/set_line_debouncer_time  | value = delay in micro sec.
/my_camera/pylon_ros2_camera_node/set_line_inverter  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/set_line_mode  | value : 0 = Input, 1 = Output
/my_camera/pylon_ros2_camera_node/set_line_selector  | value : 0 = Line1, 1 = Line2, 2 = Line3, 3 = Line4
/my_camera/pylon_ros2_camera_node/set_line_source  | value : 0 = Exposure Active, 1 = FrameTriggerWait, 2 = UserOutput1, 3 = Timer1Active, 4 = FlashWindow
/my_camera/pylon_ros2_camera_node/set_max_num_buffer  | -
/my_camera/pylon_ros2_camera_node/set_max_transfer_size  | maximum USB data transfer size in bytes
/my_camera/pylon_ros2_camera_node/set_noise_reduction  | value = reduction value
/my_camera/pylon_ros2_camera_node/set_offset_x  | value = targeted offset in x-axis
/my_camera/pylon_ros2_camera_node/set_offset_y  | value = targeted offset in y-axis
/my_camera/pylon_ros2_camera_node/set_output_queue_size  | -
/my_camera/pylon_ros2_camera_node/set_parameters  | -
/my_camera/pylon_ros2_camera_node/set_parameters_atomically  | -
/my_camera/pylon_ros2_camera_node/set_periodic_signal_delay  | value : delay to be applied to the periodic signal in microseconds
/my_camera/pylon_ros2_camera_node/set_periodic_signal_period  | value : length of the periodic signal in microseconds
/my_camera/pylon_ros2_camera_node/set_ptp_network_mode  | value : 1 = Hybrid, 2 = Multicast, 3 = Unicast
/my_camera/pylon_ros2_camera_node/set_ptp_priority  | value = value indicating the priority of the device when determining the master clock
/my_camera/pylon_ros2_camera_node/set_ptp_profile  | value : 1 = Delay Request Response Default Profile, 2 = Peer to Peer Default Profile
/my_camera/pylon_ros2_camera_node/set_ptp_uc_port_address  | value = unicast port address
/my_camera/pylon_ros2_camera_node/set_ptp_uc_port_address_index  | value = unicast port address index
/my_camera/pylon_ros2_camera_node/set_reverse_x  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/set_reverse_y  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/set_roi  | -
/my_camera/pylon_ros2_camera_node/set_sensor_readout_mode  | value : 0 = Normal, 1 = Fast
/my_camera/pylon_ros2_camera_node/set_sharpness_enhancement  | value = sharpness value
/my_camera/pylon_ros2_camera_node/set_sleeping  | -
/my_camera/pylon_ros2_camera_node/set_sync_free_run_timer_start_time_high  | value = high 32 bits of the synchronous free run trigger start time
/my_camera/pylon_ros2_camera_node/set_sync_free_run_timer_start_time_low  | value = low 32 bits of the synchronous free run trigger start time
/my_camera/pylon_ros2_camera_node/set_sync_free_run_timer_trigger_rate_abs  | value = synchronous free run trigger rate
/my_camera/pylon_ros2_camera_node/set_timer_duration  | value = duration of the currently selected timer in microseconds
/my_camera/pylon_ros2_camera_node/set_timer_selector  | value : 1 = Timer 1, 2 = Timer 2, 3 = Timer 3, 4 = Timer 4
/my_camera/pylon_ros2_camera_node/set_timer_trigger_source  | value = see valid values of TimerTriggerSourceEnums in documentation
/my_camera/pylon_ros2_camera_node/set_trigger_activation  | value : 0 = RigingEdge, 1 = FallingEdge
/my_camera/pylon_ros2_camera_node/set_trigger_delay  | value = delay in micro sec.
/my_camera/pylon_ros2_camera_node/set_trigger_mode  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/set_trigger_selector  | value : 0 = Frame start, 1 = Frame burst start (ace USB cameras) / Acquisition Start (ace GigE cameras)
/my_camera/pylon_ros2_camera_node/set_trigger_source  | value : 0 = Software, 1 = Line 1, 2 = Line 3, 3 = Line 4, 4 = Action 1, 5 = Periodic Signal 1
/my_camera/pylon_ros2_camera_node/set_trigger_timeout  | -
/my_camera/pylon_ros2_camera_node/set_user_output_[index]  | data : false = deactivate, true = activate
/my_camera/pylon_ros2_camera_node/set_user_set_default_selector  | value : 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
/my_camera/pylon_ros2_camera_node/set_user_set_selector  | value : 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
/my_camera/pylon_ros2_camera_node/set_white_balance  | -
/my_camera/pylon_ros2_camera_node/set_white_balance_auto  | value : 0 = Off, 1 = Once, 2 = Continuous
/my_camera/pylon_ros2_camera_node/start_grabbing  | -
/my_camera/pylon_ros2_camera_node/stop_grabbing  | -
/my_camera/pylon_ros2_camera_node/update_sync_free_run_timer  | -
/my_camera/set_camera_info  | -


## Action servers

Name          | Notes
------------- | -------------
/my_camera/pylon_ros2_camera_node/grab_images_raw  | -

Through this action, it is possible to grab one image or a sequence of images with user-specified parameters (e.g., exposure time, brightness value, etc.). Refer to the action definition to get more information. 

The camera-characteristic parameter such as height, width, projection matrix (by ROS2 convention, this matrix specifies the intrinsic (camera) matrix of the processed (rectified) image - see the [CameraInfo message definition](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg) for detailed information) and camera_frame were published over the /camera_info topic. Furthermore, an action-based image grabbing with desired exposure time, gain, gamma and / or brightness is provided. Hence, one can grab a sequence of images with above target settings as well as a single image. Grabbing images through this action can result in a higher frame rate.

### Tests

The folder `pylon_ros2_camera_wrapper/test` includes a test program implementing an action client sending the goal to trigger the image grabbing through the action `/my_camera/pylon_ros2_camera_node/grab_images_raw`. Each grabbed image is displayed in a dedicated popup window. This program is for testing purposes and should be adapted according to one's needs.  


## Known issues

### Getting the number of subscribers from camera publisher
It is not possible to count correctly the number of subscribers to the `image_raw` and `image_rect` topics because of a known issue with the function `CameraPublisher::getNumSubscribers`. That is why [this image_common package](https://github.com/ros-perception/image_common/tree/humble), fixing this issue, needs to be cloned and compiled together with the `pylon_ros2_camera_node`. 

### User input in terminal when starting node through launch files
The ros2 launch mechanism doesn't allow to access stdin through a terminal (see [here](https://github.com/ros2/launch_ros/issues/165) and [here](https://answers.ros.org/question/343326/ros2-prefix-in-launch-file/)). This is solved in this implementation by installing and using `xterm` to emulate a terminal with possible user interaction.

### Service shutdown
In the ROS pylon implementation, the `activate_autoflash_output` and `set_user_output` service servers are shutdowned when the connection with a camera is lost. It is not possible for now to do so with ROS2 without shutting down the whole node (see [here](https://discourse.ros.org/t/how-to-shutdown-and-reinitialize-a-publisher-node-in-ros-2/4090)). There is no way to overcome this issue at the moment. 


## Troubleshooting

To increase performance and to minimize CPU usage when grabbing images, the following settings should be considered:

### Camera hot-swapping

If you hot-swap the camera with a different camera with a non-compatible pixel encoding format (e.g., mono and color cameras), you need to restart the ROS system to replace the encoding value or replace the rosparam directly by setting the image_encoding parameter. e.g.,:
`rosparam set /pylon_camera_node/image_encoding "mono8"`

### GigE Devices

#### No connection with connected camera

To be sure to be able to connect to a specific camera, its network configuration must be manually set through Basler's pylon IP configurator. To do so, click on the camera in the list of connected devices, select the `Static IP` option, set an `IP Address` within the same range as the one of your computer, and set the same `Subnet Mask` as the one from your computer.

#### Maximum UDP Socket Buffer Size

The system's maximum UDP receive buffer size should be increased to ensure a stable image acquisition. A maximum size of 2 MB is recommended. This can be achieved by issuing the sudo sysctl net.core.rmem_max=2097152 command. To make this setting persistent, you can add the net.core.rmem_max setting to the /etc/sysctl.conf file.

#### Enable Jumbo Frames.

Many GigE network adapters support so-called jumbo frames, i.e., network packets larger than the usual 1500 bytes. To enable jumbo frames, the maximum transfer unit (MTU) size of the PC's network adapter must be set to a high value. We recommend using a value of 8192.

#### Increase the packet size.

If your network adapter supports jumbo frames, you set the adapter's MTU to 8192 as described above. In order to take advantage of the adapter's jumbo frame capability, you must also set the packet size used by the camera to 8192.

If you are working with the pylon Viewer application, you can set the packet size by first selecting a camera from the tree in the "Device" pane. In the "Features" pane, expand the features group that shows the camera's name, expand the "Transport Layer" parameters group, and set the "Packet Size" parameter to 8192. If you write your own application, use the camera API to set the PacketSize parameter to 8192.

It is possible to change the packet size by changing the default value of the `mtu_size` parameter in the pylon ROS2 wrapper launch file. When the camera is grabbing, it is not possible to modify this parameter.

#### Real-time Priority

The GigE Vision implementation of Basler pylon software uses a thread for receiving image data. Basler pylon tries to set the thread priority for the receive thread to real-time thread priority. This requires certain permissions. The 'Permissions for Real-time Thread Priorities' section of the pylon INSTALL document describes how to grant the required permissions.

### U3V Devices

#### Increasing Packet Size

For faster USB transfers you should increase the packet size. You can do this by changing the "Stream Parameters" -> "Maximum Transfer Size" value from inside the pylon Viewer or by setting the corresponding value via the API. After increasing the package size you will likely run out of kernel space and see corresponding error messages on the console. The default value set by the kernel is 16 MB. To set the value (in this example to 1000 MB) you can execute as root:
`echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb`
This would assign a maximum of 1000 MB to the USB stack.
