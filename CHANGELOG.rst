^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pylon_ros2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.3 (2025-04-23)
-------------------
* Use generic "GenICam" alias globally

3.0.2 (2024-12-12)
-------------------
* Fixing bug related to ExposureTime/ExposureTimeAbs variable ID problem
* Fixing compatibility problem with camera temperature value

3.0.1 (2024-11-27)
-------------------
* Removing image_common as a submodule.
* Fixing bug related to ExposureTime variable name for USB/Dart camera

2.2.0 (2024-10-25)
-------------------
* Taking into account auto exposure upper limit when user set is set to current setting.
* Adding service to get ptp status.
* Changing cmake minimum version.

2.1.0 (2024-09-19)
-------------------
* Easier driver installation procedure with image_common as submodule. Documented in README file.

2.0.11 (2024-08-23)
-------------------
* Fixing issue related to the use of WaitForFrameTriggerReady() in the base. This function is not supported by all camera devices. The function CanWaitForFrameTriggerReady() is now used beforehand.

2.0.10 (2024-07-22)
-------------------
* Many minor changes, bug fixes, and code update with respect to newest c++ standards
* The ROS2 driver / Humble branch has been successfully tested with pylon 7.5.0 and the supplementary package for blaze 1.6.0. If with pylon 7.5.0, the pylon viewer does not start, refer to the Troubleshooting section of the RO2 driver documentation.

2.0.9 (2024-06-18)
-------------------
* Fix memory leak with blaze images.
* Changing blaze depth map encoding and not using radial distance anymore when computing it.

2.0.8 (2024-05-11)
-------------------
* Fix issues and adjustment related to line selector, mode, and source setting. Beware that the line selector and source setting services' inputs have slightly changed. Refer to the documentation for more information.

2.0.7 (2024-05-27)
-------------------
* Fix issue related to demosaicing and rectification of images. Demosaic images before rectify them.
* Fix ros parameter initializing by declaring them if there're not set beforehand.

2.0.6 (2024-05-06)
-------------------
* Improving the setLineMode function to handle easily other modes

2.0.5 (2024-05-02)
-------------------
* Use acquisition timestamp if corresponding chunk is enabled.
* Minor fix: warn on unsupported startup profile.

2.0.4 (2024-02-12)
-------------------
* Fix access to chunk data: line status all, counter value, frame counter, exposure, timestamp. Adding dedicated test script.

2.0.3 (2023-12-01)
-------------------
* Fix PylonROS2CameraNode plugin-component mismatch 

2.0.2 (2023-11-01)
-------------------
* All streamable parameters are set, regardless of the loaded user set
* New streamable parameter set: frame transmission delay

2.0.1 (2023-10-26)
-------------------
* Blaze functionalities are now included in the official galactic and humble branches. Beware that it is now requiring the installation of the pylon Supplementary Package for blaze to be able to compile the driver.
* Readme file updated with new Basler website links.

1.3.0 (2023-02-07)
-------------------
* Official humble version integrating the blaze.

1.2.1 (2023-01-30)
-------------------
* Using tf2 for better point cloud visualization.
* Minor changes in documentation, especially about the installation procedure.

1.2.0 (2023-01-20)
-------------------
* Official galactic version integrating the blaze.

1.2.0.beta (2022-12-20)
-------------------
* Beta version of the galactic version integrating the blaze.

1.1.2 (2022-11-15)
-------------------
* Adding test program implementing an action client sending the goal to trigger the image grabbing through the action `/my_camera/pylon_ros2_camera_node/grab_images_raw`.
* Adding in documentation a small chapter about this test program.

1.1.1 (2022-11-11)
-------------------
* Adding documentation and info displays when starting the driver about the startup user set parameter file and the upper exposure time limit.
* Adding in documentation a small chapter about the priority of the launch file regarding the parameters.

1.1.0 (2022-07-12)
-------------------
* All the PTP-related parameters and commands are now implemented withiin the driver.
* Documentation is updated as well with a dedicated chapter on PTP parameters and commands.

1.0.2 (2022-04-21)
-------------------
* New chapters in the documentation about the device user id, the manual IP configuration of the camera and the packet size parameter modification
* Typo fix in the pylon node base implementation

1.0.1 (2022-03-18)
-------------------
* Making sure that the ROS2 parameters are declared and are existing whether the node is started through or not through the launch files.
* Minor bug fixes

1.0.0 (2022-03-18)
-------------------
* Porting of the existing ROS 1 pylon driver under ROS2. Functionalities are the same as the ROS 1 pylon driver.
