^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pylon_ros2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
