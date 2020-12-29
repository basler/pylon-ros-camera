^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_control_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.5 (2020-12-08)
------------------
* Add SetWhiteBalance.srv for setting the white balance of the image red/green/blue channels.

0.2.4 (2019-09-02)
------------------
* Add SetStringValue.srv for the below new ROS services :
	- set_Image_Encoding
* Add camera available image encoding and and current image encoding to the currentParams message type

0.2.3 (2019-08-13)
------------------
* Add currentParams msg type for current camera parameters publisher.
* camera_control_msgs/SetBool.h deleted, instead standard ROS std_srvs/SetBool is used

0.2.2 (2019-08-12)
------------------
* Add SetIntegerValue.srv for the below new ROS services :
	- set_black_level ROS service
	- set_acquisition_frame_count
	- set_offset_x
	- set_offset_y
	- set_demosaicing_mode
	- set_light_source_preset
	- set_balance_white_auto
	- set_sensor_readout_mode
	- set_trigger_selector
	- set_trigger_source
	- set_trigger_activation
	- set_line_selector
	- set_line_mode
	- set_line_source
	- set_device_link_throughput_limit
	- select_user_set
	- select_default_user_set
	
* Add SetFloatValue.srv for the below new ROS services :
	- set_trigger_delay
	- set_line_debouncer_time
	- set_noise_reduction
	- set_sharpness_enhancement

0.2.1 (2019-03-13)
------------------
* Cleanup CMakelists.txt
* Add setROI.srv for cropped image acquisition
* Removed maintainer

0.2.0 (2018-05-16)
------------------
* Added camera-info-object the the GrabResults
  This will allow variable binning settings.
* Updated license year

0.1.0 (2018-03-06)
------------------
* Removed deprectated msg-flags
  There were deprecated flags in camera_control_msgs/GrabImagesAction,
  that have been removed, namely
  uint8 BRIGHTNESS = 1
  uint8 EXPOSURE = 2
  uint8 target_type
  float32[] target_values

0.0.15 (2018-01-17)
-------------------
* Fixed logo-path in README

0.0.10 (2016-04-12)
-------------------
* removed outdated 'SetBrightnessSrv.srv', 'SetExposureSrv.srv', and 'SetSleepingSrv.srv'
* 0.0.8
* added LICENSE.rst file
* Contributors: Magazino Version Daemon, Marcel Debout

0.0.7 (2016-03-29)
------------------
* BinningSrv is now implemented
* Contributors: Marcel Debout

0.0.6 (2016-03-17 12:51)
------------------------
* std_srvs not yet used
* Contributors: Marcel Debout

0.0.5 (2016-03-17 12:31)
------------------------
* removed unused dependencies and added missing one
* Contributors: Marcel Debout

0.0.4 (2016-03-15)
------------------
* Merge branch 'brightness_bug_fixing' of bitbucket.org:Magazino/camera_control_msgs
* removed suffix 'srv', old services marked as deprecated
* 0.0.3
* setBinning.srv not yet implemented, because it needs to close and reopen the camera
* added SetBinning.srv
* removed deprecated SequenceExposureTimes msgs
* added feedback to to SetBrightnessSrv, GrabImagesInterface should not provide the continuous mode, so removed it
* added comments, added brightness continuous functionallity to the set BrightnessSrv
* extended interface with gain, gamma and correct brightness handling
* Contributors: Magazino Version Daemon, Marcel Debout

0.0.3 (2016-02-08)
------------------
* Added Service to set the desired gain
* Contributors: Marcel Debout

0.0.2 (2016-01-21)
------------------
* Added SetBool service.
  I'd like to remove this srv file soon once a new version of the std_srvs
  package is released.
* Contributors: Markus Grimm

0.0.1 (2016-01-11)
------------------
* reset version information
* nice package.xml, making catkin lint happy
* removed GrabSequence.action, is now in GrabImages.action
* fixed merge conf
* new actions for grabbing trigger
* fied package name
* fixed package name in package.xml
* initial commit -> renaming from pylon_camera_msgs
* Contributors: Marcel Debout, Nikolas Engelhard, Ulrich Klank
