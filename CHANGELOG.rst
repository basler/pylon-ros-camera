^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pylon_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* removed deprecated 'SetBrightnessSrv', 'SetExposureSrv' and 'SetSleepingSrv'. Please adapt to the new interface
* ROS_WARN instead of ROS_ERR if the desired brightness could not be reached
* 0.0.47
* Code review
* 0.0.46
* Changed dependencies for pylon to the new debian package
* 0.0.45
* fixed premature commit
  TORU-623
* Handle constructor failures differently
  TORU-623
* 0.0.44
* init size_t with 0 instead of -1
* 0.0.43
* readded HEader after rectification
* 0.0.42
* formatting & coding style
* 0.0.41
* added parameter for inter-pkg-delay for RaspberryPI usage
* 0.0.40
* linting
* Contributors: Magazino Version Daemon, Marcel Debout, Markus Grimm, Ulrich Klank

0.0.39 (2016-04-07)
-------------------
* removed dublicated dependency
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera into opencv_rect
* finally added rectification support using the image_geometry::pinhole_model and the CameraInfoManager
* pulled intrinsic calib-reading from opencv_class
* first implementation with the CameraInfoManager
* fixed strange overriding behaviour in case that one requests brightness with auto_exposure and auto_gain set to false
* 0.0.36
* fixed console output of the timeout duration in brightness search
* 0.0.35
* removed unused member, found shorter name for the grabbing action server
* 0.0.34
* finally added rectification support using the image_geometry::pinhole_model and the CameraInfoManager
* pulled intrinsic calib-reading from opencv_class
* first implementation with the CameraInfoManager
* started to integrate rectification
* Contributors: Magazino Version Daemon, Marcel Debout

0.0.38 (2016-04-04)
-------------------
* removed double output in case that the intensity settig fails
* Contributors: Marcel Debout

0.0.37 (2016-03-31 15:56)
-------------------------
* fixed strange overriding behaviour in case that one requests brightness with auto_exposure and auto_gain set to false
* Contributors: Marcel Debout

0.0.36 (2016-03-31 15:31)
-------------------------
* fixed console output of the timeout duration in brightness search
* Contributors: Marcel Debout

0.0.35 (2016-03-31 09:53)
-------------------------
* removed unused member, found shorter name for the grabbing action server
* Contributors: Marcel Debout

0.0.34 (2016-03-30 16:11)
-------------------------
* renamed ActionServer to GrabImagesAS
* Contributors: Marcel Debout

0.0.33 (2016-03-30 15:51)
-------------------------
* added missing 'All rights reserved' tag, added LICENSE.rst file
* Contributors: Marcel Debout

0.0.32 (2016-03-30 15:11)
-------------------------
* README.rst edited online with Bitbucket
* Contributors: Marcel Debout

0.0.31 (2016-03-30 15:01)
-------------------------
* README.rst edited online with Bitbucket
* Contributors: Marcel Debout

0.0.30 (2016-03-30 14:44)
-------------------------
* moved all logos into one file
* Contributors: Marcel Debout

0.0.29 (2016-03-30 13:41)
-------------------------
* added missing wiki_images
* Contributors: Marcel Debout

0.0.28 (2016-03-30 13:31)
-------------------------
* new logos for the documentation
* README.rst edited online with Bitbucket
* Contributors: Marcel Debout

0.0.27 (2016-03-30 11:31)
-------------------------
* edited README, added license text to all files
* Contributors: Marcel Debout

0.0.26 (2016-03-30 10:22)
-------------------------
* moved README to .rst and merged package.xml
* README.md edited online with Bitbucket
* README.md edited online with Bitbucket
* Contributors: Marcel Debout

0.0.25 (2016-03-29)
-------------------
* implemented setBinning -> be careful: CamerInfo now changes binning_x & binning_y entry while the image height and width keeps static
* Contributors: Marcel Debout

0.0.24 (2016-03-17 14:21)
-------------------------
* size of provided data through GrabImagesAction should only be checked, if the corresponding 'is_given' flag is true
* Contributors: Marcel Debout

0.0.23 (2016-03-17 12:41)
-------------------------
* fixed mapping in GrabImagesAction from deprecated to new interface, fixed error in case that values are not provided and the resulting vector size is NOT 0, but 1
* Contributors: Marcel Debout

0.0.22 (2016-03-16)
-------------------
* smarter behaviour, if the goal values of the GrabImagesAction doesn't make sense
* Contributors: Marcel Debout

0.0.21 (2016-03-15 12:52)
-------------------------
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* warnings are not errors
* Contributors: Marcel Debout

0.0.20 (2016-03-15 11:02)
-------------------------
* compiles without warnings (no return value)
* merged the two branches
* adapted device removal behaviour
* 'is deprecated' error is now a 'is deprecated' warning'
* added deprecated handling of 'set_brightness_srv', 'set_exposure_srv' and 'set_sleeping_srv', which now can be found under 'set_brightness', 'set_exposure' and 'set_sleeping'. Furthermore the usage of 'SetBrightnessSrv.srv', 'SetExposureSrv.srv' and 'SetSleepingSrv.srv' is deprecated and should be switched to 'SetBrightness.srv', SetExposure.srv' and 'SetSleeping.srv'
* implemented setBinning as runtime parameter, but finally realized that the camera does not support it. Hence the camera has to be closed and reopened to be able to set the binning. This will be a future feature
* realized new fast opening behaviour, Basler-Feedback was: Sfnc is outdated, so I replaced it using the DeviceClass and the ModelName. Futhermore its possible to detect the desired camera without opening it twice
* increased fail_safe_ctr for dart cameras -> manual: up to 50 frames needed to reach target for dart cameras
* splitted grabImagesRawActionExecuteCB() in two methods, so that it can also be called from the derived PylonCameraOpenCV class
* moved output to #if DEBUG
* did lots of changes but finally I found a logic behaviour!
* linting & formatting
* added setGamma functionallity
* finally found out that the best is to keep default camera settings as long as possible. Added lots of commands to the default config file, hopefully one can verify my thoughts ;-)
* removed outdated scripts from CMakeLists.txt
* making roslint happy
* removed outdated scripts, brightness tests are coveraged in magazino_tests, exp_caller depends maru stuff
* removed test depend, all tests are done in magazino_tests/pylon_camera_tests
* finally got a state, where brightness tests for usb & gigE are running successfull, have still problems with dart cameras
* 0.0.17
* README.md wurden online mit Bitbucket bearbeitet
* removed has_auto_exposure\_ member, because this happens already in GenAPI::isAvailable(cam\_->ExposureAuto), added getter for cam\_->AutoGainUpper & Lower limit, added throwing of std::runtime_errors
* searching for autoBrightnessFunction stuck for dart cameras
* clean up dart
* disabled gainselector setting, because each gige cam has its differen naming
* removed senseless getCurrentExp, Gain... functions, correctly implemented setGain
* removed comments
* calling the grabImagesAction with differen exp-times will no longer affect the continiously published images
* further cleaning
* rows & cols are now size_t, removed unused checkForPylonAutoFunctionRunning()
* cleaning & renaming
* cleaned up the extended brightness search, works now very well!
* setExposure() on the pylon_camera-Object (not on PylonCameraNode) has now target and reached exposure
* enabled output
* fixed GainType-bug
* moved exp_search_params, continued working on brightness fix, still problems with dart
* CMakeLists.txt formatted
* dart camera starts with the same settings like the usb camera
* not all usb cameras have GainSelector_AnalogAll
* formatting
* seperated registerConfig, openCamera and applyStartupSettings
* added output regarding gain and exposure time, facing to problems in difference of usb and dart cams
* gain setting started, checking if gain db range gige equals usb
* check if auto function running not necessary any more
* brightness search now in a seperate thread, added lots of comments (and outpouts which i will remove when the gain stuff is working)
* removed auto-functions parameter limits for gige cameras
* gain for dart cameras not hard coded any more, one can set it in initializeing process using the ros-params
* changed order of setting target brightness value & setting the auto-funktion mode
* try to get rid of all these checkForAutoFuncitonRunning() functions using only one PylonCamera::isBrightnessFunctionRunning() method
* - output to check if auto-function still running
* - added const max allowed delta (tolerance) for the brightness search
  - switched from int-mean to float mean to decrease rounding errors
  - added comments / better readability
* further comments for brightness search
* 0.0.16
* Basler-Feedback: Prevent that the image will be copied twice:
  "
  Es handelt sich um ein Missverständnis. Bei dem Ausdruck „image = std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_); “passiert folgendes:
  1.  Konstruktor von std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_) aufrufen (1. Kopie der Bildaten)
  2.  Zuweisungsoperator von „image“ aufrufen (2. Kopie der Bildaten)
  3.  Destruktor von std::vector<uint8_t>() aufrufen (1. Kopie wird verworfen)
  Der Compiler hat unter Umständen die Möglichkeit hier zu optimieren, wenn die verwendete STL und der Compiler C++11 unterstützt. Da ab C++11 der Move Assignment operator (In der Mail stand „Move Constructor“) verfügbar ist (class_name & class_name :: operator= ( class_name && ) und der Compiler weiß das der R-Value std::vector<uint8_t>() nicht weiter referenziert wird, kann er einen Kopierschritt vermeiden.
  Vorschlag, einfach folgenden Ausdruck:
  image.assign(pImageBuffer, pImageBuffer + img_size_byte\_);
  statt:
  image = std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_);
  verwenden und das Problem ist erledigt.
  "
* removed brightnessValidation() because it's a one-liner
* activated new waitForCamera() function
* added waitForCamera(), which waits for pylon_camera\_->isReady() observing a given timeout
* comment on isReady()
* Basler-Email: cam\_->GetNodeMap().InvalidateNodes() should never be necessary, so I removed it
* resorted methods
* added comments
* Contributors: Magazino Version Daemon, Marcel Debout, Nikolas Engelhard

0.0.19 (2016-02-29)
-------------------
* new device removal behaviour
* Contributors: Marcel Debout

0.0.18 (2016-02-25)
-------------------
* try to catch the logical error exception in grabImagesRawExecuteCB()
* Contributors: Marcel Debout

0.0.17 (2016-02-19)
-------------------
* README.md wurden online mit Bitbucket bearbeitet
* Contributors: Nikolas Engelhard

0.0.16 (2016-02-02)
-------------------
* Basler-Feedback: Prevent that the image will be copied twice:
  "
  Es handelt sich um ein Missverständnis. Bei dem Ausdruck „image = std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_); “passiert folgendes:
  1.  Konstruktor von std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_) aufrufen (1. Kopie der Bildaten)
  2.  Zuweisungsoperator von „image“ aufrufen (2. Kopie der Bildaten)
  3.  Destruktor von std::vector<uint8_t>() aufrufen (1. Kopie wird verworfen)
  Der Compiler hat unter Umständen die Möglichkeit hier zu optimieren, wenn die verwendete STL und der Compiler C++11 unterstützt. Da ab C++11 der Move Assignment operator (In der Mail stand „Move Constructor“) verfügbar ist (class_name & class_name :: operator= ( class_name && ) und der Compiler weiß das der R-Value std::vector<uint8_t>() nicht weiter referenziert wird, kann er einen Kopierschritt vermeiden.
  Vorschlag, einfach folgenden Ausdruck:
  image.assign(pImageBuffer, pImageBuffer + img_size_byte\_);
  statt:
  image = std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_);
  verwenden und das Problem ist erledigt.
  "
* Contributors: Marcel Debout

0.0.15 (2016-02-01 15:33)
-------------------------
* added comment
* moved cam-info setup into new method
* Contributors: Marcel Debout

0.0.14 (2016-02-01 08:22)
-------------------------
* fixed brightness assertion bug: spinOnce() does not result in a new image in case that no subscriber listens to the image topic
* assertion before accumulating
* Contributors: Marcel Debout

0.0.13 (2016-01-25 17:03)
-------------------------
* set gain implemented for gige
* Contributors: Marcel Debout

0.0.12 (2016-01-25 13:32)
-------------------------
* added lots of comments, initialized the camera_info_msg with zero-values
* Contributors: Marcel Debout

0.0.11 (2016-01-21 18:02)
-------------------------
* removed roslint
* Contributors: Markus Grimm

0.0.10 (2016-01-21 15:22)
-------------------------
* SetUserOutput is now a service
* Contributors: Markus Grimm

0.0.9 (2016-01-21 11:51)
------------------------
* README.md edited online with Bitbucket
* Contributors: Nikolas Engelhard

0.0.8 (2016-01-19 18:54)
------------------------
* fixed segfault if no camera-present-bug
* undo set gain for gige
* Contributors: Marcel Debout

0.0.7 (2016-01-19 18:23)
------------------------
* gain to 100 for gige hotfix
* Contributors: Marcel Debout

0.0.6 (2016-01-18 11:02)
------------------------
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* catkin_lint fix
* Contributors: Marcel Debout

0.0.5 (2016-01-18 10:36)
------------------------
* removed all tests, they are now in the new package: pylon_camera_tests to resolve can-dependency-problem
* Contributors: Marcel Debout

0.0.4 (2016-01-15 18:41)
------------------------
* Reviewed ROBEE-212: Found the missing part in order to use the trait
* Removed compaibilty_exposure_action.py as it is outdated (it used the old pylon_camera_msgs package)
* Contributors: Markus Grimm

0.0.3 (2016-01-15 17:12)
------------------------
* Robee-212: Support for setting the digital output pin of USB (non-Dart) and GigE cameras. So far, the std_msgs/Bool topic output_1 can be used to set the pin. Only tested on USB3-Ace camera "
* Contributors: Nikolas Engelhard

0.0.2 (2016-01-13)
------------------
* formatted cmakelist
* check if env: ON_JENKINS_TESTRIG=true before running the tests. if not, tests will have state: 'SUCCESS', but the number of test remains 0
* removed useless error-msg if no camera is present
* Contributors: Marcel Debout

0.0.1 (2016-01-11)
------------------
