====
**Message package for camera drivers**
====

This package contains message and service definitions for interacting with the pylon_camera driver.

This driver was improved by [drag and bot GmbH](www.dragandbot.com) from the version originally released by [Magazino GmbH](https://github.com/magazino/camera_control_msgs).

******
**Services**
******
- **SetBinning**
  Binning factor to get downsampled images. It refers here to any camera setting which combines rectangular neighborhoods of pixels into larger 'super-pixels'. It reduces the resolution of the output image to (width / binning_x) x (height / binning_y). The default values binning_x = binning_y = 0 are considered the same as binning_x = binning_y = 1 (no subsampling). Calling this service with target binning values will change the binning entry in the published camera_info_msg of the camera.

- **SetBrightness**
  The target brightness, which is average intensity values of the images. It depends the exposure time as well as the gain setting.
  The brightness_continuous flag controls the auto brightness function. If it is set to false, the given brightness will only be reached once.
  Hence changing light conditions lead to changing brightness values. If it is set to true, the given brightness will be reached continuously,
  trying to adapt to changing light conditions. The 'brightness_contunuous' mode is is only possible for values in the possible auto range of the camera which is e.g. [50 - 205].
  If the camera should try reach or keep the desired brightness, hence adapting to changing light conditions, at least one of the following flags **MUST** be set. If both are set, the interface will use the profile that tries to keep the gain at minimum to reduce white noise. 'exposure_auto' will adapt the exposure time to reach the brightness, wheras 'gain_auto' does so by adapting the gain.

- **SetExposure**
  The target exposure time measured in microseconds. If the limits were exceeded, the desired exposure time will be truncated.

- **SetGain**
  The target gain in percent of the maximal value the camera supports.

- **SetGamma**
  The target gamma correction of pixel intensity. Adjusts the brightness of the pixel values output by the camera's sensor to account for a non-linearity in the human perception of brightness or of the display system (such as CRT).

- **SetSleeping**
  If the camera runs in topic mode (continuously publishing images over the topics respecting the desired frame_rate) this service offers the posibillity to pause the image acquisition. To restart the grabbing, this service should be called again with set_sleeping set to false

- **SetIntegerValue**
  Used by below services:
  - set_black_level ,to change the overall brightness of an image by changing the gray values of the pixels by a specified amount.
  - set_acquisition_frame_count , to set the camera frame count  
  - set_offset_x , to set the camera image offset in x-axis.
  - set_offset_y , to set the camera image offset in y-axis.
  - set_demosaicing_mode , to set the camera demosaicing feature.
  - set_light_source_preset , to correct color shifts caused by certain light sources.
  - set_balance_white_auto , to automatically corrects color shifts in images acquired.
  - set_sensor_readout_mode , to choose between sensor readout modes that provide different sensor readout times.
  - set_trigger_selector , to select the trigger type.
  - set_trigger_source , to configure how the currently selected trigger can be triggered.
  - set_trigger_activation , to specify whether a trigger becomes active when the trigger signal rises or when it falls.
  - set_line_selector , to select the I/O line that you want to configure.
  - set_line_mode , to configure whether an I/O line is used as input or output.
  - set_line_source , to configure which signal to output on the I/O line currently selected.
  - set_device_link_throughput_limit , to set the max. amount of data (in bytes per second) that the camera could generate
  - select_user_set , to select the user set that you want to load or save.
  - select_default_user_set , to select the default user set that will be loaded at the camera startup.

- **SetFloatValue**
  Used by below services:
  - set_trigger_delay , to add a delay between the receipt of a hardware trigger signal or an action command signal and the moment the trigger becomes active.
  - set_line_debouncer_time , to filter out unwanted short hardware input signals.
  - set_noise_reduction , to reduces random variations in brightness or color information in your images.
  - set_sharpness_enhancement , to increases the sharpness of the images.

- **SetWhiteBalance**
  - The Balance White camera feature allows you to manually correct color shifts so that white objects appear white in images acquired.
  - For this purpose, a digital gain correction can be applied per color channel (red, green, blue).
  - The increase or decrease in intensity is proportional. For example, if the balance ratio for a color is set to 1.2, the intensity of that color is increased by 20 %.
  - NOTE: calling this service will turn off the "Balance White Auto"

******
**Actions**
******
- **GrabImages**
  Action to grab one or several images with desired image-intensity-setting each.
