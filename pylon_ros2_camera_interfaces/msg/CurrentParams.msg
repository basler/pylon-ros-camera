uint32 offset_x # -20000 = Error	
uint32 offset_y # -20000 = Error
bool reverse_x
bool reverse_y

int32 black_level # -10000 = error/not available

int32 pgi_mode # -3 = Unknown, -2 = Error, -1 = Not available, 0 = Off, 1 = On
int32 demosaicing_mode # -3 = Unknown, -2 = Error, -1 = Not available, 0 = Simple, 1 = BaslerPGI
float32 noise_reduction # -20000.0 = Error, -10000.0 = Not available 
float32 sharpness_enhancement # -20000.0 = Error, -10000.0 = Not available 
int32 light_source_preset # -3 = Unknown, -2 = Error, -1 = Not available, 0 = Off, 1 = Daylight5000K, 2 = Daylight6500K, 3 = Tungsten2800K
int32 balance_white_auto # -3 = Unknown, -2 = Error, -1 = Not available, 0 = Off, 1 = Once, 2 = Continuous

int32 sensor_readout_mode # -3 = Unknown, -2 = Error, -1 = Not available, 0 = Normal, 1 = Fast
int32 acquisition_frame_count # -20000 = Error, -10000 = Not available 
int32 trigger_selector # -3 = Unknown, -2 = Error, -1 = Not available, 0 = FrameStart, 1 = FrameBurstStart(USB)/AcquisitionStart(GigE)
int32 trigger_mode # -3 = Unknown, -2 = Error, -1 = Not available, 0 = Off, 1 = On
int32 trigger_source # -3 = Unknown, -2 = Error, -1 = Not available, 0 = Software, 1 = Line1, 2 = Line3, 3 = Line4, 4 = Action1(Selected Gige)
int32 trigger_activation # -3 = Unknown, -2 = Error, -1 = Not available, 0 = RisingEdge, 1 = FallingEdge
float32 trigger_delay # -20000.0 = Error, -10000.0 = Not available 

int32 user_set_selector # -3 = Unknown, -2 = Error, -1 = Not available, 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
int32 user_set_default_selector # -3 = Unknown, -2 = Error, -1 = Not available, 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw 

bool is_sleeping
float32 brightness
float32 exposure
float32 gain
float32 gamma 
uint32 binning_x
uint32 binning_y
float32 temperature # Shows the camera temperature. If not available, then 0.0. USB uses DeviceTemperature and GigE TemperatureAbs parameters.
int32 max_num_buffer		# -2 = Error, -1 = Not available
sensor_msgs/RegionOfInterest roi

string[] available_image_encoding
string current_image_encoding
string current_image_ros_encoding

string ptp_status           # latched state of the PTP clock, see https://ja.docs.baslerweb.com/pylonapi/net/T_Basler_Pylon_PLCamera_PtpStatusEnum
string ptp_servo_status     # latched state of the clock servo, see https://docs.baslerweb.com/pylonapi/net/T_Basler_Pylon_PLCamera_PtpServoStatusEnum
int64 ptp_offset            # ptp offset from master in ticks [ns]

# 3D-only parameters (populated when a 3D camera is connected; -1 otherwise).
float32 depth_min           # working depth range minimum, camera-native units (mm: blaze/mini, m: stereo ace). -1 = not available
float32 depth_max           # working depth range maximum, camera-native units (mm: blaze/mini, m: stereo ace). -1 = not available
int32 operating_mode        # -1 = not available, 0 = ShortRange, 1 = LongRange (blaze)
int32 hdr_mode              # -1 = not available, 0 = Off, 1 = On (blaze)
int32 fast_mode             # -1 = not available, 0 = Off, 1 = On (blaze)
int32 spatial_filter        # -1 = not available, 0 = Off, 1 = On (blaze)
int32 temporal_filter       # -1 = not available, 0 = Off, 1 = On (blaze)
int32 outlier_removal       # -1 = not available, 0 = Off, 1 = On (blaze)
int32 ambiguity_filter      # -1 = not available, 0 = Off, 1 = On (blaze)
float32 confidence_threshold # -1.0 = not available (blaze)
int32 projector_enable      # -1 = not available, 0 = Off, 1 = On (stereo mini)
int32 projector_level       # -1 = not available, pattern projector power level (stereo mini)
int32 depth_preset          # -1 = not available, BslDepthPreset index (stereo mini)
int32 illumination_mode     # -1 = not available, BslIlluminationMode index (stereo ace)
int32 depth_quality         # -1 = not available, BslDepthQuality index (stereo ace)
int32 static_scene          # -1 = not available, 0 = Off, 1 = On (stereo ace)

bool success
string message

