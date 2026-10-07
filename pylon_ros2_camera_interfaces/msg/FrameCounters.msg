# Counters that the camera attaches to every frame as chunk data.
#
# header.stamp is the very same stamp carried by the image message of this frame, so
# downstream nodes pair the two by exact equality and not by a tolerance window
# (sensor_msgs/Image has no seq field in ROS 2).
std_msgs/Header header

# Each counter is enabled independently, since a camera model may provide one chunk and
# not the other. A counter this camera does not provide is reported as -1.

# Frames delivered by this camera. Restarts with the acquisition. -1 if not provided.
int64 frame_counter

# Trigger pulses received on the trigger line since the camera was powered on. It keeps
# counting across stop_grabbing / start_grabbing, so cameras wired to the same trigger
# line share this index up to a constant offset fixed at power-on. -1 if not provided.
int64 trigger_input_counter
