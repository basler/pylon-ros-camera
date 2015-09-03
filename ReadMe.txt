If build fails with 

/pylon_camera/src/pylon_interface.cpp:446: error: 'Sfnc_2_1_0' was not declared in this scope
         else if (sfnc_version == Sfnc_2_1_0)
                               
sudo cp pylon_camera/SfncVersion.h /opt/pylon4/include/pylon/


Using HDR:
 - Install OpenCV 3
 - Use these parameters in launch-file:
<param name="output_hdr_img" value="true" type="bool" />
<param name="use_sequencer" value="true" type="bool" />
<rosparam param="desired_seq_exp_times">[1000, 2000, 3000]</rosparam> <!-- 1000,2000,3000 exposure times for HDR -->

