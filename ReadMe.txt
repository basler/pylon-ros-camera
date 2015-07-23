If build fails with 

/pylon_camera/src/pylon_interface.cpp:446: error: 'Sfnc_2_1_0' was not declared in this scope
         else if (sfnc_version == Sfnc_2_1_0)
                               
sudo cp pylon_camera/SfncVersion.h /opt/pylon4/include/pylon/
