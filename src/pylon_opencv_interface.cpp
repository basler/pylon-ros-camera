/*
 * pylon_opencv_interface.cpp
 *
 *  Created on: May 26, 2015
 *      Author: md
 */

#include <pylon_camera/pylon_opencv_interface.h>

namespace pylon_camera
{

PylonOpenCVInterface::PylonOpenCVInterface() :
                    img_sequence_()
{
    // TODO Auto-generated constructor stub
}
PylonOpenCVInterface::~PylonOpenCVInterface()
{
    // TODO Auto-generated destructor stub
}
int PylonOpenCVInterface::initialize(const PylonCameraParameter &params)
{
    if (PylonInterface::initialize(params))
    {
        cerr << "Error while initializing the base Pylon Interface" << endl;
    }

    int exit_code = 0;

    return exit_code;
}
int PylonOpenCVInterface::initSequencer(const PylonCameraParameter &params)
{
    switch (cam_type_)
    {
        case GIGE:
            try
            {
                if (GenApi::IsWritable(gige_cam_->SequenceEnable))
                {

                }
            }
            catch (GenICam::GenericException &e)
            {
                cerr << e.GetDescription() << endl;
                return 1;
            }
            break;
        case USB:
            try
            {

                if (GenApi::IsWritable(usb_cam_->SequencerMode))
                {
                    usb_cam_->SequencerMode.SetValue(Basler_UsbCameraParams::SequencerMode_Off);
                } else
                {
                    cerr << "Sequencer Mode not writable" << endl;
                }

                usb_cam_->SequencerConfigurationMode.SetValue(Basler_UsbCameraParams::SequencerConfigurationMode_On);

                // **** valid for all sets: reset on software signal 1 ****
                int64_t initial_set = usb_cam_->SequencerSetSelector.GetMin();

                usb_cam_->SequencerSetSelector.SetValue(initial_set);
                usb_cam_->SequencerPathSelector.SetValue(0);
                usb_cam_->SequencerSetNext.SetValue(initial_set);
                usb_cam_->SequencerTriggerSource.SetValue(Basler_UsbCameraParams::SequencerTriggerSource_SoftwareSignal1);
                // advance on Frame Start
                usb_cam_->SequencerPathSelector.SetValue(1);
                usb_cam_->SequencerTriggerSource.SetValue(Basler_UsbCameraParams::SequencerTriggerSource_FrameStart);
                // ********************************************************

                // Set the parameters for step 0: Exp1
                usb_cam_->SequencerSetNext.SetValue(1);
                PylonInterface::setExposure(1000);
                cout << "Set 1: Exposure:" << usb_cam_->ExposureTime.GetValue() << endl;
                usb_cam_->SequencerSetSave.Execute();

                usb_cam_->SequencerSetSelector.SetValue(1);
                usb_cam_->SequencerSetNext.SetValue(2);
                PylonInterface::setExposure(10000);
                cout << "Set 2: Exposure:" << usb_cam_->ExposureTime.GetValue() << endl;
                usb_cam_->SequencerSetSave.Execute();

                usb_cam_->SequencerSetSelector.SetValue(2);
                usb_cam_->SequencerSetNext.SetValue(0);
                PylonInterface::setExposure(50000);
                cout << "Set 3: Exposure:" << usb_cam_->ExposureTime.GetValue() << endl;
                usb_cam_->SequencerSetSave.Execute();

                // config finished
                usb_cam_->SequencerConfigurationMode.SetValue(Basler_UsbCameraParams::SequencerConfigurationMode_Off);
                usb_cam_->SequencerMode.SetValue(Basler_UsbCameraParams::SequencerMode_On);

            }
            catch (GenICam::GenericException &e)
            {
                cerr << "ERROR while initializing pylon sequencer:" << endl;
                cerr << e.GetDescription() << endl;
                return 1;
            }
            break;
        case DART:
            try
            {

            }
            catch (GenICam::GenericException &e)
            {
                cerr << e.GetDescription() << endl;
                return 1;
            }
            break;
        default:
            cerr << "Unknown Camera Type" << endl;
            break;
    }

    return 0;

}

bool PylonOpenCVInterface::grab(const PylonCameraParameter &params, cv::Mat &image)
{
    switch (cam_type_)
    {
        case GIGE:
            try
            {
                gige_cam_->ExecuteSoftwareTrigger();
                gige_cam_->RetrieveResult(gige_cam_->ExposureTimeAbs.GetMax() * 1.05,
                                          ptr_grab_result_,
                                          TimeoutHandling_ThrowException);
            }
            catch (GenICam::GenericException &e)
            {
                if (gige_cam_->IsCameraDeviceRemoved())
                {
                    is_cam_removed_ = true;
                }
                else
                {
                    cerr << "An image grabbing exception in pylon camera occurred:" << endl
                         << e.GetDescription()
                         << endl;
                    return false;
                }
            }
            break;
        case USB:
            try
            {
                usb_cam_->ExecuteSoftwareTrigger();
                usb_cam_->RetrieveResult(usb_cam_->ExposureTime.GetMax() * 1.05,
                                         ptr_grab_result_,
                                         TimeoutHandling_ThrowException);
            }
            catch (GenICam::GenericException &e)
            {
                if (usb_cam_->IsCameraDeviceRemoved())
                {
                    is_cam_removed_ = true;
                }
                else
                {
                    cerr << "An image grabbing exception in pylon camera occurred:" << endl
                         << e.GetDescription()
                         << endl;
                    return false;
                }
            }
            break;
        case DART:
            try
            {
                dart_cam_->ExecuteSoftwareTrigger();
                dart_cam_->RetrieveResult(dart_cam_->ExposureTime.GetMax() * 1.05,
                                          ptr_grab_result_,
                                          TimeoutHandling_ThrowException);
            }
            catch (GenICam::GenericException &e)
            {
                if (dart_cam_->IsCameraDeviceRemoved())
                {
                    is_cam_removed_ = true;
                }
                else
                {
                    cerr << "An image grabbing exception in pylon camera occurred:" << endl
                         << e.GetDescription()
                         << endl;
                    return false;
                }
            }
            break;
        default:
            cerr << "Unknown Camera Type" << endl;
            break;
    }

    if (ptr_grab_result_->GrabSucceeded())
    {
        //TODO: wird hier neuer Speicher allokiert? Kann der bereits allokierte weiterverwendet werden?
//        const uint8_t *image_buffer = ptr_grab_result_->GetBuffer();
        image = cv::Mat(img_rows_, img_cols_, CV_8UC1);
        memcpy(image.ptr(), ptr_grab_result_->GetBuffer(), img_size_byte_);
//        image = std::vector<uint8_t>(pImageBuffer, image_buffer + img_size_byte_);
        return true;
    }
    else
    {
        cout << "Error: " << ptr_grab_result_->GetErrorCode() << " "
             << ptr_grab_result_->GetErrorDescription()
             << endl;
        return false;
    }

    return false;
}
//int PylonOpenCVInterface::terminate(const PylonCameraParameter &params)
//{
//    if(!params.use_sequencer_){
//        return 0;
//    }
//    try
//    {
//        switch (cam_type_)
//        {
//            case GIGE:
//                cout << "SEQUENCER TERMINATION FOR GIGE NOT YET IMPLEMENTED" << endl;
//                break;
//            case USB:
//                cout << "switching sequencer: OFF" << endl;
//                usb_cam_->SequencerMode.SetValue(Basler_UsbCameraParams::SequencerMode_Off);
//                break;
//            case DART:
//                cout << "SEQUENCER TERMINATION FOR DART NOT YET IMPLEMENTED" << endl;
//                break;
//            default:
//                cerr << "Unknown Camera Type" << endl;
//                break;
//        }
//    }
//    catch (GenICam::GenericException &e)
//    {
//        cerr << "Error while terminating Sequencer: " << endl;
//        cerr << e.GetDescription() << endl;
//        return 1;
//    }
//    return 0;
//}
} /* namespace pylon_camera */
