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
                    img_sequence_(),
                    own_brightness_search_running_(false),
                    exp_search_params_()
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
void PylonOpenCVInterface::setupExtendedBrightnessSearch(int &brightness)
{
    double brightness_f = brightness / 255.0;

    switch (cam_type_)
    {
        case GIGE:
            {
            if (gige_cam_->AutoTargetValue.GetMin() > brightness)
            {
                gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Once);
                gige_cam_->AutoTargetValue.SetValue(gige_cam_->AutoTargetValue.GetMin(), false);
                cout << "Desired brightness " << brightness
                     << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                     << endl;
                own_brightness_search_running_ = true;
            } else if (gige_cam_->AutoTargetValue.GetMax() < brightness)
            {
                gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Once);
                gige_cam_->AutoTargetValue.SetValue(gige_cam_->AutoTargetValue.GetMax(), false);
                cout << "Desired brightness " << brightness
                     << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                     << endl;
                own_brightness_search_running_ = true;
            } else
            {
                cerr << "ERROR unexpected brightness case" << endl;
            }
            break;
        }
        case USB:
            {
            if (usb_cam_->AutoTargetBrightness.GetMin() > brightness_f)
            {
                usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Once);
                usb_cam_->AutoTargetBrightness.SetValue(usb_cam_->AutoTargetBrightness.GetMin(), false);
                cout << "Desired brightness " << brightness
                     << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                     << endl;
                own_brightness_search_running_ = true;
            } else if (usb_cam_->AutoTargetBrightness.GetMax() < brightness_f)
            {
                usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Once);
                usb_cam_->AutoTargetBrightness.SetValue(usb_cam_->AutoTargetBrightness.GetMax(), false);
                cout << "Desired brightness " << brightness
                     << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                     << endl;
                own_brightness_search_running_ = true;
            } else
            {
                cerr << "ERROR unexpected brightness case" << endl;
            }
            break;
        }
        case DART:
            {
            if (dart_cam_->AutoTargetBrightness.GetMin() > brightness_f)
            {
                dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Once);
                dart_cam_->AutoTargetBrightness.SetValue(dart_cam_->AutoTargetBrightness.GetMin(), false);
                cout << "Desired brightness " << brightness
                     << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                     << endl;
                own_brightness_search_running_ = true;
            } else if (dart_cam_->AutoTargetBrightness.GetMax() < brightness_f)
            {
                dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Once);
                dart_cam_->AutoTargetBrightness.SetValue(dart_cam_->AutoTargetBrightness.GetMax(), false);
                cout << "Desired brightness " << brightness
                     << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                     << endl;
                own_brightness_search_running_ = true;
            } else
            {
                cerr << "ERROR unexpected brightness case" << endl;
            }
            break;
        }
        default:
            break;
    }
    return;
}

// Own Auto Function for average brightness values out of the Pylon Range ([50-205])
bool PylonOpenCVInterface::setExtendedBrightness(int& brightness)
{
    switch (cam_type_)
    {
        case GIGE:
            {
            if (!exp_search_params_.is_initialized_)
            {
                gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Off);
                if (!GenApi::IsWritable(gige_cam_->ExposureTimeAbs))
                {
                    cerr << "Pylon Exposure Auto Node not writable in own auto-exp-function!" << endl;
                    return false;
                }
                if (gige_cam_->AutoTargetValue.GetMin() > brightness)
                {
                    exp_search_params_.initialize(brightness,
                                                  gige_cam_->ExposureTimeAbs.GetMin(),
                                                  gige_cam_->ExposureTimeAbs.GetValue(),
                                                  gige_cam_->ExposureTimeAbs.GetValue(),
                                                  exp_search_params_.current_brightness_);
                } else
                {
                    if (brightness > 255)
                    {
                        brightness = 255;
                    }
                    exp_search_params_.initialize(brightness,
                                                  gige_cam_->ExposureTimeAbs.GetValue(),
                                                  gige_cam_->ExposureTimeAbs.GetMax(),
                                                  gige_cam_->ExposureTimeAbs.GetValue(),
                                                  exp_search_params_.current_brightness_);
                }
            }

            if (fabs(exp_search_params_.goal_brightness_ - exp_search_params_.current_brightness_) < 1)
            {
                own_brightness_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                brightness = exp_search_params_.current_brightness_;
                cout << "Own Auto Function: Success! Goal = " << brightness << endl;
                return true;
            }
            if (exp_search_params_.last_unchanged_exposure_counter_ > 2)
            {
                own_brightness_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                exp_search_params_.last_unchanged_exposure_counter_ = 0;
                brightness = exp_search_params_.current_brightness_;
                cout << "Own Auto Function: Success! Goal = " << brightness << endl;
                return true;
            }

            exp_search_params_.updateBinarySearch();

            // truncate desired exposure if out of range
            if (exp_search_params_.desired_exposure_ < gige_cam_->ExposureTimeAbs.GetMin() || exp_search_params_
                            .desired_exposure_
                                                                                              > gige_cam_->ExposureTimeAbs
                                                                                                              .GetMax())
            {
                if (exp_search_params_.desired_exposure_ < gige_cam_->ExposureTimeAbs.GetMin())
                {
                    cout << "Desired mean brightness unreachable! Min possible exposure = "
                         << gige_cam_->ExposureTimeAbs.GetMin()
                         << ". Will limit to this value." << endl;
                    exp_search_params_.desired_exposure_ = gige_cam_->ExposureTimeAbs.GetMin();
                } else if (exp_search_params_.desired_exposure_ > gige_cam_->ExposureTimeAbs.GetMax())
                {
                    cout << "Desired mean brightness unreachable! Max possible exposure = "
                         << gige_cam_->ExposureTimeAbs.GetMax()
                         << ". Will limit to this value." << endl;
                    exp_search_params_.desired_exposure_ = gige_cam_->ExposureTimeAbs.GetMax();
                }
            }
            // Current exposure  = min/max limit value -> auto function finished -> update brightness param
            if (exp_search_params_.current_exposure_ == gige_cam_->ExposureTimeAbs.GetMin() || exp_search_params_
                            .current_exposure_
                                                                                               == gige_cam_->ExposureTimeAbs
                                                                                                               .GetMax())
            {
                own_brightness_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                cout << "WILL USE SMALLES EXP POSSIBLE!!!" << endl;
                brightness = exp_search_params_.current_brightness_;
                return true;

            }

            //gige_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
            gige_cam_->ExposureTimeAbs.SetValue(exp_search_params_.desired_exposure_);
            // Update GeniCam Cache with GetNodeMap().InvalidateNodes()
            gige_cam_->GetNodeMap().InvalidateNodes();
            // Attention: Setting and Getting exposure not necessary the same: Difference of up to 35.0 ms
            exp_search_params_.last_exposure_ = exp_search_params_.current_exposure_;
            exp_search_params_.current_exposure_ = gige_cam_->ExposureTimeAbs.GetValue();

            return false;
        }

        case USB:
            {
            double brightness_f = brightness / 255.0;

            if (!exp_search_params_.is_initialized_)
            {
                usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                if (!GenApi::IsWritable(usb_cam_->ExposureTime))
                {
                    cerr << "Pylon Exposure Auto Node not writable in own auto-exp-function!" << endl;
                    return false;
                }
                if (usb_cam_->AutoTargetBrightness.GetMin() > brightness_f)
                {
                    exp_search_params_.initialize(brightness,
                                                  usb_cam_->ExposureTime.GetMin(),
                                                  usb_cam_->ExposureTime.GetValue(),
                                                  usb_cam_->ExposureTime.GetValue(),
                                                  exp_search_params_.current_brightness_);
                } else
                {
                    if (brightness > 255)
                    {
                        brightness = 255;
                    }
                    exp_search_params_.initialize(brightness,
                                                  usb_cam_->ExposureTime.GetValue(),
                                                  usb_cam_->ExposureTime.GetMax(),
                                                  usb_cam_->ExposureTime.GetValue(),
                                                  exp_search_params_.current_brightness_);
                }
            }

            if (fabs(exp_search_params_.goal_brightness_ - exp_search_params_.current_brightness_) < 1)
            {
                own_brightness_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                brightness = exp_search_params_.current_brightness_;
                cout << "Own Auto Function: Success! Goal = " << brightness << endl;
                return true;
            }
            if (exp_search_params_.last_unchanged_exposure_counter_ > 2)
            {
                own_brightness_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                exp_search_params_.last_unchanged_exposure_counter_ = 0;
                brightness = exp_search_params_.current_brightness_;
                cout << "Own Auto Function: Success! Goal = " << brightness << endl;
                return true;
            }

            exp_search_params_.updateBinarySearch();

            // truncate desired exposure if out of range
            if (exp_search_params_.desired_exposure_ < usb_cam_->ExposureTime.GetMin() || exp_search_params_
                            .desired_exposure_
                                                                                          > usb_cam_->ExposureTime
                                                                                                          .GetMax())
            {
                if (exp_search_params_.desired_exposure_ < usb_cam_->ExposureTime.GetMin())
                {
                    cout << "Desired mean brightness unreachable! Min possible exposure = "
                         << usb_cam_->ExposureTime.GetMin()
                         << ". Will limit to this value." << endl;
                    exp_search_params_.desired_exposure_ = usb_cam_->ExposureTime.GetMin();
                } else if (exp_search_params_.desired_exposure_ > usb_cam_->ExposureTime.GetMax())
                {
                    cout << "Desired mean brightness unreachable! Max possible exposure = "
                         << usb_cam_->ExposureTime.GetMax()
                         << ". Will limit to this value." << endl;
                    exp_search_params_.desired_exposure_ = usb_cam_->ExposureTime.GetMax();
                }
            }
            // Current exposure  = min/max limit value -> auto function finished -> update brightness param
            if (exp_search_params_.current_exposure_ == usb_cam_->ExposureTime.GetMin() || exp_search_params_
                            .current_exposure_
                                                                                           == usb_cam_->ExposureTime
                                                                                                           .GetMax())
            {
                own_brightness_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                cout << "WILL USE SMALLES EXP POSSIBLE!!!" << endl;
                brightness = exp_search_params_.current_brightness_;
                return true;

            }

            //usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
            usb_cam_->ExposureTime.SetValue(exp_search_params_.desired_exposure_);
            // Update GeniCam Cache with GetNodeMap().InvalidateNodes()
            usb_cam_->GetNodeMap().InvalidateNodes();
            // Attention: Setting and Getting exposure not necessary the same: Difference of up to 35.0 ms
            exp_search_params_.last_exposure_ = exp_search_params_.current_exposure_;
            exp_search_params_.current_exposure_ = usb_cam_->ExposureTime.GetValue();

            return false;
        }
        case DART:
            {
            double brightness_f = brightness / 255.0;

            if (!exp_search_params_.is_initialized_)
            {
                dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                if (!GenApi::IsWritable(dart_cam_->ExposureTime))
                {
                    cerr << "Pylon Exposure Auto Node not writable in own auto-exp-function!" << endl;
                    return false;
                }
                if (dart_cam_->AutoTargetBrightness.GetMin() > brightness_f)
                {
                    exp_search_params_.initialize(brightness,
                                                  dart_cam_->ExposureTime.GetMin(),
                                                  dart_cam_->ExposureTime.GetValue(),
                                                  dart_cam_->ExposureTime.GetValue(),
                                                  exp_search_params_.current_brightness_);
                } else
                {
                    if (brightness > 255)
                    {
                        brightness = 255;
                    }
                    exp_search_params_.initialize(brightness,
                                                  dart_cam_->ExposureTime.GetValue(),
                                                  dart_cam_->ExposureTime.GetMax(),
                                                  dart_cam_->ExposureTime.GetValue(),
                                                  exp_search_params_.current_brightness_);
                }
            }

            if (fabs(exp_search_params_.goal_brightness_ - exp_search_params_.current_brightness_) < 1)
            {
                own_brightness_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                brightness = exp_search_params_.current_brightness_;
                cout << "Own Auto Function: Success! Goal = " << brightness << endl;
                return true;
            }
            if (exp_search_params_.last_unchanged_exposure_counter_ > 2)
            {
                own_brightness_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                exp_search_params_.last_unchanged_exposure_counter_ = 0;
                brightness = exp_search_params_.current_brightness_;
                cout << "Own Auto Function: Success! Goal = " << brightness << endl;
                return true;
            }

            exp_search_params_.updateBinarySearch();

            // truncate desired exposure if out of range
            if (exp_search_params_.desired_exposure_ < dart_cam_->ExposureTime.GetMin() || exp_search_params_
                            .desired_exposure_
                                                                                           > dart_cam_->ExposureTime
                                                                                                           .GetMax())
            {
                if (exp_search_params_.desired_exposure_ < dart_cam_->ExposureTime.GetMin())
                {
                    cout << "Desired mean brightness unreachable! Min possible exposure = "
                         << dart_cam_->ExposureTime.GetMin()
                         << ". Will limit to this value." << endl;
                    exp_search_params_.desired_exposure_ = dart_cam_->ExposureTime.GetMin();
                } else if (exp_search_params_.desired_exposure_ > dart_cam_->ExposureTime.GetMax())
                {
                    cout << "Desired mean brightness unreachable! Max possible exposure = "
                         << dart_cam_->ExposureTime.GetMax()
                         << ". Will limit to this value." << endl;
                    exp_search_params_.desired_exposure_ = dart_cam_->ExposureTime.GetMax();
                }
            }
            // Current exposure  = min/max limit value -> auto function finished -> update brightness param
            if (exp_search_params_.current_exposure_ == dart_cam_->ExposureTime.GetMin() || exp_search_params_
                            .current_exposure_
                                                                                            == dart_cam_->ExposureTime
                                                                                                            .GetMax())
            {
                own_brightness_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                cout << "WILL USE SMALLES EXP POSSIBLE!!!" << endl;
                brightness = exp_search_params_.current_brightness_;
                return true;

            }

            //dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
            dart_cam_->ExposureTime.SetValue(exp_search_params_.desired_exposure_);
            // Update GeniCam Cache with GetNodeMap().InvalidateNodes()
            dart_cam_->GetNodeMap().InvalidateNodes();
            // Attention: Setting and Getting exposure not necessary the same: Difference of up to 35.0 ms
            exp_search_params_.last_exposure_ = exp_search_params_.current_exposure_;
            exp_search_params_.current_exposure_ = dart_cam_->ExposureTime.GetValue();

            return false;
        }
        default:
            break;
    }

    return false;
}

//bool PylonOpenCVInterface::isAutoBrightnessFunctionRunning()
//{
//    if (own_brightness_search_running_)
//    {
//        is_pylon_auto_function_running_ = true;
//    }
//    else
//    {
//        switch (cam_type_)
//        {
//            case GIGE:
//            {
//                is_pylon_auto_function_running_ = !(gige_cam_->ExposureAuto.GetValue()
//                                == Basler_GigECameraParams::ExposureAuto_Off);
//                break;
//            }
//            case USB:
//            {
//                is_pylon_auto_function_running_ = !(usb_cam_->ExposureAuto.GetValue()
//                                == Basler_UsbCameraParams::ExposureAuto_Off);
//                break;
//            }
//            case DART:
//            {
//                is_pylon_auto_function_running_ = !(dart_cam_->ExposureAuto.GetValue()
//                                == Basler_UsbCameraParams::ExposureAuto_Off);
//                break;
//            }
//            default:
//                break;
//        }
//
//    }
//    return is_pylon_auto_function_running_;
//}

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

        cv::imshow("view", image);
        cv::waitKey(1);
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
