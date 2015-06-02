/*
 * pylon_single_exposure_interface.cpp
 *
 *  Created on: May 26, 2015
 *      Author: md
 */

#include <pylon_camera/pylon_single_exposure_interface.h>

namespace pylon_camera {

PylonSingleExposureInterface::PylonSingleExposureInterface(){
	// TODO Auto-generated constructor stub
}

PylonSingleExposureInterface::~PylonSingleExposureInterface() {
	// TODO Auto-generated destructor stub
}
int PylonSingleExposureInterface::setupCameraConfiguration(const PylonCameraParameter &params) {
	switch (cam_type_) {
	case GIGE:
		break;
	case USB:
		try {
			usb_cam_->RegisterConfiguration(new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
			usb_cam_->Open();
			usb_cam_->StartGrabbing();
			img_rows_ = (int) usb_cam_->Height.GetValue();
			img_cols_ = (int) usb_cam_->Width.GetValue();
			img_encoding_ = usb_cam_->PixelFormat.GetValue();
			img_pixel_depth_ = usb_cam_->PixelSize.GetValue();
			max_framerate_ = usb_cam_->ResultingFrameRate.GetValue();
		} catch (GenICam::GenericException &e) {
			cerr << e.GetDescription() << endl;
			return 1;
		}
		break;
	case DART:
		try {
			dart_cam_->RegisterConfiguration(new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
			dart_cam_->Open();
			dart_cam_->StartGrabbing();
			img_rows_ = (int) dart_cam_->Height.GetValue();
			img_cols_ = (int) dart_cam_->Width.GetValue();
			img_encoding_ = dart_cam_->PixelFormat.GetValue();
			img_pixel_depth_ = dart_cam_->PixelSize.GetValue();
			max_framerate_ = dart_cam_->ResultingFrameRate.GetValue();
		} catch (GenICam::GenericException &e) {
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
const uint8_t* PylonSingleExposureInterface::grab(const PylonCameraParameter &params) {
	switch (cam_type_) {
	case GIGE:
		break;
	case USB:
		try {
			usb_cam_->ExecuteSoftwareTrigger();
			usb_cam_->RetrieveResult(2000, ptr_grab_result_, TimeoutHandling_ThrowException);

			if (ptr_grab_result_->GrabSucceeded()) {
				const uint8_t *pImageBuffer = (uint8_t *) ptr_grab_result_->GetBuffer();
				return pImageBuffer;
			} else {
				cout << "Error: " << ptr_grab_result_->GetErrorCode() << " " << ptr_grab_result_->GetErrorDescription() << endl;
			}
		} catch (GenICam::GenericException &e) {
			cerr << "An image grabbing exception in pylon camera occurred:" << endl << e.GetDescription() << endl;
			return NULL;
		}
		break;
	case DART:
		try {
			dart_cam_->ExecuteSoftwareTrigger();
			dart_cam_->RetrieveResult(2000, ptr_grab_result_, TimeoutHandling_ThrowException);

			if (ptr_grab_result_->GrabSucceeded()) {
				const uint8_t *pImageBuffer = (uint8_t *) ptr_grab_result_->GetBuffer();
				return pImageBuffer;
			} else {
				cout << "Error: " << ptr_grab_result_->GetErrorCode() << " " << ptr_grab_result_->GetErrorDescription() << endl;
			}
		} catch (GenICam::GenericException &e) {
			cerr << "An image grabbing exception in pylon camera occurred:" << endl << e.GetDescription() << endl;
			return NULL;
		}
		break;
	default:
		cerr << "Unknown Camera Type" << endl;
		break;
	}
	return NULL;
}

} /* namespace pylon_camera */
