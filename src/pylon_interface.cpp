/*
 * pylon_interface.cpp
 *
 *  Created on: May 21, 2015
 *      Author: md
 */

#include <pylon_camera/pylon_interface.h>

namespace pylon_camera {

PylonInterface::PylonInterface():
		img_rows_(-1),
		img_cols_(-1),
		max_framerate_(-1.0),
		gige_cam_(NULL),
		usb_cam_(NULL),
		dart_cam_(NULL) {
	// TODO Auto-generated constructor stub
}

PylonInterface::~PylonInterface() {
	// TODO Auto-generated destructor stub
}
int PylonInterface::img_rows(){
	return this->img_rows_;
}
int PylonInterface::img_cols(){
	return this->img_cols_;
}
float PylonInterface::max_possible_framerate(){
	return this->max_framerate_;
}
std::string PylonInterface::img_encoding(){
	switch (img_encoding_) {
		case Basler_UsbCameraParams::PixelFormat_Mono8:
			return "mono8";
			break;
		default:
			cerr << "Image encoding " << img_encoding_ << " not yet implemented!" << endl;
			return "";
			break;
	}
	return "";
	// List from #include <sensor_msgs/image_encodings.h>
	//    const std::string RGB8 = "rgb8";
	//    const std::string RGBA8 = "rgba8";
	//    const std::string RGB16 = "rgb16";
	//    const std::string RGBA16 = "rgba16";
	//    const std::string BGR8 = "bgr8";
	//    const std::string BGRA8 = "bgra8";
	//    const std::string BGR16 = "bgr16";
	//    const std::string BGRA16 = "bgra16";
	//    const std::string MONO8="mono8";
	//    const std::string MONO16="mono16";
}
int PylonInterface::img_pixel_depth(){
	switch (img_pixel_depth_) {
		case Basler_UsbCameraParams::PixelSize_Bpp8:
			return sizeof(uint8_t);
			break;
//		case Basler_UsbCameraParams::PixelSize_Bpp12:
//			return 12;
//			break;
//		case Basler_UsbCameraParams::PixelSize_Bpp16:
//			return 16;
//			break;
//		case Basler_UsbCameraParams::PixelSize_Bpp24:
//			return 24;
//			break;
		default:
			cerr << "Image Pixel Depth not yet implemented!" << endl;
			return -1;
			break;
	}
	return -1;
}
std::string PylonInterface::pylonCamTypeToString(const PYLON_CAM_TYPE type){
	switch (type) {
	case GIGE:
		return "GigE";
	case USB:
		return "USB";
	case DART:
		return "Dart";
	}
	return "Unknown Camera Type";
}
int PylonInterface::findDesiredCam(const PylonCameraParameter &params) {

	if (params.magazino_cam_id_ == "x") { // Select camera device found first
		try {
			CInstantCamera* cam = new CInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
			cam->Open();
			cam_type_ = detectPylonCamType(cam);
			cout << "Desired Cam is a " << pylonCamTypeToString(cam_type_) << " camera." << endl;
			cam->Close();
			switch (cam_type_) {
			case GIGE:
				gige_cam_ = new Pylon::CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
				break;
			case USB:
				usb_cam_ = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
				break;
			case DART:
				dart_cam_ = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
				break;
			default:
				cerr << "Unknown Camera Type" << endl;
				break;
			}

		} catch (GenICam::GenericException &e) {
			cerr << "An exception while selecting the first pylon camera found occurred:" << endl;
			cerr << e.GetDescription() << endl;
			return 1;
		}
	} else { // Select camera device with specified magazino_cam_id
		try {
			// Get the transport layer factory.
			CTlFactory& transport_layer_factory = CTlFactory::GetInstance();

			// Get all attached devices and exit application if no device is found.
			DeviceInfoList_t device_info_list;
			if (transport_layer_factory.EnumerateDevices(device_info_list) == 0) {
				throw RUNTIME_EXCEPTION( "No camera present.");
				return 2;
			}

			// Create an array of instant cameras for the found devices
			CInstantCameraArray camera_array(device_info_list.size());

			bool found_desired_device = false;

			// Create and attach all Pylon Devices.
			size_t cam_pos = -1;
			for (size_t i = 0; i < camera_array.GetSize(); ++i) {

				camera_array[i].Attach(transport_layer_factory.CreateDevice(device_info_list[i]));

				camera_array[i].Open();

				GenApi::INodeMap& node_map = camera_array[i].GetNodeMap();
				GenApi::CStringPtr DeviceUserID(node_map.GetNode("DeviceUserID"));

				if (std::string(DeviceUserID->GetValue()) == params.magazino_cam_id_) {
					found_desired_device = true;
					cam_pos = i;
					cout << "Found the desired Camera with Magazino ID: " << params.magazino_cam_id_ << ": "
							<< camera_array[cam_pos].GetDeviceInfo().GetModelName() << endl;
					break;
				}
				camera_array[i].Close();
			}
			if (!found_desired_device) {
				cerr << "Maybe the given magazino_cam_id (" << params.magazino_cam_id_.c_str()
						<< ") is wrong or has not yet been written to the camera using 'write_magazino_id_to_camera' ?!" << endl;
				return 3;
			}
			if (!camera_array[cam_pos].IsOpen()) {
				camera_array[cam_pos].Open();
			}
			cam_type_ = detectPylonCamType(&camera_array[cam_pos]);
			camera_array[cam_pos].Close();

			cout << "CAM_TYPE = " << cam_type_ << endl;
			switch (cam_type_) {
			case GIGE:
				gige_cam_ = new Pylon::CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
				break;
			case USB:
				usb_cam_ = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
				break;
			case DART:
				dart_cam_ = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
				break;
			default:
				cerr << "Unknown Camera Type" << endl;
				break;
			}

		} catch (GenICam::GenericException &e) {
			cerr << "An exception while opening the desired camera with Magazino ID: " << params.magazino_cam_id_ << " occurred:" << endl;
			cerr << e.GetDescription() << endl;
			return 3;
		}
	}

	return 0;
}
PYLON_CAM_TYPE PylonInterface::detectPylonCamType(const CInstantCamera* cam) {
	PYLON_CAM_TYPE cam_type;
	try {
		VersionInfo sfnc_version = cam->GetSfncVersion();

		if (sfnc_version < Sfnc_2_0_0) { // ace GigE
			//CIntegerPtr ExposureTimeRaw (nodemap.GetNode("ExposureTimeRaw"));
			cam_type = GIGE;
		} else if (sfnc_version == Sfnc_2_1_0) { // ace USB
			// CFloatPtr ExposureTime (nodemap.GetNode("ExposureTime"));
			cam_type = USB;
		} else if (sfnc_version == Sfnc_2_2_0) { // Dart Cameras
			// CFloatPtr ExposureTime (nodemap.GetNode("ExposureTime"));
			cam_type = DART;
		}
	} catch (GenICam::GenericException &e) {
		cerr << "An exception while detecting the pylon camera type from its SFNC-Version occurred:" << endl;
		cerr << e.GetDescription() << endl;
	}
	return cam_type;
}
int PylonInterface::initialize(const PylonCameraParameter &params) {
	int exit_code = 0;

	// Find the desired camera device in the list of pluged-in devices
	// Use either magazino_cam_id or the device found first
	if (findDesiredCam(params)) {
		cerr << "Error while searching the desired camera" << endl;
		return 1;
	}
	if (!(cam_type_ == GIGE || cam_type_ == USB || cam_type_ == DART)) {
		cerr << "Error while detecting pylon camera type: Unknown type" << endl;
		return 2;
	}
	return exit_code;
}
} /* namespace pylon_camera */
