/*
 * pylon_interface.h
 *
 *  Created on: May 21, 2015
 *      Author: md
 */

#ifndef PYLON_INTERFACE_H_
#define PYLON_INTERFACE_H_
#include <iostream>
#include <pylon/PylonIncludes.h>
#include <GenApi/IEnumEntry.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>

#include <pylon_camera/pylon_camera_parameter.h>

using std::cout;
using std::cerr;
using std::endl;
using namespace Pylon;

namespace pylon_camera {

enum PYLON_CAM_TYPE {
	GIGE = 1,
	USB = 2,
	DART = 3,
};

class PylonInterface {
public:
	PylonInterface();
	virtual ~PylonInterface();

	int initialize(const PylonCameraParameter &params);
	virtual int setupCameraConfiguration(const PylonCameraParameter &params) = 0;
	virtual const uint8_t* grab(const PylonCameraParameter &params) = 0;

    // Con-/ De-structor automagically calls PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm auto_init_term_;

    CInstantCamera* c_instant_cam_;

    virtual int img_rows();
    virtual int img_cols();
    virtual std::string img_encoding();
    virtual int img_pixel_depth();
    virtual float max_possible_framerate();
protected:
    int img_rows_, img_cols_;
    Basler_UsbCameraParams::PixelFormatEnums img_encoding_;
    Basler_UsbCameraParams::PixelSizeEnums img_pixel_depth_;
    float max_framerate_;

	PYLON_CAM_TYPE cam_type_;
	std::string pylonCamTypeToString(const PYLON_CAM_TYPE type);
	int	 findDesiredCam(const PylonCameraParameter &params);
	PYLON_CAM_TYPE detectPylonCamType(const CInstantCamera* cam);

	// This smart pointer will receive the grab result data.
	CGrabResultPtr ptr_grab_result_;

	CBaslerGigEInstantCamera* gige_cam_;
	CBaslerUsbInstantCamera* usb_cam_;
	CBaslerUsbInstantCamera* dart_cam_;
};

} /* namespace pylon_camera */

#endif /* PYLON_INTERFACE_H_ */
