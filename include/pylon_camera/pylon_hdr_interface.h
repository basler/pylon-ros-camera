/*
 * pylon_hdr_interface.h
 *
 *  Created on: May 26, 2015
 *      Author: md
 */

#ifndef PYLON_HDR_INTERFACE_H_
#define PYLON_HDR_INTERFACE_H_

#include <pylon_camera/pylon_interface.h>

namespace pylon_camera {

class PylonHDRInterface: public PylonInterface {
public:
	PylonHDRInterface();
	virtual ~PylonHDRInterface();

	int initialize(const PylonCameraParameter &params);
//	int grab(uint8_t *img_ptr, const PylonCameraParameter &params);
	const uint8_t* grab(const PylonCameraParameter &params);
};

} /* namespace pylon_camera */

#endif /* PYLON_HDR_INTERFACE_H_ */
