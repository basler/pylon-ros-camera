/*
 * pylon_hdr_interface.cpp
 *
 *  Created on: May 26, 2015
 *      Author: md
 */

#include <pylon_camera/pylon_hdr_interface.h>

namespace pylon_camera {

PylonHDRInterface::PylonHDRInterface() {
	// TODO Auto-generated constructor stub

}

PylonHDRInterface::~PylonHDRInterface() {
	// TODO Auto-generated destructor stub
}
int PylonHDRInterface::initialize(const PylonCameraParameter &params) {
	int exit_code = 0;
	return exit_code;
}
const uint8_t* grab(const PylonCameraParameter &params){
//int PylonHDRInterface::grab(uint8_t *img_ptr, const PylonCameraParameter &params) {
	int exit_code = 0;
	return NULL;
}
} /* namespace pylon_camera */
