/*
 * pylon_sequencer_interface.h
 *
 *  Created on: May 26, 2015
 *      Author: md
 */

#ifndef PYLON_SEQUENCER_INTERFACE_H_
#define PYLON_SEQUENCER_INTERFACE_H_

#include <pylon_camera/pylon_interface.h>

namespace pylon_camera {

class PylonSequencerInterface: public PylonInterface {
public:
	PylonSequencerInterface();
	virtual ~PylonSequencerInterface();

	int initialize(const PylonCameraParameter &params);

	virtual int initSequencer(const PylonCameraParameter &params);

	std::vector<cv::Mat> img_sequence_;
};

} /* namespace pylon_camera */

#endif /* PYLON_SEQUENCER_INTERFACE_H_ */
