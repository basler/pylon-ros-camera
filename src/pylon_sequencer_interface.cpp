/*
 * pylon_sequencer_interface.cpp
 *
 *  Created on: May 26, 2015
 *      Author: md
 */

#include <pylon_camera/pylon_sequencer_interface.h>

namespace pylon_camera
{

PylonSequencerInterface::PylonSequencerInterface() :
        img_sequence_()
{
  // TODO Auto-generated constructor stub
}
PylonSequencerInterface::~PylonSequencerInterface()
{
  // TODO Auto-generated destructor stub
}
int PylonSequencerInterface::initialize(const PylonCameraParameter &params)
{
  if (PylonInterface::initialize(params))
  {
    cerr << "Error while initializing the base Pylon Interface" << endl;
  }

  int exit_code = 0;

  return exit_code;
}
int PylonSequencerInterface::initSequencer(const PylonCameraParameter &params)
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

        GenApi::INodeMap& nodemap = usb_cam_->GetNodeMap();
        GenApi::CEnumEntryPtr SequencerMode(nodemap.GetNode("SequencerMode"));
        SequencerMode->FromString("Off");
        GenApi::CEnumEntryPtr SequencerConfigurationMode(nodemap.GetNode("SequencerConfigurationMode"));
        SequencerConfigurationMode->FromString("ON");
        GenApi::CEnumEntryPtr SequencerSetStart(nodemap.GetNode("SequencerSetStart"));

//			usb_cam_->SequencerConfigurationMode.SetValue(SequencerConfigurationMode_On);
//			 Disable the sequencer feature
//			usb_cam_->SequencerMode.SetValue(SequencerMode_Off);
//			// Enable the sequencer configuration mode
//			usb_cam_->SequencerConfigurationMode.SetValue(SequencerConfigurationMode_On);
//			// Select the first sequencer set (always sequencer set 0)
//			usb_cam_->SequencerSetStart.SetValue(0);
//			// Select a sequencer set by its index number
//			usb_cam_->SequencerSetSelector.SetValue(0);
//			// Select path 0 for the selected sequencer set
//			usb_cam_->SequencerPathSelector.SetValue(0);
//			// Select the sequencer set that will be applied after the current sequencer set
//			usb_cam_->SequencerSetNext.SetValue(0);
//			// Select the trigger source for sequencer set advance
//			usb_cam_->SequencerTriggerSource.SetValue(SequencerTriggerSource_Line_3);
//			// Select the logic for the sequencer set advance trigger source for path 0(always LevelHigh)
//			usb_cam_->SequencerTriggerActivation.SetValue(SequencerTriggerActivation_LevelHigh);
//			// Select path 1 for the selected sequencer set
//			usb_cam_->SequencerPathSelector.SetValue(1);
//			// Select the sequencer set that will be applied after the current sequencer set
//			usb_cam_->SequencerSetNext.SetValue(1);
//			// Select the trigger source for sequencer set advance
//			usb_cam_->SequencerTriggerSource.SetValue(SequencerTriggerSource_Line_4);
//			// Select the logic for the sequencer set advance trigger source for path 1(always LevelHigh)
//			usb_cam_->SequencerTriggerActivation.SetValue(SequencerTriggerActivation_LevelHigh);
//			// Save the camera parameter values and the sequencer set-related parameter values
//			// for the selected sequencer set
//			usb_cam_->SequencerSetSave.Execute( );

//			gige_cam_->
//		    if ( GenApi::IsWritable(usb_cam_->Seq)){

//		    }
//
//		        // Disable the sequencer before changing parameters.
//		        // The parameters under control of the sequencer are locked
//		        // when the sequencer is enabled. For a list of parameters
//		        // controlled by the sequencer, see the camera User's Manual.
//		        camera.SequenceEnable.SetValue(false);
//		        // Maximize the image area of interest (Image AOI).
//		        if (IsWritable(camera.OffsetX))
//		        {
//		            camera.OffsetX.SetValue(camera.OffsetX.GetMin());
//		        }
//		        if (IsWritable(camera.OffsetY))
//		        {
//		            camera.OffsetY.SetValue(camera.OffsetY.GetMin());
//		        }
//		        camera.Width.SetValue(camera.Width.GetMax());
//		        camera.Height.SetValue(camera.Height.GetMax());
//		        // Set the pixel data format.
//		        camera.PixelFormat.SetValue(PixelFormat_Mono8);
//		        // Set up sequence sets.
//		        // Configure how the sequence will advance.
//		        // 'Auto' refers to the auto sequence advance mode.
//		        // The advance from one sequence set to the next will occur automatically with each image acquired.
//		        // After the end of the sequence set cycle was reached a new sequence set cycle will start.
//		        camera.SequenceAdvanceMode = SequenceAdvanceMode_Auto;
//		        // Our sequence sets relate to three steps (0..2).
//		        // In each step we will increase the height of the Image AOI by one increment.
//		        camera.SequenceSetTotalNumber = 3;
//		        int64_t increments = (camera.Height.GetMax() - camera.Height.GetMin()) / camera.Height.GetInc();
//		        // Set the parameters for step 0; quarter height image.
//		        camera.SequenceSetIndex = 0;
//		        camera.Height.SetValue( camera.Height.GetInc() * (increments / 4));
//		        camera.SequenceSetStore.Execute();
//		        // Set the parameters for step 1; half height image.
//		        camera.SequenceSetIndex = 1;
//		        camera.Height.SetValue( camera.Height.GetInc() * (increments / 2));
//		        camera.SequenceSetStore.Execute();
//		        // Set the parameters for step 2; full height image.
//		        camera.SequenceSetIndex = 2;
//		        camera.Height.SetValue( camera.Height.GetInc() * increments);
//		        camera.SequenceSetStore.Execute();
//		        // Enable the sequencer feature.
//		        // From here on you cannot change the sequencer settings anymore.
//		        camera.SequenceEnable.SetValue(true);
//		    } else {
//		    	cerr << "Sequencer function not available for this cam!" << endl;
//		    }
      }
      catch (GenICam::GenericException &e)
      {
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
} /* namespace pylon_camera */
