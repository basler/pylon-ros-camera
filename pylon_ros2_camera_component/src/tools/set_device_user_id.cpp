/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2022, Basler AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * No contributors' name may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

// no arguments -> set user id to the first connected camera
// -sn with serial -> set user id of the camera with the specified serial

#include <pylon/PylonIncludes.h>

#include <algorithm>
#include <unistd.h>
#include <string>

// Argument parser
// from https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
// @author iain
class InputParser
{
public:
    InputParser (int &argc, char **argv)
    {
        for (int i=1; i < argc; ++i)
        {
            this->tokens.push_back(std::string(argv[i]));
        }
    }

    const std::string& getCmdOption(const std::string &option) const
    {
        std::vector<std::string>::const_iterator itr;
        itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
        if (itr != this->tokens.end() && ++itr != this->tokens.end())
        {
            return *itr;
        }
        static const std::string empty_string("");
        return empty_string;
    }

    bool cmdOptionExists(const std::string &option) const
    {
        return std::find(this->tokens.begin(), this->tokens.end(), option)
                != this->tokens.end();
    }

private:
    std::vector <std::string> tokens;
};

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

// main
int main(int argc, char* argv[])
{
    bool sn_arg_is_on = false;
    if(cmdOptionExists(argv, argv+argc, "-sn"))
    {
        sn_arg_is_on = true;
    }

    if ((!sn_arg_is_on && (argc < 2)) || (sn_arg_is_on && (argc < 4)))
    {
        std::cerr << "ERROR: Not enough parameters!" << std::endl;
        std::cout << "USAGE:" << std::endl;
        std::cout << "   set_user_id_to_camera DEVICE_USER_ID           " << "\t-> Assign user id to first available camera" << std::endl;
        std::cout << "   set_user_id_to_camera -sn SERIAL DEVICE_USER_ID" << "\t-> Assign user id to camera specified by serial" << std::endl;
        std::cout << "TIPS: run IPAutoConfig to get the serials of the available gigE cameras and \"lsusb -v | grep Serial\" to get the serials of the available USB cameras" << std::endl;
        return 1;
    }

    int arg_serial_index = -1;
    int arg_device_id_index = 1;
    if (sn_arg_is_on)
    {
        arg_serial_index = 2;
        arg_device_id_index = 3;
    }

    std::cout << "User input(s):" << std::endl;
    if (sn_arg_is_on)
        std::cout << "  Serial: " << argv[arg_serial_index] << std::endl;
    std::cout << "  Device ID: " << argv[arg_device_id_index] << std::endl;
    std::string device_user_id(reinterpret_cast<char *>(argv[arg_device_id_index]));
    if (device_user_id.empty())
    {
        std::cout << "ERROR:" << std::endl;
        std::cout << "The specified device_user_id is empty!" << std::endl;
        return 2;
    }

    std::string serial = "";
    if (sn_arg_is_on)
    {
        serial = std::string(reinterpret_cast<char *>(argv[arg_serial_index]));
        if (serial.empty())
        {
            std::cout << "ERROR:" << std::endl;
            std::cout << "The specified serial is empty!" << std::endl;
            return 2;
        }
    }

    Pylon::PylonInitialize();

    try
    {
        if (!sn_arg_is_on)
        {
            // if no serial is specified, consider first available camera

            Pylon::CDeviceInfo dev_info;
            // Create an instant camera object with the first available camera.
            Pylon::CInstantCamera dev(Pylon::CTlFactory::GetInstance().CreateFirstDevice(dev_info));

            dev.Open();
            GenApi::INodeMap& node_map = dev.GetNodeMap();
            GenApi::CStringPtr current_device_user_id(node_map.GetNode("DeviceUserID"));
            current_device_user_id->SetValue(Pylon::String_t(device_user_id.c_str()));

            std::cout << "Successfully wrote " << current_device_user_id->GetValue() << " to the camera "
                    << dev.GetDeviceInfo().GetModelName() << std::endl;
            std::cout << "  Don't forget to disconnect/reconnect the camera if it is a USB one" << std::endl;
            dev.Close();
        }
        else
        {
            // enumerate the cameras and look for the one with the specified serial

            Pylon::CTlFactory &tl_factory = Pylon::CTlFactory::GetInstance();
            Pylon::DeviceInfoList_t devices;
            if (tl_factory.EnumerateDevices(devices) == 0)
            {
                std::cout << "No cameras detected!" << std::endl;
                return false;
            }

            size_t i = 0;
            for (i; i < devices.size(); i++)
            {
                Pylon::CDeviceInfo &dev_info = devices[i];
                std::string serial_number(dev_info.GetSerialNumber().c_str());

                if (serial_number == serial)
                {
                    Pylon::CInstantCamera dev(tl_factory.CreateDevice(dev_info));

                    dev.Open();
                    GenApi::INodeMap& node_map = dev.GetNodeMap();
                    GenApi::CStringPtr current_device_user_id(node_map.GetNode("DeviceUserID"));
                    current_device_user_id->SetValue(Pylon::String_t(device_user_id.c_str()));

                    std::cout << "Successfully wrote " << current_device_user_id->GetValue() << " to the camera "
                        << dev.GetDeviceInfo().GetModelName() << std::endl;
                    std::cout << "  Don't forget to disconnect/reconnect the camera if it is a USB one" << std::endl;
                    dev.Close();

                    break;
                }
            }

            if (i == devices.size())
            {
                std::cout << "Camera with serial " << serial << " not found!" << std::endl
                        << "Check the serial by running IPAutoConfig for gigE cameras and \"lsusb -v | grep Serial\" for USB cameras" << std::endl;
            }
        }
    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred."
                  << std::endl
                  << e.GetDescription()
                  << std::endl;
        return 3;
    }

    // Releases all pylon resources.
    Pylon::PylonTerminate();

    return 0;
}
