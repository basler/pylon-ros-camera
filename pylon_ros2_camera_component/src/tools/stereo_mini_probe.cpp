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

// Read-only diagnostic for the Stereo mini. Requires exclusive access, so the
// ROS node must NOT be running when this tool is used.
//
//   stereo_mini_probe -list          -> list every detected camera (model/uid/serial)
//   stereo_mini_probe                 -> probe first available camera
//   stereo_mini_probe -uid <id>       -> probe camera with the given DeviceUserID
//   stereo_mini_probe -sn <serial>    -> probe camera with the given serial
//
// It reports the projector and depth-preset nodes and whether each is writable
// while grabbing:
//   BslLaserEnable (projector on/off), BslLaserLevel (projector power, min/max),
//   BslDepthPreset (depth tuning preset, available entries).

#include <pylon/PylonIncludes.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace
{

// Minimal command-line parser (same style as the other tools in this folder).
class InputParser
{
public:
    InputParser(int& argc, char** argv)
    {
        for (int i = 1; i < argc; ++i)
        {
            this->tokens.push_back(std::string(argv[i]));
        }
    }

    std::string getCmdOption(const std::string& option) const
    {
        auto itr = std::find(this->tokens.begin(), this->tokens.end(), option);
        if (itr != this->tokens.end() && ++itr != this->tokens.end())
        {
            return *itr;
        }
        return std::string();
    }

    bool cmdOptionExists(const std::string& option) const
    {
        return std::find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
    }

private:
    std::vector<std::string> tokens;
};

// GenApi access mode -> human-readable state, so writability is unambiguous.
std::string accessModeToString(GenApi::EAccessMode mode)
{
    switch (mode)
    {
        case GenApi::NI: return "NotImplemented";
        case GenApi::NA: return "NotAvailable";
        case GenApi::WO: return "WriteOnly";
        case GenApi::RO: return "ReadOnly";
        case GenApi::RW: return "ReadWrite";
        default:         return "Undefined";
    }
}

void probeBool(GenApi::INodeMap& node_map, const char* name)
{
    GenApi::CBooleanPtr node(node_map.GetNode(name));
    if (!node.IsValid())
    {
        std::cout << "  " << name << ": <node absent>" << std::endl;
        return;
    }
    std::cout << "  " << name << " [IBoolean, " << accessModeToString(node->GetAccessMode()) << "]";
    if (GenApi::IsReadable(node))
    {
        std::cout << " value=" << (node->GetValue() ? "true" : "false");
    }
    std::cout << std::endl;
}

void probeInt(GenApi::INodeMap& node_map, const char* name)
{
    GenApi::CIntegerPtr node(node_map.GetNode(name));
    if (!node.IsValid())
    {
        std::cout << "  " << name << ": <node absent>" << std::endl;
        return;
    }
    std::cout << "  " << name << " [IInteger, " << accessModeToString(node->GetAccessMode()) << "]";
    if (GenApi::IsReadable(node))
    {
        std::cout << " value=" << node->GetValue()
                  << " min=" << node->GetMin()
                  << " max=" << node->GetMax();
    }
    std::cout << std::endl;
}

void probeEnum(GenApi::INodeMap& node_map, const char* name)
{
    GenApi::CEnumerationPtr node(node_map.GetNode(name));
    if (!node.IsValid())
    {
        std::cout << "  " << name << ": <node absent>" << std::endl;
        return;
    }
    std::cout << "  " << name << " [IEnumeration, " << accessModeToString(node->GetAccessMode()) << "]";
    if (GenApi::IsReadable(node))
    {
        std::cout << " value='" << node->ToString() << "' entries={";
        GenApi::NodeList_t entries;
        node->GetEntries(entries);
        bool first = true;
        for (auto& e : entries)
        {
            GenApi::CEnumEntryPtr entry(e);
            if (entry.IsValid() && GenApi::IsAvailable(entry))
            {
                if (!first) std::cout << ", ";
                std::cout << entry->GetSymbolic();
                first = false;
            }
        }
        std::cout << "}";
    }
    std::cout << std::endl;
}

// Print only the access mode of a node, and whether it can be written while grabbing.
void printAccess(GenApi::INodeMap& node_map, const char* name)
{
    GenApi::CNodePtr node(node_map.GetNode(name));
    const std::string state = node.IsValid() ? accessModeToString(node->GetAccessMode()) : "absent";
    std::cout << "  " << name << " access=" << state
              << " => " << ((state == "ReadWrite") ? "WRITABLE WHILE GRABBING" : "NOT WRITABLE WHILE GRABBING")
              << std::endl;
}

// Read a device's live DeviceUserID by opening it. Some cameras (e.g. the Stereo
// mini over GenTL) report a blank name at enumeration time and only expose the
// DeviceUserID once the device is open, so enumeration data alone cannot match
// them by id. Returns an empty string if the device cannot be opened or read.
std::string liveDeviceUserId(Pylon::CTlFactory& tl_factory, const Pylon::CDeviceInfo& info)
{
    try
    {
        Pylon::CInstantCamera probe(tl_factory.CreateDevice(info));
        probe.Open();
        GenApi::CStringPtr user_id_node(probe.GetNodeMap().GetNode("DeviceUserID"));
        std::string live_id;
        if (user_id_node.IsValid() && GenApi::IsReadable(user_id_node))
            live_id = std::string(user_id_node->GetValue().c_str());
        probe.Close();
        return live_id;
    }
    catch (const GenICam::GenericException&)
    {
        return std::string();
    }
}

// Match a device against the requested serial and/or user id. The user id is
// checked against the enumeration-time name first; for devices that report a
// blank name there, fall back to the live DeviceUserID, mirroring the driver.
bool deviceMatches(Pylon::CTlFactory& tl_factory, const Pylon::CDeviceInfo& info,
                   const std::string& serial, const std::string& user_id)
{
    if (!serial.empty() && std::string(info.GetSerialNumber().c_str()) == serial)
        return true;
    if (user_id.empty())
        return false;

    const std::string enum_name(info.GetUserDefinedName().c_str());
    if (!enum_name.empty())
        return enum_name == user_id;

    return liveDeviceUserId(tl_factory, info) == user_id;
}

void probeCamera(Pylon::CInstantCamera& cam)
{
    cam.Open();
    GenApi::INodeMap& node_map = cam.GetNodeMap();

    std::cout << "Camera: " << cam.GetDeviceInfo().GetModelName()
              << " (serial " << cam.GetDeviceInfo().GetSerialNumber() << ")" << std::endl;

    std::cout << "\n[Phase2] Projector + depth-preset nodes (camera stopped):" << std::endl;
    probeBool(node_map, "BslLaserEnable");
    probeInt(node_map, "BslLaserLevel");
    probeEnum(node_map, "BslDepthPreset");

    std::cout << "\n[Phase2] Same nodes while grabbing:" << std::endl;
    try
    {
        cam.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
        Pylon::CGrabResultPtr result;
        cam.RetrieveResult(5000, result, Pylon::TimeoutHandling_Return);  // warm up one frame
        printAccess(node_map, "BslLaserEnable");
        printAccess(node_map, "BslLaserLevel");
        printAccess(node_map, "BslDepthPreset");
        cam.StopGrabbing();
    }
    catch (const GenICam::GenericException& e)
    {
        std::cout << "  grabbing test skipped (" << e.GetDescription() << ")" << std::endl;
        if (cam.IsGrabbing()) cam.StopGrabbing();
    }

    cam.Close();
}

}  // namespace

int main(int argc, char* argv[])
{
    InputParser input(argc, argv);
    const std::string user_id = input.getCmdOption("-uid");
    const std::string serial  = input.getCmdOption("-sn");
    const bool list_only = input.cmdOptionExists("-list");

    Pylon::PylonInitialize();

    int rc = 0;
    try
    {
        Pylon::CTlFactory& tl_factory = Pylon::CTlFactory::GetInstance();

        if (list_only)
        {
            Pylon::DeviceInfoList_t devices;
            if (tl_factory.EnumerateDevices(devices) == 0)
            {
                std::cerr << "No cameras detected!" << std::endl;
                Pylon::PylonTerminate();
                return 1;
            }
            for (size_t i = 0; i < devices.size(); ++i)
            {
                // Fall back to the live DeviceUserID when the enumeration name is
                // blank, so cameras like the Stereo mini still show their id.
                std::string uid(devices[i].GetUserDefinedName().c_str());
                if (uid.empty())
                    uid = liveDeviceUserId(tl_factory, devices[i]);
                std::cout << "[" << i << "] model='" << devices[i].GetModelName()
                          << "' uid='" << uid
                          << "' serial='" << devices[i].GetSerialNumber() << "'" << std::endl;
            }
            Pylon::PylonTerminate();
            return 0;
        }

        if (!serial.empty() || !user_id.empty())
        {
            Pylon::DeviceInfoList_t devices;
            if (tl_factory.EnumerateDevices(devices) == 0)
            {
                std::cerr << "No cameras detected!" << std::endl;
                Pylon::PylonTerminate();
                return 1;
            }

            size_t i = 0;
            for (; i < devices.size(); ++i)
            {
                if (deviceMatches(tl_factory, devices[i], serial, user_id))
                {
                    Pylon::CInstantCamera cam(tl_factory.CreateDevice(devices[i]));
                    probeCamera(cam);
                    break;
                }
            }
            if (i == devices.size())
            {
                std::cerr << "Camera not found (serial='" << serial
                          << "', uid='" << user_id << "')." << std::endl;
                rc = 2;
            }
        }
        else
        {
            Pylon::CDeviceInfo dev_info;
            Pylon::CInstantCamera cam(tl_factory.CreateFirstDevice(dev_info));
            probeCamera(cam);
        }
    }
    catch (const GenICam::GenericException& e)
    {
        std::cerr << "An exception occurred." << std::endl << e.GetDescription() << std::endl;
        rc = 3;
    }

    Pylon::PylonTerminate();
    return rc;
}
