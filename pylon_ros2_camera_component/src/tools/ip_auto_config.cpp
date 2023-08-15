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

#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/GigETransportLayer.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/BaslerGigECamera.h>

#include <unistd.h>
#include <fstream>
#include <arpa/inet.h>
#include <ifaddrs.h>

using namespace Pylon;
using namespace GenApi;
using namespace Basler_GigECameraParams;
using namespace Basler_GigEStreamParams;

typedef Pylon::CBaslerGigECamera camera_t;
std::vector<std::string> logs;

std::string autoProbe(CBaslerGigEDeviceInfo &bdi, IGigETransportLayer* s_pTl);
void enumurateNIC(std::vector<std::tuple <std::string, std::string>> *InterfaceList);
void enumurateNIC();
void displayCurrentStatus(DeviceInfoList_t &listDevices, IGigETransportLayer* s_pTl);
int readKB(int);
bool checkIPFormat(char* IP);
bool checkSubNetFormat(char* SubnetMask);
void writeLogToFile();


/*
 * 
 */
int main(int argc, char** argv)
{
    // The exit code of the sample application.
    int exitCode = 0;
    
    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonInitialize();

    //getting all IP addresses of nics except loopback

    try
    {
        PylonInitialize();
        int exitCode = 0;
        int selection = -1;
        DeviceInfoList_t listDevices;
        DeviceInfoList_t listReachableDevices;
        
        CTlFactory & theFactory(CTlFactory::GetInstance());
        ITransportLayer * const pTemp(theFactory.CreateTl(CBaslerGigECamera::DeviceClass()));
        IGigETransportLayer* s_pTl = dynamic_cast<IGigETransportLayer*> (pTemp);
        //IGigETransportLayer *pTl = (IGigETransportLayer*)theFactory.CreateTl(BaslerGigEDeviceClass);

        do
        {
            if (!s_pTl)
            {
                logs.push_back("Failed to create transport layer object");
                std::cerr << "Failed to create transport layer object";
            }

            s_pTl->EnumerateAllDevices(listDevices); // Finding all available Basler GigE cameras.
            s_pTl->EnumerateDevices(listReachableDevices);
            
            if(listDevices.size() != listReachableDevices.size() )
            {
                // there are some cameras with not matching IP addres.
                
                for(size_t x = 0 ; x < listDevices.size() ; ++x)
                {
                    std::cout << x;
                    std::string InterfaceAdd = "";
                    CBaslerGigEDeviceInfo &info = static_cast<CBaslerGigEDeviceInfo&> (listDevices[x]);
                    InterfaceAdd = info.GetInterface().c_str();
                    if(InterfaceAdd  == "255.255.255.255")
                    {
                        // the camera where IP is not set properly will report worng interface address.
                        std::string InterfaceAdd = autoProbe(info, s_pTl);
                        std::cout << "\n The unconfigured camera " << info.GetSerialNumber() << " is connected at the NIC " << InterfaceAdd << std::endl;
                    }
                }
            }
           
            if (listDevices.size() > 0)
            {
                std::stringstream ss ;
                ss << listDevices.size();
                logs.push_back("Found cameras " + ss.str());
                enumurateNIC();
                //showing all cameras
                displayCurrentStatus(listDevices, s_pTl);

                std::cerr << std::endl << std::endl;
                std::cerr << "Please select the camera to be configured (type in the camera number or 0 for exit): " << std::endl;
                selection = readKB(listDevices.size());
                if (selection > 0)
                {
                    char IP[256];
                    char Subnet[256];
                    bool bFormat = false;
                    std::cout << "Please, enter the target IP for your selected camera: " << std::endl;
                    while (!bFormat) // Getting the IP form user input
                    {
                        std::cin.getline(IP, 256);
                        std::cin.getline(IP, 256);

                        // this function does not check on IP Format!!!!!
                        bFormat = checkIPFormat(IP);
                        if (!bFormat)
                            std::cout << "The entered IP format or range is not correct, please re-enter the correct IP: " << std::endl;
                    }

                    bFormat = false;
                    std::cout << "Please, enter the Subnet mask for your selected camera: " << std::endl;
                    while (!bFormat) // Getting the Subnet form user input
                    {
                        std::cin.getline(Subnet, 256);
                        bFormat = checkSubNetFormat(Subnet);
                        if (!bFormat)
                            std::cout << "The entered Subnet format or range is not correct, please re-enter the correct Subnet: " << std::endl;
                    }
                    std::cerr << "Your camera will get following setting" << std::endl;
                    std::cerr << "IP " << IP << " subnet : " << Subnet << std::endl;

                    CBaslerGigEDeviceInfo &bdi = static_cast<CBaslerGigEDeviceInfo&> (listDevices[selection - 1]);
                    s_pTl->ForceIp(bdi.GetMacAddress(), IP, Subnet, "0.0.0.0");
                    std::cerr << "Thread will sleep for 1s in order to wait on updated ARP table" << std::endl;
                    sleep(2); // needed to get ARP Table be updated;
                    CBaslerGigEDeviceInfo bdi_new(s_pTl->CreateDeviceInfo());
                    bdi_new.SetIpAddress(IP);

                    s_pTl->EnumerateAllDevices(listDevices);

                    enumurateNIC();
                    //showing all cameras
                    displayCurrentStatus(listDevices, s_pTl);

                    sleep(1);

                    // EDeviceAccessiblityInfo isAccessable;
                    try 
                    {
                        // 1 meaning the camera is reachable after assigning a temporary ip1
                        if (s_pTl->IsDeviceAccessible(bdi_new, Control))
                        { 
                            camera_t camera(s_pTl->CreateDevice(bdi_new));
                            camera.Open();
                            camera.ChangeIpConfiguration(true, true);
                            camera.SetPersistentIpAddress(IP, Subnet, "0.0.0.0");
                            camera.Close();
                            s_pTl->RestartIpConfiguration(bdi.GetMacAddress());
                            std::cerr << "The IP has been assigned to " << bdi.GetSerialNumber() << "successfully" << std::endl;

                            std::cerr << "Thread will sleep for 2ms in order to wait on updated ARP table" << std::endl;
                            sleep(2); // needed to get ARP Table be updated;
                            std::cerr << "Do you want assign IP for another camera or display the current status?" << std::endl;
                            std::cerr << "1= yes or 0 = exit" << std::endl;
                        }
                    } 
                    catch (GenICam::GenericException &e)
                    {
                        std::cerr << "Only a temporary IP address has been assigned, because camera is not reachable after assigning your addresses. Try once more: " << std::endl;
                        std::cerr << "1 = repeat or 0 = exit" << std::endl;
                    }

                    selection = readKB(1);
                } // if (selection >0)
            } 
            else
            {
                std::cerr << "No device found, will try once more in 2s..." << std::endl;
                sleep(2);
            }
        }
        while (selection != 0);
    } 
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl << e.GetDescription() << std::endl;
        exitCode = 1;
    }

    writeLogToFile();

    Pylon::PylonTerminate();
    
    // Comment the following two lines to disable waiting on exit.
    std::cerr << std::endl << "Press Enter to exit." << std::endl;
    while (std::cin.get() != '\n');
    
    return exitCode;
}

std::string autoProbe(CBaslerGigEDeviceInfo &bdi, IGigETransportLayer* s_pTl)
{
    String_t OriginalIP =  bdi.GetIpAddress();
    String_t OriginalSubnet = bdi.GetSubnetMask();  
    std::vector <std::tuple <std::string, std::string>> InterfaceList;
    enumurateNIC(&InterfaceList);

    for(auto& el : InterfaceList)
    {
        std::string IPaddr = std::get<0>(el);
        std::string Subnet = std::get<1>(el);
        in_addr_t TemIP = inet_addr(IPaddr.c_str());
        TemIP = ntohl(TemIP);
        TemIP += 1;
        TemIP = ntohl(TemIP);
        struct in_addr address_struct;
        address_struct.s_addr = TemIP;
        std::string camIP = inet_ntoa(address_struct);
        s_pTl->ForceIp(bdi.GetMacAddress(), camIP.c_str(), Subnet.c_str(), "0.0.0.0");
        DeviceInfoList_t devicelist;
        s_pTl->EnumerateDevices(devicelist);
        for(auto& di : devicelist)
        {
            CBaslerGigEDeviceInfo &GigEdi = static_cast<CBaslerGigEDeviceInfo&>(di) ;
            if(GigEdi.GetMacAddress() == bdi.GetMacAddress())
            {
                s_pTl->ForceIp(bdi.GetMacAddress(),OriginalIP,OriginalSubnet, "0.0.0.0");
                return IPaddr;
            }
        }
    }

    return "";
}

void enumurateNIC(std::vector<std::tuple <std::string, std::string>> *InterfaceList) 
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *addr;
    getifaddrs(&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next) 
    {
        if (ifa->ifa_addr->sa_family == AF_INET) 
        {
            sa = (struct sockaddr_in *) ifa->ifa_addr;
            addr = inet_ntoa(sa->sin_addr);
            std::string s = addr;
            sa= (struct sockaddr_in *) ifa->ifa_netmask;
            addr = inet_ntoa(sa->sin_addr);
            std::string subnet = addr;
            if (s != "127.0.0.1") 
            {
                InterfaceList->push_back(std::tuple<std::string, std::string>(s,subnet));
                logs.push_back(addr);
            }
        }
    }
    freeifaddrs(ifap);
}

void enumurateNIC() 
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *addr;
    getifaddrs(&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next) 
    {
        if (ifa->ifa_addr->sa_family == AF_INET) 
        {
            sa = (struct sockaddr_in *) ifa->ifa_addr;
            addr = inet_ntoa(sa->sin_addr);
            std::string s = addr;
            if (s != "127.0.0.1")
            {
                std::stringstream ss;
                ss << "Interfaces " <<" " << ifa->ifa_name  << " "<< addr;
                printf("Interface: %s\tAddress: %s\n", ifa->ifa_name, addr);
                logs.push_back(ss.str());
            }     
            std::cerr << std::endl;
        }
    }
    freeifaddrs(ifap);
}

void displayCurrentStatus(DeviceInfoList_t &listDevices, IGigETransportLayer* s_pTl) 
{
    if (listDevices.size() > 0) 
    {
        for (uint x = 0; x < listDevices.size(); x++) 
        {
            std::cerr << "Camera No :      SN:            Current IP       Interface IP       Status " << std::endl << std::endl;
            Pylon::EDeviceAccessiblityInfo isAccessable;
            //camera_t camera=s_pTl->CreateDevice(listDevices[x]);
            s_pTl->IsDeviceAccessible(listDevices[x], Control, &isAccessable);
            CBaslerGigEDeviceInfo &bdi = static_cast<CBaslerGigEDeviceInfo&> (listDevices[x]);

            std::string status = "";
            
            switch (isAccessable) 
            {
                case Accessibility_NotReachable:
                    status = "NotReachable";
                    break;
                case Accessibility_Ok:
                    status = "OK";
                    break;
                case Accessibility_Opened:
                    status = "Opened";
                    break;
                case Accessibility_OpenedExclusively:
                    status = "OpenedExclusively";
                    break;
                default:
                    break;
            }
            std::cerr << "    " << x + 1 << "         " << bdi.GetSerialNumber() << "         " << bdi.GetIpAddress() << "     " << bdi.GetInterface() << "       " << status << std::endl;
            std::cerr << std::endl;
        }
    }
}

int readKB(int iNumberofCamera) 
{
    int x = -1;
    std::cerr << "Type in a number between 0 and " << iNumberofCamera << "  " << std::endl;
    do
    {
        x = getchar();
        if (x >= '0' || x <= '9') 
        {
            x = x - '0'; // c will have the value of the digit in the range 0-9
            if (x > iNumberofCamera)
                std::cerr << "Type in a number between 0 and " << iNumberofCamera << "  " << std::endl;
        }
    } 
    while (x < 0 || x > iNumberofCamera);

    return x;
}

bool checkIPFormat(char* _IP) 
{
    std::string strTempIP = _IP;
    std::istringstream ss(strTempIP);
    std::string token;
    std::vector<std::string> result;
    if (strTempIP.length() > 10 && strTempIP.length() <= 15) 
    {
        while(getline(ss, token, '.'))
        {
            result.push_back(token);
        }
        if (result.size() != 4)
            return false;
        else 
        {
            for (size_t i = 0; i < result.size(); i++)
            {
                int number;
                std::istringstream iss (result[i]);
                iss >> number;
                if (iss.fail())
                    return false;
                if  (!((number >= 0 ) && (number <= 255 )))
                    return false;
            }
            return true;
        }
    }
    else
        return false;
}

bool checkSubNetFormat(char* _SubNet) 
{
    std::string strTempSubNet = _SubNet;
    std::istringstream ss(strTempSubNet);
    std::string token;
    std::vector<std::string> result;
    if (strTempSubNet.length() > 10 && strTempSubNet.length() <= 15) 
    {
        while(getline(ss, token, '.'))
        {
            result.push_back(token);
        }
        if (result.size() != 4)
            return false;
        else 
        {
            for (size_t i = 0; i < result.size(); i++)
                {
                    int number;
                    std::istringstream iss (result[i]);
                    iss >> number;
                    if (iss.fail())
                        return false;
                    if  (!((number >= 0 ) && (number <= 255 )))
                        return false;
                }
            return true;
        }
    } 
    else
        return false;
}

void writeLogToFile()
{
    if (logs.size() == 0) 
    {
        logs.push_back("No lines to be written");
    }
    std::ofstream ofs("ip_auto_config_output.txt");
    std::cout << "Log has " << logs.size()  << " Lines " << std::endl;
     
    for(std::vector<std::string>::const_iterator i = logs.begin(); i != logs.end(); ++i) 
    {
        ofs << *i << '\n';
    }
}
