/* 
 * File:   main.cpp
 * Author: S.Mathi
 *
 * Created on 26. Juni 2018, 11:51
 */

#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/GigETransportLayer.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/BaslerGigECamera.h>

#include <cstdlib>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <list>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iterator>
#include<arpa/inet.h>
#include<sys/socket.h>
#include<ifaddrs.h>

using namespace Pylon;
using namespace GenApi;
using namespace Basler_GigECameraParams;
using namespace Basler_GigEStreamParams;
using namespace std;

typedef Pylon::CBaslerGigECamera Camera_t;
vector<string> Log;

int readKB(int);
void DisplayCurrentStatus(DeviceInfoList_t &lstDevices, IGigETransportLayer* s_pTl);
bool checkIPFormat(char* IP);
bool checkSubNetFormat(char* SubnetMask);
void enumurateNIC(vector<tuple <string,string>> *InterfaceList);
void enumurateNIC();

string  AutoProbe(CBaslerGigEDeviceInfo &bdi, IGigETransportLayer* s_pTl);

void WriteToFile() {


    if (Log.size() == 0) {
        Log.push_back("No lines to be written");
    }

    ofstream ofs("appoutput.txt");
    cout << "log has " << Log.size()  << " Lines " <<endl;
     
    for(vector<string>::const_iterator i = Log.begin(); i != Log.end(); ++i) 
    {
         ofs << *i << '\n';
    }
    
}

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
    // The exit code of the sample application.
    int exitCode = 0;
    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonInitialize();

    //getting all IP addresses of nics except loopback

    try {
        PylonInitialize();
        int exitCode = 0;
        int selction = -1;
        DeviceInfoList_t lstDevices;
        DeviceInfoList_t lstReachableDevices;
       
        
        CTlFactory & theFactory(CTlFactory::GetInstance());
        ITransportLayer * const pTemp(theFactory.CreateTl(CBaslerGigECamera::DeviceClass()));
        IGigETransportLayer* s_pTl = dynamic_cast<IGigETransportLayer*> (pTemp);
        //IGigETransportLayer *pTl = (IGigETransportLayer*)theFactory.CreateTl(BaslerGigEDeviceClass);

        do {
            if (!s_pTl) {
                Log.push_back("Failed to create transport layer object");
                cerr << "Failed to create transport layer object";

            }


            s_pTl->EnumerateAllDevices(lstDevices); // Finding all available Basler GigE cameras.
            s_pTl->EnumerateDevices(lstReachableDevices);
            
            if(lstDevices.size() != lstReachableDevices.size() )
            {
                // there are some cameras with not matching IP addres.
                
                for(int x = 0 ; x < lstDevices.size() ; ++x)
                {
                    cout << x;
                    string InterfaceAdd = "";
                    CBaslerGigEDeviceInfo &info = static_cast<CBaslerGigEDeviceInfo&> (lstDevices[x]);
                    InterfaceAdd = info.GetInterface().c_str();
                    if(InterfaceAdd  == "255.255.255.255")
                    {
                        // the camera where IP is not set properly will report worng interface address.
                        string InterfaceAdd = AutoProbe(info, s_pTl);
                        
                        cout << "\n The unconfigured camera " << info.GetSerialNumber() << " is connected at the NIC " << InterfaceAdd <<endl;
                       
                    }
                            
                    
                }
                
            }
           
            
            if (lstDevices.size() > 0) {
                  stringstream ss ;
                  ss << lstDevices.size();
                Log.push_back("Found cameras " + ss.str());
                enumurateNIC();
                //showing all  cameras
                DisplayCurrentStatus(lstDevices, s_pTl);

                cerr << endl << endl;
                cerr << "Please select the camera to be configured: \n type in the camera number or 0 for exit : " << endl;
                selction = readKB(lstDevices.size());
                if (selction > 0) {

                    char IP[256];
                    char Subnet[256];
                    bool bFormat = false;
                    cout << "Please, enter the target IP for your selected camera : " << endl;
                    while (!bFormat) // Getting the IP form user input
                    {

                        cin.getline(IP, 256);
                        cin.getline(IP, 256);

                        // this function does not check on IP Format!!!!!
                        bFormat = checkIPFormat(IP);
                        if (!bFormat)
                            cout << "The entered IP format or range is not correct, please re-enter the correct IP " << endl;

                    }
                    bFormat = false;
                    cout << "Please, enter the Subnet mask for your selected camera : " << endl;
                    while (!bFormat) // Getting the Subnet form user input
                    {
                        
                        cin.getline(Subnet, 256);
                        bFormat = checkSubNetFormat(Subnet);
                        if (!bFormat)
                            cout << "The entered Subnet format or range is not correct, please re-enter the correct Subnet " << endl;
                    }
                    cerr << "your camera will get following setting" << endl;
                    cerr << "IP " << IP << " subnet : " << Subnet << endl;

                    CBaslerGigEDeviceInfo &bdi = static_cast<CBaslerGigEDeviceInfo&> (lstDevices[selction - 1]);
                    s_pTl->ForceIp(bdi.GetMacAddress(), IP, Subnet, "0.0.0.0");
                    cerr << "Thread will sleep for 1s in order to wait on updated ARP table" << endl;
                    sleep(2); // needed to get ARP Table be updated;
                    CBaslerGigEDeviceInfo bdi_new(s_pTl->CreateDeviceInfo());
                    bdi_new.SetIpAddress(IP);

                    s_pTl->EnumerateAllDevices(lstDevices);

                    enumurateNIC();
                    //showing all  cameras
                    DisplayCurrentStatus(lstDevices, s_pTl);

                    sleep(1);

                    // EDeviceAccessiblityInfo isAccessable;
                    try {
                        if (s_pTl->IsDeviceAccessible(bdi_new, Control)) { // 1 meaning the camera is reachable after assigning a temporary ip1
                            Camera_t camera(s_pTl->CreateDevice(bdi_new));
                            camera.Open();
                            camera.ChangeIpConfiguration(true, true);
                            camera.SetPersistentIpAddress(IP, Subnet, "0.0.0.0");
                            camera.Close();
                            s_pTl->RestartIpConfiguration(bdi.GetMacAddress());
                            cerr << "the IP has been assigned to " << bdi.GetSerialNumber() << "successfully" << endl;

                            cerr << "Thread will sleep for 2ms in order to wait on updated ARP table" << endl;
                            sleep(2); // needed to get ARP Table be updated;
                            cerr << "do you want assign IP for another camera or display the current status?" << endl;
                            cerr << "1= yes or 0 = exit" << endl;
                        }
                    } catch (GenICam::GenericException &e) {
                        cerr << "only a temporary IP address has been assigned, because camera is not reachable after assigning your addresses . Try once more " << endl;
                        cerr << "1= repeat or 0 = exit" << endl;
                    }

                    selction = readKB(1);
                } // if (selection >0)
            } else {
                cerr << " no device found, will try once more in 2 s" << endl;
                sleep(2);
            }
        } while (selction != 0);
    } catch (GenICam::GenericException &e) {
        // Error handling.
        cerr << "An exception occurred." << endl
                << e.GetDescription() << endl;
        exitCode = 1;
    }
    WriteToFile();

     Pylon::PylonTerminate();
    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press Enter to exit." << endl;
    while (cin.get() != '\n');
    return exitCode;
}


string AutoProbe(CBaslerGigEDeviceInfo &bdi, IGigETransportLayer* s_pTl)
{
    String_t OriginalIP =  bdi.GetIpAddress();
    String_t OriginalSubnet = bdi.GetSubnetMask();  
    vector <tuple <string,string>> InterfaceList;
    enumurateNIC(&InterfaceList);
    for(auto& el : InterfaceList)
    {
        
        string IPaddr = get<0>(el);
        string Subnet = get<1>(el);
        bool done = false;
        in_addr_t TemIP = inet_addr(IPaddr.c_str());
        TemIP = ntohl(TemIP);
        TemIP += 1;
        TemIP = ntohl(TemIP);
        struct in_addr address_struct;
        address_struct.s_addr = TemIP;
        string camIP = inet_ntoa(address_struct);
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


void enumurateNIC(vector<tuple <string,string>> *InterfaceList) 
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
            string s = addr;
            sa= (struct sockaddr_in *) ifa->ifa_netmask;
            addr= inet_ntoa(sa->sin_addr);        
            string subnet = addr;
            if (s != "127.0.0.1") 
            {
                InterfaceList->push_back(tuple<string, string>(s,subnet));
                Log.push_back(addr);
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
            string s = addr;
            if (s != "127.0.0.1")
            {
                stringstream ss ;
                ss << "Interfaces " <<" " << ifa->ifa_name  << " "<< addr;
                  printf("Interface: %s\tAddress: %s\n", ifa->ifa_name, addr);
                   Log.push_back(ss.str());
            }     
            cerr << endl;
        }
    }
    freeifaddrs(ifap);
}

bool checkSubNetFormat(char* _SubNet) 
{
    string strTempSubNet = _SubNet;
    istringstream ss(strTempSubNet);
    string token;
    vector<string> result;
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
                    istringstream iss (result[i]);
                    iss >> number;
                    if (iss.fail())
                        return false;
                    if  (!((number >= 0 ) && (number <= 255 )))
                        return false;
                }
            return true;
        }
    } else
        return false;
}

bool checkIPFormat(char* _IP) 
{
    string strTempIP = _IP;
    istringstream ss(strTempIP);
    string token;
    vector<string> result;
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
                    istringstream iss (result[i]);
                    iss >> number;
                    if (iss.fail())
                        return false;
                    if  (!((number >= 0 ) && (number <= 255 )))
                        return false;
                }
            return true;
        }
    } else
        return false;
}

int readKB(int iNumberofCamera) 
{
    int x = -1;
    cerr << "type in a number between 0 and " << iNumberofCamera << "  " << endl;
    do {
        x = getchar();
        if (x >= '0' || x <= '9') {
            x = x - '0'; // c will have the value of the digit in the range 0-9
            if (x > iNumberofCamera)
                cerr << "type in a number between 0 and " << iNumberofCamera << "  " << endl;
        }

    } while (x < 0 || x > iNumberofCamera);

    return x;
}

void DisplayCurrentStatus(DeviceInfoList_t &lstDevices, IGigETransportLayer* s_pTl) 
{
    bool bDone = false;
    if (lstDevices.size() > 0) 
    {
        for (uint x = 0; x < lstDevices.size(); x++) 
        {
            cerr << "Camera No :      SN:            Current IP       Interface IP       Status " << endl << endl;
            Pylon::EDeviceAccessiblityInfo isAccessable;
            //Camera_t camera=s_pTl->CreateDevice(lstDevices[x]);
            s_pTl->IsDeviceAccessible(lstDevices[x], Control, &isAccessable);
            CBaslerGigEDeviceInfo &bdi = static_cast<CBaslerGigEDeviceInfo&> (lstDevices[x]);

            string status = "";
            
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
            cerr << "    " << x + 1 << "         " << bdi.GetSerialNumber() << "         " << bdi.GetIpAddress() << "     " << bdi.GetInterface() << "       " << status << endl;
            cerr << endl;
        }
    }
}

