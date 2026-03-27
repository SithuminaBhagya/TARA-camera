#include "GalaxyIncludes.h"
#include <iostream>

int main()
{
    try
    {
        IGXFactory::GetInstance().Init();

        GxIAPICPP::gxdeviceinfo_vector vecDeviceInfo;
        IGXFactory::GetInstance().UpdateDeviceList(1000, vecDeviceInfo);

        int camCount = (int)vecDeviceInfo.size();

        if (camCount == 0)
        {
            std::cout << "No cameras found." << std::endl;
            IGXFactory::GetInstance().Uninit();
            return -1;
        }

        std::cout << "Found " << camCount << " camera(s):\n" << std::endl;

        for (int i = 0; i < camCount; ++i)
        {
            std::cout << "Camera " << i << ":" << std::endl;
            std::cout << "  Model  : " << vecDeviceInfo[i].GetModelName() << std::endl;
            std::cout << "  SN     : " << vecDeviceInfo[i].GetSN()        << std::endl;
            std::cout << "  IP     : " << vecDeviceInfo[i].GetIP()        << std::endl;
            std::cout << "  MAC    : " << vecDeviceInfo[i].GetMAC()       << std::endl;
            std::cout << "  NIC IP : " << vecDeviceInfo[i].GetNICIP()     << std::endl;
            std::cout << std::endl;
        }

        IGXFactory::GetInstance().Uninit();
    }
    catch (CGalaxyException& e)
    {
        std::cout << "Camera error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}