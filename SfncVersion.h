//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2009-2014 Basler AG
//  http://www.baslerweb.com
//  Author: Andreas Gau
//-----------------------------------------------------------------------------
/*!
\file
\brief  Contains the support for SFNC version handling.
*/

#ifndef INCLUDED_SFNCVERSION_H_3193628
#define INCLUDED_SFNCVERSION_H_3193628

#include <pylon/Platform.h>
#include <pylon/PylonVersionInfo.h>
#include <GenApi/GenApi.h>
#include <GenApi/Pointer.h>
#include <GenApi/INodeMap.h>

namespace Pylon
{
    /** \addtogroup Pylon_InstantCameraApiGeneric
     * @{
     */
    /// Constant for undefined SFNC version.
    static const VersionInfo Sfnc_VersionUndefined(0,0,0);
    /// Constant for SFNC version 1.2.1.
    static const VersionInfo Sfnc_1_2_1(1,2,1);
    /// Constant for SFNC version 1.3.0.
    static const VersionInfo Sfnc_1_3_0(1,3,0);
    /// Constant for SFNC version 1.4.0.
    static const VersionInfo Sfnc_1_4_0(1,4,0);
    /// Constant for SFNC version 1.5.0.
    static const VersionInfo Sfnc_1_5_0(1,5,0);
    /// Constant for SFNC version 1.5.1.
    static const VersionInfo Sfnc_1_5_1(1,5,1);
    /// Constant for SFNC version 2.0.0. This version or a later version is used by USB camera devices.
    static const VersionInfo Sfnc_2_0_0(2,0,0);
    static const VersionInfo Sfnc_2_1_0(2,1,0);
    static const VersionInfo Sfnc_2_2_0(2,2,0);

    /**
     * @}
     */

    /// Helper function for getting the SFNC version from the camera device node map.
    inline VersionInfo GetSfncVersion( GenApi::INodeMap* pNodeMap)
    {
        // If pNodeMap is a valid node map.
        if ( pNodeMap)
        {
            // Check to see whether node for major version exists.
            GenApi::CIntegerPtr major = pNodeMap->GetNode("DeviceSFNCVersionMajor");
            if (major)
            {
                // Get further version Info.
                GenApi::CIntegerPtr minor = pNodeMap->GetNode("DeviceSFNCVersionMinor"   );
                GenApi::CIntegerPtr subminor = pNodeMap->GetNode("DeviceSFNCVersionSubMinor");

                if ( !IsReadable(major) || !IsReadable(minor) || !IsReadable(subminor))
                {
                    throw RUNTIME_EXCEPTION("Failed to read SFNC version from camera device node map.");
                }

                VersionInfo v( static_cast<unsigned int>(major->GetValue()   ),
                               static_cast<unsigned int>(minor->GetValue()   ),
                               static_cast<unsigned int>(subminor->GetValue()));
                return v;
            }
        }

        return Sfnc_VersionUndefined;
    }
}

#endif /* INCLUDED_SFNCVERSION_H_3193628 */
