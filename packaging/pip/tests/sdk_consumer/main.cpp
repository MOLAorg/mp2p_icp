/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
// Minimal C++ program built against the C++ SDK of the mp2p-icp wheel.
// Usage: sdk_consumer <file.mm>

#include <mp2p_icp/ICP.h>
#include <mp2p_icp_filters/FilterDecimateVoxels.h>
#include <mrpt/rtti/CObject.h>
#include <sdk_consumer_lib.h>

#include <iostream>

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <file.mm>\n";
        return 1;
    }

    // A map from the library built with mola_add_library():
    const auto m = sdk_consumer::make_map(10);
    if (sdk_consumer::count_points(*m) != 10)
    {
        return 1;
    }

    // .mm files:
    mp2p_icp::metric_map_t loaded;
    if (!loaded.load_from_file(argv[1]))
    {
        std::cerr << "Cannot load " << argv[1] << "\n";
        return 1;
    }
    std::cout << "map: " << loaded.contents_summary() << "\n";

    // The other libraries, and their classes in the MRPT registry:
    mp2p_icp::ICP                          icp;
    mp2p_icp_filters::FilterDecimateVoxels filter;
    for (const char* c :
         {"mp2p_icp::ICP", "mp2p_icp::Matcher_Point2Plane",
          "mp2p_icp_filters::FilterDecimateVoxels"})
    {
        if (!mrpt::rtti::findRegisteredClass(c))
        {
            std::cerr << "Class not registered: " << c << "\n";
            return 1;
        }
    }
    std::cout << "classes: " << icp.GetRuntimeClass()->className << ", "
              << filter.GetRuntimeClass()->className << "\n";
    return 0;
}
