/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file   load_xyz_file.cpp
 * @brief  Unit tests common utilities
 * @author Jose Luis Blanco Claraco
 * @date   July 11, 2020
 */

#include <mp2p_icp/load_xyz_file.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>

#include <fstream>

// Loads from XYZ file, possibly gz-compressed:
mrpt::maps::CSimplePointsMap::Ptr mp2p_icp::load_xyz_file(const std::string& fil)
{
    ASSERT_FILE_EXISTS_(fil);

    std::string fileToRead = fil;

    if (mrpt::system::extractFileExtension(fil) == "gz")
    {
        mrpt::io::CFileGZInputStream f(fil);
        std::string                  buf;
        while (!f.checkEOF())
        {
            const size_t N = 10000;
            std::string  tmp;
            tmp.resize(N);
            const auto n = f.Read(&tmp[0], N);
            tmp.resize(n);
            buf += tmp;
        }

        const auto tmpFil = mrpt::system::getTempFileName();

        std::ofstream fo;
        fo.open(tmpFil.c_str());
        ASSERT_(fo.is_open());
        fo << buf;

        fileToRead = tmpFil;
    }

    auto m = mrpt::maps::CSimplePointsMap::Create();
    m->load3D_from_text_file(fileToRead);
    ASSERTMSG_(
        m->size() > 1,
        mrpt::format(
            "Could not parse a valid point cloud from ASCII file '%s'", fileToRead.c_str()));

    return m;
}
