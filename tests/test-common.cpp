/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-common.cpp
 * @brief  Unit tests common utilities
 * @author Jose Luis Blanco Claraco
 * @date   July 11, 2020
 */

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>
#include <fstream>

#include "test-common.h"

// Loads from XYZ file, possibly gz-compressed:
mrpt::maps::CSimplePointsMap::Ptr load_xyz_file(const std::string& fil)
{
    ASSERT_FILE_EXISTS_(fil);

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
    {
        std::ofstream fo;
        fo.open(tmpFil.c_str());
        ASSERT_(fo.is_open());
        fo << buf;
    }

    auto m = mrpt::maps::CSimplePointsMap::Create();
    m->load3D_from_text_file(tmpFil);
    ASSERT_GT_(m->size(), 100U);

    return m;
}
