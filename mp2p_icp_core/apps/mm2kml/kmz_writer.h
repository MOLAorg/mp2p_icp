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

/**
 * @file   mm2kml/kmz_writer.h
 * @brief  Minimal PNG encoder and ZIP archive writer, enough to build KMZ files
 * @author Jose Luis Blanco Claraco
 * @date   Nov 3, 2026
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mm2kml
{
/** Encodes an 8-bit RGBA image (row-major, 4 bytes per pixel) as a PNG file
 * in memory. */
std::vector<uint8_t> encode_png_rgba(
    const std::vector<uint8_t>& rgba, uint32_t width, uint32_t height);

/** Minimal writer of ZIP archives (deflate compression), as needed for KMZ
 * files. Google Earth requires the KML document to be the first entry. */
class ZipWriter
{
   public:
    void add_file(const std::string& name, const std::vector<uint8_t>& data);
    void add_file(const std::string& name, const std::string& data);

    /** Writes the archive to disk. Returns false on I/O errors. */
    [[nodiscard]] bool save(const std::string& fileName) const;

   private:
    struct Entry
    {
        std::string          name;
        uint32_t             crc     = 0;
        uint32_t             rawSize = 0;
        std::vector<uint8_t> deflated;
    };
    std::vector<Entry> entries_;
};

}  // namespace mm2kml
