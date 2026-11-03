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

#include "kmz_writer.h"

#include <zlib.h>

#include <fstream>
#include <limits>
#include <stdexcept>

namespace mm2kml
{
namespace
{
void put_u16_le(std::vector<uint8_t>& buf, uint32_t v)
{
    buf.push_back(static_cast<uint8_t>(v & 0xff));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xff));
}

void put_u32_le(std::vector<uint8_t>& buf, uint32_t v)
{
    put_u16_le(buf, v & 0xffff);
    put_u16_le(buf, v >> 16);
}

void put_u32_be(std::vector<uint8_t>& buf, uint32_t v)
{
    buf.push_back(static_cast<uint8_t>((v >> 24) & 0xff));
    buf.push_back(static_cast<uint8_t>((v >> 16) & 0xff));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xff));
    buf.push_back(static_cast<uint8_t>(v & 0xff));
}

// Compresses with deflate. With zlibHeader=false, a raw deflate stream is
// produced (as used inside ZIP files); otherwise, a zlib stream (as in PNG).
std::vector<uint8_t> deflate_buffer(const uint8_t* data, size_t len, bool zlibHeader)
{
    if (len > std::numeric_limits<uInt>::max())
    {
        throw std::runtime_error("deflate_buffer: input too large");
    }
    z_stream  strm{};
    const int windowBits = zlibHeader ? 15 : -15;
    if (deflateInit2(&strm, Z_BEST_COMPRESSION, Z_DEFLATED, windowBits, 8, Z_DEFAULT_STRATEGY) !=
        Z_OK)
    {
        throw std::runtime_error("deflateInit2() failed");
    }
    std::vector<uint8_t> out(deflateBound(&strm, static_cast<uLong>(len)));
    strm.next_in   = const_cast<Bytef*>(data);
    strm.avail_in  = static_cast<uInt>(len);
    strm.next_out  = out.data();
    strm.avail_out = static_cast<uInt>(out.size());
    const int ret  = deflate(&strm, Z_FINISH);
    deflateEnd(&strm);
    if (ret != Z_STREAM_END)
    {
        throw std::runtime_error("deflate() failed");
    }
    out.resize(strm.total_out);
    return out;
}

void png_chunk(std::vector<uint8_t>& png, const char* type, const std::vector<uint8_t>& data)
{
    put_u32_be(png, static_cast<uint32_t>(data.size()));
    const size_t typeStart = png.size();
    png.insert(png.end(), type, type + 4);
    png.insert(png.end(), data.begin(), data.end());
    const uLong crc = crc32(0L, png.data() + typeStart, static_cast<uInt>(png.size() - typeStart));
    put_u32_be(png, static_cast<uint32_t>(crc));
}

// Fixed DOS date/time (1980-01-01 00:00) so outputs are reproducible.
constexpr uint32_t ZIP_DOS_TIME = 0;
constexpr uint32_t ZIP_DOS_DATE = (1 << 5) | 1;

}  // namespace

std::vector<uint8_t> encode_png_rgba(
    const std::vector<uint8_t>& rgba, uint32_t width, uint32_t height)
{
    const size_t rowBytes = static_cast<size_t>(width) * 4;
    if (rgba.size() != rowBytes * height)
    {
        throw std::invalid_argument("encode_png_rgba: buffer size mismatch");
    }

    // Each scanline is prefixed with its filter type (0: none)
    std::vector<uint8_t> raw;
    raw.reserve((rowBytes + 1) * height);
    for (uint32_t r = 0; r < height; r++)
    {
        raw.push_back(0);
        const auto* row = rgba.data() + r * rowBytes;
        raw.insert(raw.end(), row, row + rowBytes);
    }

    std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};

    std::vector<uint8_t> ihdr;
    put_u32_be(ihdr, width);
    put_u32_be(ihdr, height);
    ihdr.push_back(8);  // bit depth
    ihdr.push_back(6);  // color type: RGBA
    ihdr.push_back(0);  // compression
    ihdr.push_back(0);  // filter
    ihdr.push_back(0);  // interlace
    png_chunk(png, "IHDR", ihdr);
    png_chunk(png, "IDAT", deflate_buffer(raw.data(), raw.size(), true));
    png_chunk(png, "IEND", {});
    return png;
}

void ZipWriter::add_file(const std::string& name, const std::vector<uint8_t>& data)
{
    if (data.size() > std::numeric_limits<uint32_t>::max())
    {
        throw std::runtime_error("ZipWriter: files larger than 4 GiB are not supported");
    }
    Entry e;
    e.name     = name;
    e.rawSize  = static_cast<uint32_t>(data.size());
    e.crc      = static_cast<uint32_t>(crc32(0L, data.data(), static_cast<uInt>(data.size())));
    e.deflated = deflate_buffer(data.data(), data.size(), false);
    entries_.push_back(std::move(e));
}

void ZipWriter::add_file(const std::string& name, const std::string& data)
{
    add_file(name, std::vector<uint8_t>(data.begin(), data.end()));
}

bool ZipWriter::save(const std::string& fileName) const
{
    // Classic ZIP stores offsets as 32-bit values (no ZIP64 here):
    uint64_t archiveSize = 22;  // end of central directory record
    for (const auto& e : entries_)
    {
        archiveSize += 30 + 46 + 2 * e.name.size() + e.deflated.size();
    }
    if (archiveSize > std::numeric_limits<uint32_t>::max())
    {
        throw std::runtime_error("ZipWriter: archives larger than 4 GiB are not supported");
    }

    std::vector<uint8_t> out;
    std::vector<uint8_t> centralDir;

    for (const auto& e : entries_)
    {
        const auto offset   = static_cast<uint32_t>(out.size());
        const auto nameLen  = static_cast<uint32_t>(e.name.size());
        const auto compSize = static_cast<uint32_t>(e.deflated.size());

        // Local file header
        put_u32_le(out, 0x04034b50);
        put_u16_le(out, 20);  // version needed
        put_u16_le(out, 0);  // flags
        put_u16_le(out, 8);  // method: deflate
        put_u16_le(out, ZIP_DOS_TIME);
        put_u16_le(out, ZIP_DOS_DATE);
        put_u32_le(out, e.crc);
        put_u32_le(out, compSize);
        put_u32_le(out, e.rawSize);
        put_u16_le(out, nameLen);
        put_u16_le(out, 0);  // extra length
        out.insert(out.end(), e.name.begin(), e.name.end());
        out.insert(out.end(), e.deflated.begin(), e.deflated.end());

        // Central directory record
        put_u32_le(centralDir, 0x02014b50);
        put_u16_le(centralDir, 20);  // version made by
        put_u16_le(centralDir, 20);  // version needed
        put_u16_le(centralDir, 0);  // flags
        put_u16_le(centralDir, 8);  // method
        put_u16_le(centralDir, ZIP_DOS_TIME);
        put_u16_le(centralDir, ZIP_DOS_DATE);
        put_u32_le(centralDir, e.crc);
        put_u32_le(centralDir, compSize);
        put_u32_le(centralDir, e.rawSize);
        put_u16_le(centralDir, nameLen);
        put_u16_le(centralDir, 0);  // extra length
        put_u16_le(centralDir, 0);  // comment length
        put_u16_le(centralDir, 0);  // disk number
        put_u16_le(centralDir, 0);  // internal attributes
        put_u32_le(centralDir, 0);  // external attributes
        put_u32_le(centralDir, offset);
        centralDir.insert(centralDir.end(), e.name.begin(), e.name.end());
    }

    const auto cdOffset = static_cast<uint32_t>(out.size());
    out.insert(out.end(), centralDir.begin(), centralDir.end());

    // End of central directory
    put_u32_le(out, 0x06054b50);
    put_u16_le(out, 0);
    put_u16_le(out, 0);
    put_u16_le(out, static_cast<uint32_t>(entries_.size()));
    put_u16_le(out, static_cast<uint32_t>(entries_.size()));
    put_u32_le(out, static_cast<uint32_t>(centralDir.size()));
    put_u32_le(out, cdOffset);
    put_u16_le(out, 0);  // comment length

    std::ofstream f(fileName, std::ios::binary);
    if (!f.is_open())
    {
        return false;
    }
    f.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
    f.close();
    return !f.fail();
}

}  // namespace mm2kml
