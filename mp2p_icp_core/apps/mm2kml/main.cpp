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
 * @file   mm2kml/main.cpp
 * @brief  Exports a georeferenced metric map (and optionally a trajectory) as
 *         a KMZ file for Google Earth
 * @author Jose Luis Blanco Claraco
 * @date   Nov 3, 2026
 */

#include <mp2p_icp/metricmap.h>
#include <mrpt/core/format.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/topography/conversions.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "kmz_writer.h"

namespace
{
// CLI flags:
CLI::App cmd{"mm2kml"};

std::string              argMapFile;
std::string              argOutput;
std::vector<std::string> argLayers;
std::string              argTrajectory;
std::string              argTrajectoryFrame      = "map";
std::string              argCloud                = "both";
double                   argMinHeightAboveGround = 0.35;
double                   argGroundCell           = 1.0;
double                   argColorMaxHeight       = 0;
double                   argOverlayResolution    = 0;
size_t                   argModelMaxPoints       = 100000;
double                   argModelPointSize       = 0;
double                   argModelAltitudeOffset  = 0;
double                   argHeading              = 0;
double                   argTourDuration         = 0;
double                   argTourTilt             = 60;
std::string              arg_plugins;

// Google Earth refuses ground overlay textures larger than this:
constexpr int MAX_OVERLAY_PIXELS = 2048;

// Number of discrete colors (materials) in the 3D model:
constexpr int MODEL_COLOR_BINS = 32;

struct GeoPoint
{
    double lat = 0;
    double lon = 0;
};

GeoPoint enu_to_geodetic(
    const mrpt::math::TPoint3D& enu, const mrpt::topography::TGeodeticCoords& origin)
{
    mrpt::topography::TGeocentricCoords geocentric;
    mrpt::topography::ENUToGeocentric(
        enu, origin, geocentric, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
    mrpt::topography::TGeodeticCoords out;
    mrpt::topography::geocentricToGeodetic(geocentric, out);
    return {out.lat.getDecimalValue(), out.lon.getDecimalValue()};
}

// Polynomial approximation of the "turbo" colormap. Input in [0,1].
std::array<uint8_t, 3> turbo_color(double x)
{
    x              = std::clamp(x, 0.0, 1.0);
    const auto   f = [](double v) { return static_cast<uint8_t>(std::clamp(v, 0.0, 1.0) * 255.0); };
    const double r =
        0.13572138 +
        x * (4.61539260 +
             x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))));
    const double g =
        0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 +
                                                              x * (4.27729857 + x * 2.82956604))));
    const double b =
        0.10667330 +
        x * (12.64194608 +
             x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))));
    return {f(r), f(g), f(b)};
}

uint64_t cell_key(int64_t ix, int64_t iy)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
           static_cast<uint64_t>(static_cast<uint32_t>(iy));
}

// 21 bits per axis: unique within +-1e6 voxels, wraps around beyond that.
uint64_t voxel_key(int64_t ix, int64_t iy, int64_t iz)
{
    constexpr uint64_t mask = (1ULL << 21) - 1;
    return ((static_cast<uint64_t>(ix) & mask) << 42) | ((static_cast<uint64_t>(iy) & mask) << 21) |
           (static_cast<uint64_t>(iz) & mask);
}

int64_t cell_idx(double coord, double cellSize)
{
    return static_cast<int64_t>(std::floor(coord / cellSize));
}

// Point cloud in ENU coordinates, plus the height of each point above the
// local ground.
struct EnuCloud
{
    std::vector<mrpt::math::TPoint3Df> pts;
    std::vector<float>                 heightAboveGround;
    double                             minX = 0;
    double                             maxX = 0;
    double                             minY = 0;
    double                             maxY = 0;
};

// Local ground = lowest point in each cell, smoothed with the minimum of the
// 3x3 neighborhood, so cells with only canopy returns are handled.
class GroundGrid
{
   public:
    GroundGrid(const std::vector<mrpt::math::TPoint3Df>& pts, double cellSize) : cell_(cellSize)
    {
        std::unordered_map<uint64_t, float> rawMin;
        for (const auto& p : pts)
        {
            const auto key = cell_key(cell_idx(p.x, cell_), cell_idx(p.y, cell_));
            auto       it  = rawMin.find(key);
            if (it == rawMin.end())
            {
                rawMin[key] = p.z;
            }
            else
            {
                it->second = std::min(it->second, p.z);
            }
        }
        for (const auto& p : pts)
        {
            const auto ix  = cell_idx(p.x, cell_);
            const auto iy  = cell_idx(p.y, cell_);
            const auto key = cell_key(ix, iy);
            if (ground_.count(key))
            {
                continue;
            }
            float g = rawMin.at(key);
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    const auto itN = rawMin.find(cell_key(ix + dx, iy + dy));
                    if (itN != rawMin.end())
                    {
                        g = std::min(g, itN->second);
                    }
                }
            }
            ground_[key] = g;
        }
    }

    [[nodiscard]] float ground_at(double x, double y) const
    {
        return ground_.at(cell_key(cell_idx(x, cell_), cell_idx(y, cell_)));
    }

    /// Ground height at the cell nearest to (x,y) among those with data.
    [[nodiscard]] float ground_near(double x, double y) const
    {
        const auto ix    = cell_idx(x, cell_);
        const auto iy    = cell_idx(y, cell_);
        float      best  = 0;
        int64_t    bestD = std::numeric_limits<int64_t>::max();
        for (const auto& [key, g] : ground_)
        {
            const int64_t dx = static_cast<int32_t>(key >> 32) - ix;
            const int64_t dy = static_cast<int32_t>(key & 0xffffffff) - iy;
            const int64_t d  = dx * dx + dy * dy;
            if (d < bestD)
            {
                bestD = d;
                best  = g;
            }
        }
        return best;
    }

   private:
    double                              cell_;
    std::unordered_map<uint64_t, float> ground_;
};

std::string fmt_lonlat(const GeoPoint& g) { return mrpt::format("%.9f,%.9f", g.lon, g.lat); }

std::string make_overlay_png(
    const EnuCloud& cloud, const std::vector<size_t>& idxs, double colorMaxHeight, double& res,
    int& width, int& height)
{
    const double ext = std::max(cloud.maxX - cloud.minX, cloud.maxY - cloud.minY);
    res              = argOverlayResolution > 0 ? argOverlayResolution
                                                : std::max(0.05, ext / (MAX_OVERLAY_PIXELS - 48));
    width            = static_cast<int>(std::ceil((cloud.maxX - cloud.minX) / res));
    height           = static_cast<int>(std::ceil((cloud.maxY - cloud.minY) / res));
    if (width > MAX_OVERLAY_PIXELS || height > MAX_OVERLAY_PIXELS)
    {
        THROW_EXCEPTION_FMT(
            "Overlay image would be %ix%i pixels, larger than the %i pixels that Google Earth "
            "accepts. Increase --overlay-resolution.",
            width, height, MAX_OVERLAY_PIXELS);
    }

    std::vector<uint8_t> rgba(static_cast<size_t>(width) * height * 4, 0);

    // Draw lower points first, so the highest ones (e.g. tree canopies) end on top:
    for (const size_t i : idxs)
    {
        const auto& p   = cloud.pts[i];
        const auto  col = turbo_color(cloud.heightAboveGround[i] / colorMaxHeight);
        const int   px  = static_cast<int>((p.x - cloud.minX) / res);
        const int   py  = height - 1 - static_cast<int>((p.y - cloud.minY) / res);
        for (int dx = 0; dx <= 1; dx++)
        {
            for (int dy = 0; dy <= 1; dy++)
            {
                const int x = std::clamp(px + dx, 0, width - 1);
                const int y = std::clamp(py + dy, 0, height - 1);
                auto*     d = &rgba[(static_cast<size_t>(y) * width + x) * 4];
                d[0]        = col[0];
                d[1]        = col[1];
                d[2]        = col[2];
                d[3]        = 220;
            }
        }
    }
    const auto png = mm2kml::encode_png_rgba(rgba, width, height);
    return std::string(png.begin(), png.end());
}

// COLLADA model with one small octahedron per (decimated) point, grouped in
// color bins (one material each). Coordinates are ENU, relative to the
// georeferencing origin.
std::string make_model_dae(
    const EnuCloud& cloud, const std::vector<size_t>& idxs, double colorMaxHeight,
    size_t& numPoints, double& voxelSize)
{
    // Voxel decimation, growing the voxel size until under the points budget.
    std::vector<size_t> kept;
    voxelSize = 0.05;
    for (;;)
    {
        kept.clear();
        std::unordered_set<uint64_t> occupied;
        for (const size_t i : idxs)
        {
            const auto& p = cloud.pts[i];
            if (!occupied
                     .insert(voxel_key(
                         cell_idx(p.x, voxelSize), cell_idx(p.y, voxelSize),
                         cell_idx(p.z, voxelSize)))
                     .second)
            {
                continue;
            }
            kept.push_back(i);
            if (kept.size() > argModelMaxPoints)
            {
                break;
            }
        }
        if (kept.size() <= argModelMaxPoints)
        {
            break;
        }
        voxelSize *= 1.25;
    }
    numPoints = kept.size();

    const double r = argModelPointSize > 0 ? 0.5 * argModelPointSize : 0.6 * voxelSize;

    // Octahedron: vertices +x,-x,+y,-y,+z,-z; outward-facing triangles:
    constexpr std::array<std::array<int, 3>, 8> faces = {
        {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}, {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}}};

    std::array<std::vector<size_t>, MODEL_COLOR_BINS> bins;
    for (size_t k = 0; k < kept.size(); k++)
    {
        const double h = cloud.heightAboveGround[kept[k]] / colorMaxHeight;
        const int bin = std::clamp(static_cast<int>(h * MODEL_COLOR_BINS), 0, MODEL_COLOR_BINS - 1);
        bins[bin].push_back(k);
    }

    std::ostringstream ss;
    ss << R"(<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_effects>
)";
    for (int b = 0; b < MODEL_COLOR_BINS; b++)
    {
        const auto c = turbo_color((b + 0.5) / MODEL_COLOR_BINS);
        ss << mrpt::format(
            "    <effect id=\"fx%i\"><profile_COMMON><technique sid=\"t\"><lambert>"
            "<emission><color>%.3f %.3f %.3f 1</color></emission>"
            "<diffuse><color>%.3f %.3f %.3f 1</color></diffuse>"
            "</lambert></technique></profile_COMMON></effect>\n",
            b, 0.5 * c[0] / 255.0, 0.5 * c[1] / 255.0, 0.5 * c[2] / 255.0, c[0] / 255.0,
            c[1] / 255.0, c[2] / 255.0);
    }
    ss << "  </library_effects>\n  <library_materials>\n";
    for (int b = 0; b < MODEL_COLOR_BINS; b++)
    {
        ss << mrpt::format(
            "    <material id=\"mat%i\"><instance_effect url=\"#fx%i\"/></material>\n", b, b);
    }
    ss << "  </library_materials>\n  <library_geometries>\n"
       << "    <geometry id=\"cloud\"><mesh>\n"
       << mrpt::format(
              "      <source id=\"pos\"><float_array id=\"pos-a\" count=\"%zu\">",
              kept.size() * 6 * 3);
    for (const size_t i : kept)
    {
        const auto& p = cloud.pts[i];
        ss << mrpt::format(
            "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f "
            "%.2f ",
            p.x + r, p.y, p.z, p.x - r, p.y, p.z, p.x, p.y + r, p.z, p.x, p.y - r, p.z, p.x, p.y,
            p.z + r, p.x, p.y, p.z - r);
    }
    ss << "</float_array>\n"
       << mrpt::format(
              "        <technique_common><accessor source=\"#pos-a\" count=\"%zu\" stride=\"3\">"
              "<param name=\"X\" type=\"float\"/><param name=\"Y\" type=\"float\"/>"
              "<param name=\"Z\" type=\"float\"/></accessor></technique_common>\n",
              kept.size() * 6)
       << "      </source>\n"
       << "      <vertices id=\"verts\"><input semantic=\"POSITION\" "
          "source=\"#pos\"/></vertices>\n";
    for (int b = 0; b < MODEL_COLOR_BINS; b++)
    {
        if (bins[b].empty())
        {
            continue;
        }
        ss << mrpt::format(
            "      <triangles material=\"m%i\" count=\"%zu\">"
            "<input semantic=\"VERTEX\" source=\"#verts\" offset=\"0\"/><p>",
            b, bins[b].size() * faces.size());
        for (const size_t k : bins[b])
        {
            const size_t base = k * 6;
            for (const auto& f : faces)
            {
                ss << base + f[0] << ' ' << base + f[1] << ' ' << base + f[2] << ' ';
            }
        }
        ss << "</p></triangles>\n";
    }
    ss << "    </mesh></geometry>\n  </library_geometries>\n"
       << "  <library_visual_scenes><visual_scene id=\"scene\"><node id=\"n\">"
       << "<instance_geometry url=\"#cloud\"><bind_material><technique_common>\n";
    for (int b = 0; b < MODEL_COLOR_BINS; b++)
    {
        ss << mrpt::format("        <instance_material symbol=\"m%i\" target=\"#mat%i\"/>\n", b, b);
    }
    ss << "      </technique_common></bind_material></instance_geometry></node>"
       << "</visual_scene></library_visual_scenes>\n"
       << "  <scene><instance_visual_scene url=\"#scene\"/></scene>\n</COLLADA>\n";
    return ss.str();
}

std::string kml_lookat(
    const GeoPoint& center, double heading, double tilt, double range, const char* indent)
{
    return mrpt::format(
        "%s<LookAt><longitude>%.9f</longitude><latitude>%.9f</latitude><altitude>0</altitude>"
        "<heading>%.2f</heading><tilt>%.2f</tilt><range>%.1f</range>"
        "<altitudeMode>relativeToGround</altitudeMode></LookAt>\n",
        indent, center.lon, center.lat, heading, tilt, range);
}

void run_mm2kml()
{
    if (!arg_plugins.empty())
    {
        std::string errMsg;
        std::cout << "Loading plugin(s): " << arg_plugins << std::endl;
        if (!mrpt::system::loadPluginModules(arg_plugins, errMsg))
        {
            throw std::runtime_error(errMsg);
        }
    }

    ASSERT_FILE_EXISTS_(argMapFile);
    ASSERTMSG_(
        argCloud == "both" || argCloud == "overlay" || argCloud == "model" || argCloud == "none",
        "--cloud must be one of: both, overlay, model, none");

    std::cout << "[mm2kml] Reading input map from: '" << argMapFile << "'..." << std::endl;
    mp2p_icp::metric_map_t mm;
    mm.load_from_file(argMapFile);
    std::cout << "[mm2kml] Map contents: " << mm.contents_summary() << "\n";

    ASSERTMSG_(
        mm.georeferencing.has_value(),
        "The input map has no georeferencing information. Use mm-georef to add it.");

    const auto& geoOrigin  = mm.georeferencing->geo_coord;
    const auto  T_enu_map  = mm.georeferencing->T_enu_to_map.mean;
    const bool  wantCloud  = argCloud != "none";
    const bool  wantOverly = argCloud == "both" || argCloud == "overlay";
    const bool  wantModel  = argCloud == "both" || argCloud == "model";

    // Collect points in ENU:
    EnuCloud cloud;
    if (wantCloud)
    {
        std::vector<std::string> layers = argLayers;
        if (layers.empty())
        {
            for (const auto& [name, _] : mm.layers)
            {
                layers.push_back(name);
            }
        }
        for (const auto& name : layers)
        {
            ASSERTMSG_(mm.layers.count(name) == 1, "Layer not found in metric map: " + name);
            const auto* pts = mp2p_icp::MapToPointsMap(*mm.layers.at(name));
            if (!pts)
            {
                if (!argLayers.empty())
                {
                    THROW_EXCEPTION_FMT(
                        "Layer '%s' cannot be converted into a point cloud.", name.c_str());
                }
                std::cout << "[mm2kml] Skipping non point-cloud layer: '" << name << "'\n";
                continue;
            }
            const auto& xs = pts->getPointsBufferRef_x();
            const auto& ys = pts->getPointsBufferRef_y();
            const auto& zs = pts->getPointsBufferRef_z();
            for (size_t i = 0; i < xs.size(); i++)
            {
                double ex = 0;
                double ey = 0;
                double ez = 0;
                T_enu_map.composePoint(xs[i], ys[i], zs[i], ex, ey, ez);
                cloud.pts.emplace_back(
                    static_cast<float>(ex), static_cast<float>(ey), static_cast<float>(ez));
            }
            std::cout << "[mm2kml] Layer '" << name << "': " << xs.size() << " points\n";
        }
        ASSERTMSG_(!cloud.pts.empty(), "No points found in the selected layers.");
    }

    // Trajectory (in the map frame) to ENU:
    std::vector<mrpt::math::TPoint3D> trajEnu;
    if (!argTrajectory.empty())
    {
        ASSERT_FILE_EXISTS_(argTrajectory);
        mrpt::poses::CPose3DInterpolator traj;
        ASSERTMSG_(
            traj.loadFromTextFile_TUM(argTrajectory),
            "Error loading TUM trajectory: " + argTrajectory);
        for (const auto& [t, p] : traj)
        {
            const auto poseEnu = argTrajectoryFrame == "enu" ? mrpt::poses::CPose3D(p)
                                                             : T_enu_map + mrpt::poses::CPose3D(p);
            const auto pEnu    = mrpt::math::TPoint3D(poseEnu.x(), poseEnu.y(), poseEnu.z());
            // Decimate to avoid huge LineStrings:
            if (trajEnu.empty() || (pEnu - trajEnu.back()).norm() > 0.25)
            {
                trajEnu.push_back(pEnu);
            }
        }
        std::cout << "[mm2kml] Trajectory: " << traj.size() << " poses (" << trajEnu.size()
                  << " after decimation)\n";
    }
    ASSERTMSG_(wantCloud || !trajEnu.empty(), "Nothing to export: no cloud and no trajectory.");

    // Bounding box (ENU):
    {
        bool first = true;
        auto add   = [&](double x, double y)
        {
            if (first)
            {
                cloud.minX = cloud.maxX = x;
                cloud.minY = cloud.maxY = y;
                first                   = false;
                return;
            }
            cloud.minX = std::min(cloud.minX, x);
            cloud.maxX = std::max(cloud.maxX, x);
            cloud.minY = std::min(cloud.minY, y);
            cloud.maxY = std::max(cloud.maxY, y);
        };
        for (const auto& p : cloud.pts)
        {
            add(p.x, p.y);
        }
        for (const auto& p : trajEnu)
        {
            add(p.x, p.y);
        }
        const double margin = 2.0;
        cloud.minX -= margin;
        cloud.minY -= margin;
        cloud.maxX += margin;
        cloud.maxY += margin;
    }

    const GeoPoint center = enu_to_geodetic(
        {0.5 * (cloud.minX + cloud.maxX), 0.5 * (cloud.minY + cloud.maxY), 0}, geoOrigin);
    const double extent = std::max(cloud.maxX - cloud.minX, cloud.maxY - cloud.minY);

    mm2kml::ZipWriter  kmz;
    std::string        pngData;
    std::string        daeData;
    std::ostringstream kml;

    const std::string docName = mrpt::system::extractFileName(argMapFile) + "." +
                                mrpt::system::extractFileExtension(argMapFile);
    kml << R"(<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">
<Document>
)"
        << "  <name>" << docName << "</name>\n"
        << kml_lookat(center, argHeading, wantModel ? 45.0 : 0.0, 1.3 * extent, "  ")
        << "  <Style id=\"traj\"><LineStyle><color>ff00d7ff</color><width>4</width></LineStyle>"
           "</Style>\n";

    if (wantCloud)
    {
        const GroundGrid ground(cloud.pts, argGroundCell);
        cloud.heightAboveGround.resize(cloud.pts.size());
        for (size_t i = 0; i < cloud.pts.size(); i++)
        {
            const auto& p              = cloud.pts[i];
            cloud.heightAboveGround[i] = p.z - ground.ground_at(p.x, p.y);
        }

        std::vector<size_t> idxs;
        for (size_t i = 0; i < cloud.pts.size(); i++)
        {
            if (argMinHeightAboveGround <= 0 ||
                cloud.heightAboveGround[i] > argMinHeightAboveGround)
            {
                idxs.push_back(i);
            }
        }
        ASSERTMSG_(!idxs.empty(), "No points left after removing the ground.");
        std::sort(
            idxs.begin(), idxs.end(),
            [&](size_t a, size_t b)
            { return cloud.heightAboveGround[a] < cloud.heightAboveGround[b]; });

        double colorMaxHeight = argColorMaxHeight;
        if (colorMaxHeight <= 0)
        {
            // 98% percentile of heights (idxs is sorted by height):
            colorMaxHeight =
                std::max(1.0, double(cloud.heightAboveGround[idxs[(idxs.size() * 98) / 100]]));
        }
        std::cout << "[mm2kml] " << idxs.size() << " points above ground, color scale: 0-"
                  << colorMaxHeight << " m\n";

        if (wantOverly)
        {
            double res    = 0;
            int    width  = 0;
            int    height = 0;
            pngData       = make_overlay_png(cloud, idxs, colorMaxHeight, res, width, height);
            const auto nw = enu_to_geodetic({cloud.minX, cloud.maxY, 0}, geoOrigin);
            const auto se = enu_to_geodetic({cloud.maxX, cloud.minY, 0}, geoOrigin);
            kml << "  <GroundOverlay>\n    <name>Point cloud (2D overlay)</name>\n"
                << "    <drawOrder>1</drawOrder>\n    <Icon><href>overlay.png</href></Icon>\n"
                << mrpt::format(
                       "    <LatLonBox><north>%.9f</north><south>%.9f</south><east>%.9f</east>"
                       "<west>%.9f</west></LatLonBox>\n",
                       nw.lat, se.lat, se.lon, nw.lon)
                << "  </GroundOverlay>\n";
            std::cout << "[mm2kml] Overlay: " << width << "x" << height << " px, " << res
                      << " m/px\n";
        }
        if (wantModel)
        {
            size_t numPts    = 0;
            double voxelSize = 0;
            daeData          = make_model_dae(cloud, idxs, colorMaxHeight, numPts, voxelSize);

            // Place the model so the ground under the map origin lies on the terrain:
            const double alt = -ground.ground_near(0, 0) + argModelAltitudeOffset;
            kml << "  <Placemark>\n    <name>Point cloud (3D model)</name>\n    <Model>\n"
                << "      <altitudeMode>relativeToGround</altitudeMode>\n"
                << mrpt::format(
                       "      <Location><longitude>%.9f</longitude><latitude>%.9f</latitude>"
                       "<altitude>%.3f</altitude></Location>\n",
                       geoOrigin.lon.getDecimalValue(), geoOrigin.lat.getDecimalValue(), alt)
                << "      <Orientation><heading>0</heading><tilt>0</tilt><roll>0</roll>"
                   "</Orientation>\n"
                << "      <Scale><x>1</x><y>1</y><z>1</z></Scale>\n"
                << "      <Link><href>model.dae</href></Link>\n    </Model>\n  </Placemark>\n";
            std::cout << "[mm2kml] 3D model: " << numPts << " points (voxel size: " << voxelSize
                      << " m)\n";
        }
    }

    if (!trajEnu.empty())
    {
        kml << "  <Placemark>\n    <name>Trajectory</name>\n    <styleUrl>#traj</styleUrl>\n"
            << "    <LineString><tessellate>1</tessellate>"
               "<altitudeMode>clampToGround</altitudeMode><coordinates>\n";
        for (const auto& p : trajEnu)
        {
            kml << "      " << fmt_lonlat(enu_to_geodetic(p, geoOrigin)) << ",0\n";
        }
        kml << "    </coordinates></LineString>\n  </Placemark>\n";
    }

    if (argTourDuration > 0)
    {
        // A full orbit around the map center:
        constexpr int nSteps = 72;
        kml << "  <gx:Tour>\n    <name>Fly-by</name>\n    <gx:Playlist>\n";
        for (int i = 0; i <= nSteps; i++)
        {
            const double heading = argHeading + 360.0 * i / nSteps;
            const double dur     = i == 0 ? 3.0 : argTourDuration / nSteps;
            kml << mrpt::format(
                       "      <gx:FlyTo><gx:duration>%.3f</gx:duration>"
                       "<gx:flyToMode>%s</gx:flyToMode>\n",
                       dur, i == 0 ? "bounce" : "smooth")
                << kml_lookat(
                       center, std::fmod(heading, 360.0), argTourTilt, 1.1 * extent, "        ")
                << "      </gx:FlyTo>\n";
        }
        kml << "    </gx:Playlist>\n  </gx:Tour>\n";
    }

    kml << "</Document>\n</kml>\n";

    // KML document must be the first entry in a KMZ:
    kmz.add_file("doc.kml", kml.str());
    if (!pngData.empty())
    {
        kmz.add_file("overlay.png", pngData);
    }
    if (!daeData.empty())
    {
        kmz.add_file("model.dae", daeData);
    }
    ASSERTMSG_(kmz.save(argOutput), "Error writing output file: " + argOutput);
    std::cout << "[mm2kml] Saved: '" << argOutput << "'\n";
}

}  // namespace

int main(int argc, char** argv)
{
    cmd.add_option("input", argMapFile, "Georeferenced metric map file (*.mm)")->required();
    cmd.add_option("-o,--output", argOutput, "Output Google Earth file (*.kmz)")->required();
    cmd.add_option(
        "-l,--layer", argLayers,
        "Point cloud layer to export. If not provided, all point cloud layers are merged. "
        "This argument can appear several times.");
    cmd.add_option(
        "-t,--trajectory", argTrajectory,
        "Optional trajectory file in TUM format, drawn as a line clamped to the ground. See "
        "--trajectory-frame.");
    cmd.add_option(
           "--trajectory-frame", argTrajectoryFrame,
           "Frame of the --trajectory poses: 'map' (the local frame of the input map) or 'enu' "
           "(East-North-Up, e.g. a trajectory already converted with the map georeferencing).")
        ->check(CLI::IsMember({"map", "enu"}))
        ->capture_default_str();
    cmd.add_option(
           "--cloud", argCloud,
           "How to export the point cloud: 'overlay' (top-down image draped on the terrain), "
           "'model' (3D COLLADA model, one small solid per decimated point), 'both', or 'none'.")
        ->check(CLI::IsMember({"both", "overlay", "model", "none"}))
        ->capture_default_str();
    cmd.add_option(
           "--min-height-above-ground", argMinHeightAboveGround,
           "Points lower than this height [m] above the local ground are not exported, so the "
           "satellite imagery remains visible. Use 0 to keep all points.")
        ->capture_default_str();
    cmd.add_option(
           "--ground-cell", argGroundCell,
           "Cell size [m] of the grid used to estimate the local ground height.")
        ->capture_default_str();
    cmd.add_option(
        "--color-max-height", argColorMaxHeight,
        "Height above ground [m] mapped to the top of the color scale. Default (0): the 98% "
        "percentile of point heights.");
    cmd.add_option(
        "--overlay-resolution", argOverlayResolution,
        "Ground overlay resolution [m/pixel]. Default (0): automatic, the finest one (>=5 cm) "
        "keeping the image under 2048 pixels.");
    cmd.add_option(
           "--model-max-points", argModelMaxPoints,
           "Maximum number of points in the 3D model. The cloud is voxel-decimated to fit.")
        ->capture_default_str();
    cmd.add_option(
        "--model-point-size", argModelPointSize,
        "Size [m] of each point in the 3D model. Default (0): from the decimation voxel size.");
    cmd.add_option(
           "--model-altitude-offset", argModelAltitudeOffset,
           "Extra altitude [m] for the 3D model, in case it does not sit on the terrain.")
        ->capture_default_str();
    cmd.add_option("--heading", argHeading, "Heading [deg] of the initial view.")
        ->capture_default_str();
    cmd.add_option(
        "--tour-duration", argTourDuration,
        "If >0, adds a 'Fly-by' tour [s] orbiting around the map. In Google Earth Pro, it can be "
        "recorded as a video with Tools > Movie Maker.");
    cmd.add_option("--tour-tilt", argTourTilt, "Camera tilt [deg] for the fly-by tour.")
        ->capture_default_str();
    cmd.add_option(
        "--load-plugins", arg_plugins,
        "One or more (comma separated) *.so files to load as plugins");

    CLI11_PARSE(cmd, argc, argv);

    try
    {
        run_mm2kml();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
