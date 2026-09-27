/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include "CameraTravelling.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

namespace mm_viewer
{
namespace
{
constexpr size_t NUM_COMPONENTS = 6;
using Vec                       = std::array<double, NUM_COMPONENTS>;

// Minimum zoom distance, to keep the log() well defined.
constexpr double MIN_ZOOM = 1e-6;

Vec toVec(const CameraKeyframe& k)
{
    return {k.x, k.y, k.z, k.azimuthDeg, k.elevationDeg, std::log(std::max(k.zoom, MIN_ZOOM))};
}

CameraKeyframe fromVec(const Vec& v)
{
    CameraKeyframe k;
    k.x            = v[0];
    k.y            = v[1];
    k.z            = v[2];
    k.azimuthDeg   = std::remainder(v[3], 360.0);
    k.elevationDeg = std::clamp(v[4], -90.0, 90.0);
    k.zoom         = std::exp(v[5]);
    return k;
}
}  // namespace

CameraKeyframe interpolateCameraPath(
    const CameraPath& path, double t, TravellingInterpolation method)
{
    const size_t n = path.size();

    std::vector<double> times;
    std::vector<Vec>    values;
    times.reserve(n);
    values.reserve(n);
    for (const auto& [kt, kf] : path)
    {
        Vec v = toVec(kf);
        if (!values.empty())
        {
            // Unwrap azimuth, so each step takes the shortest way around:
            const double prevAz = values.back()[3];
            v[3]                = prevAz + std::remainder(v[3] - prevAz, 360.0);
        }
        times.push_back(kt);
        values.push_back(v);
    }

    if (n == 1 || t <= times.front())
    {
        return path.begin()->second;
    }
    if (t >= times.back())
    {
        return path.rbegin()->second;
    }

    // Segment [k, k+1] containing t:
    const size_t k = static_cast<size_t>(
        std::distance(times.begin(), std::upper_bound(times.begin(), times.end(), t)) - 1);

    const double dt = times[k + 1] - times[k];
    const double s  = (t - times[k]) / dt;

    Vec out{};
    if (method == TravellingInterpolation::Linear)
    {
        for (size_t c = 0; c < NUM_COMPONENTS; c++)
        {
            out[c] = (1.0 - s) * values[k][c] + s * values[k + 1][c];
        }
        return fromVec(out);
    }

    // Catmull-Rom tangents (for non-uniform keyframe times), one-sided at both path ends:
    const auto tangent = [&](size_t i, size_t c)
    {
        const size_t i0 = (i == 0) ? 0 : i - 1;
        const size_t i1 = (i == n - 1) ? n - 1 : i + 1;
        return (values[i1][c] - values[i0][c]) / (times[i1] - times[i0]);
    };

    // Cubic Hermite basis:
    const double s2  = s * s;
    const double s3  = s2 * s;
    const double h00 = 2 * s3 - 3 * s2 + 1;
    const double h10 = s3 - 2 * s2 + s;
    const double h01 = -2 * s3 + 3 * s2;
    const double h11 = s3 - s2;

    for (size_t c = 0; c < NUM_COMPONENTS; c++)
    {
        out[c] = h00 * values[k][c] + h10 * dt * tangent(k, c) + h01 * values[k + 1][c] +
                 h11 * dt * tangent(k + 1, c);
    }
    return fromVec(out);
}

bool saveCameraPath(const CameraPath& path, const std::string& file, std::string& errorMsg)
{
    std::ofstream f(file);
    if (!f.is_open())
    {
        errorMsg = "Cannot write to file: " + file;
        return false;
    }
    f << "# mm-viewer camera path\n# t[s] x y z azimuth[deg] elevation[deg] zoom[m]\n";
    f << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& [t, k] : path)
    {
        f << t << " " << k.x << " " << k.y << " " << k.z << " " << k.azimuthDeg << " "
          << k.elevationDeg << " " << k.zoom << "\n";
    }
    if (!f.good())
    {
        errorMsg = "Error writing file: " + file;
        return false;
    }
    return true;
}

bool loadCameraPath(CameraPath& path, const std::string& file, std::string& errorMsg)
{
    std::ifstream f(file);
    if (!f.is_open())
    {
        errorMsg = "Cannot open file: " + file;
        return false;
    }

    CameraPath  loaded;
    std::string line;
    size_t      lineNum = 0;
    while (std::getline(f, line))
    {
        lineNum++;
        const auto firstNonBlank = line.find_first_not_of(" \t\r");
        if (firstNonBlank == std::string::npos || line[firstNonBlank] == '#')
        {
            continue;
        }

        std::istringstream iss(line);
        double             t = 0;
        CameraKeyframe     k;
        if (!(iss >> t >> k.x >> k.y >> k.z >> k.azimuthDeg >> k.elevationDeg >> k.zoom))
        {
            errorMsg = "Malformed line " + std::to_string(lineNum) + " in file: " + file;
            return false;
        }
        if (k.zoom <= 0)
        {
            errorMsg =
                "Zoom must be positive, in line " + std::to_string(lineNum) + " in file: " + file;
            return false;
        }
        loaded[t] = k;
    }

    path = std::move(loaded);
    return true;
}

}  // namespace mm_viewer
