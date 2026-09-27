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

// Degrees to radians (M_PI is not standard C++):
constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;

/** Offset from the point looked at to the camera eye, i.e. eye = (x,y,z) + offset. */
std::array<double, 3> eyeOffset(const Vec& v)
{
    const double az   = v[3] * DEG2RAD;
    const double el   = v[4] * DEG2RAD;
    const double dist = std::exp(v[5]);
    return {
        dist * std::cos(az) * std::cos(el), dist * std::sin(az) * std::cos(el),
        dist * std::sin(el)};
}

/** Replaces the point looked at in v[0:2] by the point at fraction `beta` of the way from it
 *  to the eye. */
Vec toPivot(Vec v, double beta)
{
    const auto off = eyeOffset(v);
    for (size_t c = 0; c < 3; c++)
    {
        v[c] += beta * off[c];
    }
    return v;
}

/** Inverse of toPivot(). */
Vec fromPivot(Vec v, double beta)
{
    const auto off = eyeOffset(v);
    for (size_t c = 0; c < 3; c++)
    {
        v[c] -= beta * off[c];
    }
    return v;
}

/** Pivot fraction (0: point looked at, 1: eye) to interpolate between keyframes a and b: whichever
 *  of both points moves the least. This keeps fixed whatever point the user kept fixed while moving
 *  the camera between both keyframes: the point looked at while orbiting or zooming, the eye while
 *  looking around. */
double pivotFraction(const Vec& a, const Vec& b)
{
    const auto offA      = eyeOffset(a);
    const auto offB      = eyeOffset(b);
    double     targetSqr = 0;
    double     eyeSqr    = 0;
    for (size_t c = 0; c < 3; c++)
    {
        const double dTarget = b[c] - a[c];
        const double dEye    = dTarget + offB[c] - offA[c];
        targetSqr += dTarget * dTarget;
        eyeSqr += dEye * dEye;
    }
    return eyeSqr < targetSqr ? 1.0 : 0.0;
}

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

    // Interpolate the point (eye or looked at) that moves the least along this segment, then
    // recover the point looked at from it and the interpolated view direction:
    const double beta = pivotFraction(values[k], values[k + 1]);
    for (size_t i = (k == 0 ? 0 : k - 1); i <= std::min(k + 2, n - 1); i++)
    {
        values[i] = toPivot(values[i], beta);
    }

    Vec out{};
    if (method == TravellingInterpolation::Linear)
    {
        for (size_t c = 0; c < NUM_COMPONENTS; c++)
        {
            out[c] = (1.0 - s) * values[k][c] + s * values[k + 1][c];
        }
        return fromVec(fromPivot(out, beta));
    }

    // Catmull-Rom tangents (for non-uniform keyframe times), one-sided at both path ends.
    // Only the pivot values of keyframes k-1..k+2 are valid here, which is all they read:
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
    return fromVec(fromPivot(out, beta));
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
