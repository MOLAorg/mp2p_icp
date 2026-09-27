/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mm-viewer/CameraTravelling.h
 * @brief  Camera keyframes and their interpolation, for fly-by animations
 */
#pragma once

#include <map>
#include <string>

namespace mm_viewer
{
/** One orbit-camera state, as used by mrpt::viz::COrbitCameraController. */
struct CameraKeyframe
{
    double x            = 0;  //!< Point the camera looks at [m]
    double y            = 0;
    double z            = 0;
    double azimuthDeg   = 0;
    double elevationDeg = 0;
    double zoom         = 1;  //!< Distance from the camera to (x,y,z) [m], must be > 0
};

enum class TravellingInterpolation
{
    Linear = 0,
    /** Cubic Catmull-Rom: passes through every keyframe, smooth velocity. */
    Spline
};

/** Keyframes indexed by their time [s]. */
using CameraPath = std::map<double, CameraKeyframe>;

/** Camera state at time `t`, clamped to the path time range.
 *  Azimuth takes the shortest way around, and zoom is interpolated in log scale so its relative
 *  rate of change stays uniform. The path must not be empty. */
CameraKeyframe interpolateCameraPath(
    const CameraPath& path, double t, TravellingInterpolation method);

/** Saves as a text file, one "t x y z azimuth_deg elevation_deg zoom" line per keyframe.
 *  Returns false and fills `errorMsg` on error. */
bool saveCameraPath(const CameraPath& path, const std::string& file, std::string& errorMsg);

/** Loads a file written by saveCameraPath(). Lines starting with '#' are ignored.
 *  Returns false and fills `errorMsg` on error, leaving `path` untouched. */
bool loadCameraPath(CameraPath& path, const std::string& file, std::string& errorMsg);

}  // namespace mm_viewer
