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
 * @file   pointcloud_field_utils.h
 * @brief  Raises warnings on missing fields that would be filled with zeros.
 * @author Jose Luis Blanco Claraco
 * @date   Apr 9, 2026
 */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/COutputLogger.h>

#include <string>
#include <vector>

namespace mp2p_icp
{

/** Checks whether copying points from \a src into \a dst via
 *  CPointsMap::prepareForInsertPointsFrom() + insertPointFrom() would result
 *  in any destination field being zero-padded (i.e. the field exists in \a dst
 *  but is absent in \a src).
 *
 *  This situation is benign on MRPT < 2.15.13 (those fields were simply
 *  skipped, which caused sanity-check crashes), and is
 *  handled correctly from MRPT 2.15.13 onwards by writing explicit zeros.
 *  However, silent zero-padding may still indicate a configuration problem,
 *  or sensor drops.
 *
 *  One WARN-level message is emitted per mismatched field name.
 *
 *  \param src  Source point cloud (the one being inserted FROM).
 *  \param dst  Destination point cloud (the one being inserted INTO).
 *  \param logger  Logger used for WARN messages.
 *  \return True if at least one mismatch was found.
 */
bool warn_on_field_padding_mismatch(
    const mrpt::maps::CPointsMap& src, const mrpt::maps::CPointsMap& dst,
    const mrpt::system::COutputLogger& logger);

/** Rotates the per-point view-direction unit vector fields
 *  (`view_x`, `view_y`, `view_z`) of \a pts in place by the rotation part of
 *  \a tf, leaving the point coordinates untouched.
 *
 *  This must be called whenever a point cloud carrying these fields is
 *  re-expressed in a different frame of reference (e.g. via
 *  `CPointsMap::insertAnotherMap()`), since that operation transforms the XYZ
 *  coordinates but copies all other registered fields verbatim, which would
 *  otherwise leave the view vectors expressed in the wrong frame.
 *
 *  If \a pts does not have all three fields, or \a tf is the identity
 *  transformation, this is a no-op.
 *
 *  \param pts  Point cloud to modify in place.
 *  \param tf   Transformation whose rotation part is applied to the view
 *              vectors.
 */
void rotateViewDirectionFields(mrpt::maps::CPointsMap& pts, const mrpt::poses::CPose3D& tf);

}  // namespace mp2p_icp
