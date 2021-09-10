/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Generator.h
 * @brief  Base virtual class for point cloud filters
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#pragma once

#include <mp2p_icp/pointcloud.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

#include <cstdint>
#include <stdexcept>

namespace mp2p_icp_filters
{
/** Used in Generator
 * \ingroup mp2p_icp_filters_grp
 */
struct NotImplementedError : public std::runtime_error
{
    // NotImplementedError() = default;
    template <typename T>
    NotImplementedError(T v) : std::runtime_error(v)
    {
    }
};

/** Generic base class providing generate_point_cloud(), converting a generic
 *  mrpt::obs::CObservation into a pointcloud_t with just a point cloud layer (named `raw`).
 *
 * This pointcloud-t can optionally then be passed through one or more FilterBase
 * implementations to detect features, decimate it, etc.
 *
 * Specializations of Generator may exist and could be implemented to exploit
 * the structured information in the original CObservation data to be more
 * efficient in detecting features (e.g. corners, etc.).

 * Different signatures exists for:
 * - 2D LiDAR range scans (mrpt::obs::CObservation2DRangeScan)
 * - 3D Velodyne scans (mrpt::obs::CObservationVelodyneScan)
 * - 3D RGBD camera images (mrpt::obs::CObservation3DRangeScan)
 * - Generic 2D/3D point clouds (mrpt::obs::CObservationPointCloud)
 * - Generic multi-beam rotating (or flash) Lidars (mrpt::obs::CObservationRotatingScan)
 *
 * Derived classes may implement all or only one of those methods. An
 * exception NotImplementedError will be thrown if an non-implemented method is
 * called.
 *
 * \note generate_point_cloud() is not required to be thread (multientry) safe.
 *
 * \sa Implementation in FilterEdgesPlanes
 *
 * \ingroup mp2p_icp_filters_grp
 */
class Generator : public mrpt::rtti::CObject,  // RTTI support
                  public mrpt::system::COutputLogger  // Logging support
{
    DEFINE_VIRTUAL_MRPT_OBJECT(Generator)

   public:
    Generator();

    /** \name API for all filtering/segmentation algorithms
     *  @{ */

    /** Loads, from a YAML configuration block, all the common, and
     * implementation-specific parameters. */
    virtual void initialize(const std::string& cfg_block) = 0;

    /** See docs above for Generator.
     * This method dispatches the observation by type to the corresponding
     * virtual method
     */
    virtual mp2p_icp::pointcloud_t generate_point_cloud(
        const mrpt::obs::CObservation::Ptr& input_raw);

    /** @} */

   protected:
    // To be overrided in derived classes, if implemented:
    /** Process a 2D lidar scan. \return false if not implemented */
    virtual bool filterScan2D(
        const mrpt::obs::CObservation2DRangeScan& pc,
        mp2p_icp::pointcloud_t&                   out);
    /** Process a depth camera observation. \return false if not implemented */
    virtual bool filterScan3D(
        const mrpt::obs::CObservation3DRangeScan& pc,
        mp2p_icp::pointcloud_t&                   out);
    /** Process a 3D lidar scan. \return false if not implemented   */
    virtual bool filterVelodyneScan(
        const mrpt::obs::CObservationVelodyneScan& pc,
        mp2p_icp::pointcloud_t&                    out);
    /** Process a 2D/3D point-cloud. \return false if not implemented  */
    virtual bool filterPointCloud(
        const mrpt::maps::CPointsMap& pc, mp2p_icp::pointcloud_t& out);
    /** Process a 3D lidar scan. \return false if not implemented   */
    virtual bool filterRotatingScan(
        const mrpt::obs::CObservationRotatingScan& pc,
        mp2p_icp::pointcloud_t&                    out);
};

}  // namespace mp2p_icp_filters
