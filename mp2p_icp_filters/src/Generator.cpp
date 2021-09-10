/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Generator.cpp
 * @brief  Base virtual class for point cloud filters
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp_filters/Generator.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(Generator, mrpt::rtti::CObject, mp2p_icp_filters)

using namespace mp2p_icp_filters;

Generator::Generator() : mrpt::system::COutputLogger("Generator") {}

mp2p_icp::pointcloud_t Generator::generate_point_cloud(
    const mrpt::obs::CObservation::Ptr& o)
{
    MRPT_START
    using namespace mrpt::obs;

    ASSERT_(o);

    bool processed = false;

    mp2p_icp::pointcloud_t out;

    if (auto o0 = mrpt::ptr_cast<CObservationPointCloud>::from(o); o0)
    {
        ASSERT_(o0->pointcloud);
        processed = filterPointCloud(*o0->pointcloud, out);
    }
    else if (auto o1 = mrpt::ptr_cast<CObservation2DRangeScan>::from(o); o1)
        processed = filterScan2D(*o1, out);
    else if (auto o2 = mrpt::ptr_cast<CObservation3DRangeScan>::from(o); o2)
        processed = filterScan3D(*o2, out);
    else if (auto o3 = mrpt::ptr_cast<CObservationVelodyneScan>::from(o); o3)
        processed = filterVelodyneScan(*o3, out);
    else
    {
        THROW_EXCEPTION_FMT(
            "Unhandled observation type: `%s`",
            o->GetRuntimeClass()->className);
    }

    // done?
    if (!processed)
    {
        mrpt::maps::CSimplePointsMap pc;
        if (!o->insertObservationInto(&pc))
        {
            THROW_EXCEPTION_FMT(
                "Observation of type '%s' could not be converted into a point "
                "cloud, and none of the specializations handled it, so I do "
                "not know what to do with this observation!",
                o->GetRuntimeClass()->className);
        }
    }

    return out;

    MRPT_END
}

bool Generator::filterScan2D(  //
    [[maybe_unused]] const mrpt::obs::CObservation2DRangeScan& pc,
    [[maybe_unused]] mp2p_icp::pointcloud_t&                   out)
{
    return false;  // Not implemented
}

bool Generator::filterVelodyneScan(  //
    [[maybe_unused]] const mrpt::obs::CObservationVelodyneScan& pc,
    [[maybe_unused]] mp2p_icp::pointcloud_t&                    out)
{
    return false;  // Not implemented
}

bool Generator::filterScan3D(  //
    [[maybe_unused]] const mrpt::obs::CObservation3DRangeScan& pc,
    [[maybe_unused]] mp2p_icp::pointcloud_t&                   out)
{
    return false;  // Not implemented
}

bool Generator::filterPointCloud(  //
    const mrpt::maps::CPointsMap& pc, mp2p_icp::pointcloud_t& out)
{
    auto& outPc = out.point_layers[mp2p_icp::pointcloud_t::PT_LAYER_RAW] =
        mrpt::maps::CSimplePointsMap::Create();

    outPc->insertAnotherMap(&pc, mrpt::poses::CPose3D::Identity());

    return true;
}

bool Generator::filterRotatingScan(
    const mrpt::obs::CObservationRotatingScan& pc, mp2p_icp::pointcloud_t& out)
{
    return false;  // Not implemented
}
