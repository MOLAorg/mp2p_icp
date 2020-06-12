/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mp2p_icp/Parameters.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

IMPLEMENTS_MRPT_OBJECT(Parameters, mrpt::serialization::CSerializable, mp2p_icp)

using namespace mp2p_icp;

// Implementation of the CSerializable virtual interface:
uint8_t Parameters::serializeGetVersion() const { return 0; }
void    Parameters::serializeTo(mrpt::serialization::CArchive& out) const
{
    out << maxIterations << maxPairsPerLayer << minAbsStep_trans
        << minAbsStep_rot << thresholdDist << thresholdAng
        << thresholdPlane2PlaneNormalAng << use_kernel
        << use_scale_outlier_detector << scale_outlier_threshold
        << relative_weight_planes_attitude << weight_pt2pt_layers
        << pt2pl_layer;
}
void Parameters::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        {
            in >> maxIterations >> maxPairsPerLayer >> minAbsStep_trans >>
                minAbsStep_rot >> thresholdDist >> thresholdAng >>
                thresholdPlane2PlaneNormalAng >> use_kernel >>
                use_scale_outlier_detector >> scale_outlier_threshold >>
                relative_weight_planes_attitude >> weight_pt2pt_layers >>
                pt2pl_layer;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}
