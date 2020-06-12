/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/core/bits_math.h>  // DEG2RAD()
#include <mrpt/serialization/CSerializable.h>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace mp2p_icp
{
/** ICP parameters.
 * \sa ICP_Base
 * \ingroup mp2p_icp_grp
 */
struct Parameters : public mrpt::serialization::CSerializable
{
    DEFINE_SERIALIZABLE(Parameters, mp2p_icp)

   public:
    /** @name Termination criteria
        @{ */
    /** Maximum number of iterations to run. */
    uint32_t maxIterations{40};

    /** Max. number of pairings per layer (point-to-point, plane-to-plane...).
     * Decimation of the point cloud being registered against the reference
     * one. The speed-up comes from a decimation of the number of KD-tree
     * queries, the most expensive step in ICP
     */
    uint32_t maxPairsPerLayer{500};

    /** If the correction in all translation coordinates (X,Y,Z) is below
     * this threshold (in meters), iterations are terminated (Default:1e-6)
     */
    double minAbsStep_trans{5e-4};
    /** If the correction in all rotation coordinates (yaw,pitch,roll) is
     * below this threshold (in radians), iterations are terminated
     * (Default:1e-6) */
    double minAbsStep_rot{1e-4};
    /** @} */

    /** Treshold distance for pair two near points */
    double thresholdDist{0.75}, thresholdAng{mrpt::DEG2RAD(0.15)};

    /** Maximum angle (radians) between potential matching plane normals to be
     * accepted as a pairing. */
    double thresholdPlane2PlaneNormalAng{mrpt::DEG2RAD(5.0)};

    /** Whether to use kernel_rho to smooth distances, or use distances
     * directly (default=true) */
    bool use_kernel{false};

    /** Enables the use of the scale-based outlier detector. Refer to the
     * technical report.  This robustness feature is independent from
     * use_robust_kernel.
     */
    bool use_scale_outlier_detector{true};

    /** If use_scale_outlier_detector==true, discard a potential point-to-point
     * pairing if the ratio between the norm of their final vectors is larger
     * than this value. A value of "1.0" will only allow numerically perfect
     * pairings, so a slightly larger value is required. The closer to 1, the
     * stricter. A much larger threshold (e.g. 5.0) would only reject the
     * most obvious outliers. Refer to the technical report. */
    double scale_outlier_threshold{1.20};

    double relative_weight_planes_attitude{1.0};

    /** Weight for each layer. Those not present here  */
    std::map<std::string, double> weight_pt2pt_layers;

    /** [Only used by ICP_GaussNewton] The name of a layer of points
     * to be paired individually to planes (pointcloud_t::planes).
     */
    std::string pt2pl_layer;
};

}  // namespace mp2p_icp
