/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Matcher_Point2Line.h
 * @brief  Pointcloud matcher: point to line-fit of nearby points
 * @author Jose Luis Blanco Claraco
 * @date   Nov 29, 2021
 */
#pragma once

#include <mp2p_icp/Matcher_Points_Base.h>

namespace mp2p_icp
{
/** Pointcloud matcher: point to plane-fit of nearby points
 *
 * Finds point-to-line pairings between `local` point layers and points fitting
 * a line in layers of the `global` input metric map.
 *
 * By default, each `local` point layer is matched against the layer with the
 * same name in the `global` map, unless specified otherwise in the base class
 * member `weight_pt2pt_layers`. Refer to example configuration YAML files for
 * example configurations.
 *
 * \ingroup mp2p_icp_grp
 */
class Matcher_Point2Line : public Matcher_Points_Base
{
    DEFINE_MRPT_OBJECT(Matcher_Point2Line, mp2p_icp)

   public:
    Matcher_Point2Line();

    /** Parameters:
     * - `distanceThreshold`: Inliers distance threshold [meters][mandatory]
     * - `knn`: Number of neighbors to look for [mandatory]
     * - `minimumLinePoints`: Minimum number of found points [mandatory]
     * - `lineEigenThreshold`: maximum e0/e2 and e1/e2 ratio [mandatory]
     *
     * Where [e0, e1, e2] are the smallest to largest eigenvalues of the
     * Gaussian covariance fitting the knn closest global points for each local
     * point.
     *
     * Plus: the parameters of Matcher_Points_Base::initialize()
     */
    void initialize(const mrpt::containers::yaml& params) override;

   private:
    double   distanceThreshold  = 0.50;
    uint32_t knn                = 4;
    uint32_t minimumLinePoints  = 4;
    double   lineEigenThreshold = 0.01;

    void implMatchOneLayer(
        const mrpt::maps::CMetricMap& pcGlobal, const mrpt::maps::CPointsMap& pcLocal,
        const mrpt::poses::CPose3D& localPose, MatchState& ms, const layer_name_t& globalName,
        const layer_name_t& localName, Pairings& out) const override;
};

}  // namespace mp2p_icp
