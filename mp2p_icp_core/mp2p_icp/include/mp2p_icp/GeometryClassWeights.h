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
 * @file   GeometryClassWeights.h
 * @brief  Extra per-pairing weight as a function of the map surface geometry.
 * @author Jose Luis Blanco Claraco
 * @date   Aug 21, 2026
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose3D.h>

#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

/** An extra multiplicative weight for each cov-to-cov pairing, depending only
 *  on a coarse *geometry class* of the map-side surface it landed on:
 *
 *  \f[ w_i \rightarrow w_i \cdot \lambda(\text{class}_i) \f]
 *
 *  This makes the registration behave differently on ground than on walls, so
 *  it is **opt-in, disabled by default, and has no universal setting**: a
 *  weight fitted in one scene class does not transfer to another, so the values
 *  belong to a particular platform-and-environment configuration and must be
 *  chosen deliberately. A vector of all ones is bit-exactly inert.
 *
 *  That last point is measured, not assumed. A weight fitted on one dataset
 *  family and applied unchanged to three others produced no coherent effect at
 *  all -- improving 2 of 12 sequences, with per-sequence ratios spanning two
 *  orders of magnitude -- while on its own family it improved 12 of 13
 *  consistently. **Enabling this with someone else's numbers therefore spends
 *  observability for no bias reduction.** It is not a neutral default.
 *
 *  Two class variables are offered and they are not equivalent:
 *
 *  - `Incidence` (default): the angle between the ray and the surface normal.
 *    Sensor-relative, needs no world vertical, and therefore cannot couple the
 *    registration to the attitude estimate.
 *  - `Verticality`: \f$ |n \cdot up| \f$. This is gravity-referenced, so it
 *    creates a feedback path -- attitude error tilts the map, which reassigns
 *    surfaces near the class edges, which changes the weights, which changes
 *    the attitude error. It requires an explicit gravity source and refuses to
 *    run without one rather than silently using the map frame.
 *
 *  Class membership is a smooth partition of unity, not a hard bin: a threshold
 *  on a noisy estimated quantity flips a whole band of surfaces at once when
 *  the estimate moves, and correspondences move between ICP iterations. Set
 *  `softness` to 0 to recover hard bins.
 *
 *  \ingroup mp2p_icp_grp
 */
struct GeometryClassWeights
{
    GeometryClassWeights() = default;

    enum class Variable : uint8_t
    {
        Incidence = 0,  //!< angle(ray, normal) [deg]; gravity-free
        Verticality  //!< |n . up|; needs a gravity source
    };

    enum class GravitySource : uint8_t
    {
        Imu = 0,  //!< from the solver's gravity observation; throws if absent
        MapFrame  //!< the map frame's own +Z. Discouraged: it drifts.
    };

    /** Explicit kill switch, honored before anything else. */
    bool enabled = false;

    Variable variable = Variable::Incidence;

    /** Class edges, ascending: degrees for `Incidence`, \f$ |n \cdot up| \f$ in
     *  [0,1] for `Verticality`. Must be strictly increasing and separated by at
     *  least `softness`, otherwise the memberships would not form a partition
     *  of unity. */
    std::vector<double> breakpoints;

    /** One more entry than `breakpoints`. All ones == disabled. Only the
     *  *ratios* matter: `dx = -H^-1 g` is invariant to a global scale, so
     *  rescaling the whole vector changes nothing. */
    std::vector<double> weights;

    /** Width of the ramp at each breakpoint, in the units of `breakpoints`.
     *  0 gives hard bins. Should be at least the angular error of the normal
     *  estimate: a gate on an estimated quantity is sized by that estimate's
     *  error. */
    double softness = 5.0;

    GravitySource gravitySource = GravitySource::Imu;

    void load_from(const mrpt::containers::yaml& p);
    void save_to(mrpt::containers::yaml& p) const;

    /** Throws if the configuration cannot be evaluated as written. */
    void validate() const;

    /** False when this can be skipped entirely: disabled, empty, or the
     *  identity. Callers use it to keep the untouched path bit-exact. */
    bool active() const;

    /** The weight for one pairing.
     *
     * @param normal   Map-side surface normal, in the MAP frame.
     * @param local    Scan point, in the vehicle frame (the pairing's `local`).
     * @param pose     Current pose of the scan in the map frame; this is what
     *                 rotates the ray into the map frame. Contracting the two
     *                 without it yields incidence mixed with vehicle yaw.
     * @param upInMap  Required for `Verticality`, ignored for `Incidence`.
     */
    double operator()(
        const Eigen::Vector3d& normal, const mrpt::math::TPoint3Df& local,
        const mrpt::poses::CPose3D&           pose,
        const std::optional<Eigen::Vector3d>& upInMap = std::nullopt) const;

    /** The class variable's value for one pairing, in the units of
     *  `breakpoints`. Exposed for tests and for the diagnostics. */
    double variable_value(
        const Eigen::Vector3d& normal, const mrpt::math::TPoint3Df& local,
        const mrpt::poses::CPose3D&           pose,
        const std::optional<Eigen::Vector3d>& upInMap = std::nullopt) const;

    /** Membership of `x` in each class; sums to 1 by construction. */
    void memberships(double x, std::vector<double>& out) const;
};

/** @} */

}  // namespace mp2p_icp
