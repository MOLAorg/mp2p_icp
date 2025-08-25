/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   pt2ln_pl_to_pt2pt.cpp
 * @brief  Converter of pairings.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 15, 2021
 */

#include <mp2p_icp/pt2ln_pl_to_pt2pt.h>

using namespace mp2p_icp;

static void append_from_sorted(
    const std::multimap<double, mrpt::tfest::TMatchingPair>& sorted, Pairings& out,
    const double ratio)
{
    // Heuristic: only add those pairings with a given % of the largest error,
    // since the smallest ones are already satisfied and may constraint the
    // solution too much:
    if (sorted.empty()) return;

    const double largestError = sorted.rbegin()->first;
    const double threshold    = largestError * ratio;

    for (auto it = sorted.rbegin(); it != sorted.rend(); ++it)
    {
        const double err = it->first;
        // skip the rest... except if we don't have correspondences
        if (err < threshold && out.paired_pt2pt.size() >= 3) break;
        out.paired_pt2pt.push_back(it->second);
    }
}

// See .h for docs
Pairings mp2p_icp::pt2ln_pl_to_pt2pt(const Pairings& in, const SolverContext& sc)
{
    Pairings out;

    ASSERT_(sc.guessRelativePose.has_value());
    const auto& relPose = *sc.guessRelativePose;

    const double                                      ratio = 0.25;
    std::multimap<double, mrpt::tfest::TMatchingPair> sortedOut;

    // ===========================================================
    // point-to-plane => point to closest point on plane
    // ===========================================================
    for (const auto& p : in.paired_pt2pl)
    {
        const auto  pt_g = relPose.composePoint(p.pt_local);
        const auto& pl   = p.pl_global.plane;

        const auto plNormal = pl.getNormalVector();

        ASSERT_NEAR_(plNormal.norm(), 1.0, 1e-4);

        // project
        const double d = pl.evaluatePoint(pt_g);

        const mrpt::math::TPoint3D c = pt_g - (plNormal * d);

        mrpt::tfest::TMatchingPair new_p;

        new_p.globalIdx = 0;  // dummy
        new_p.localIdx  = 0;

        new_p.global = c;
        new_p.local  = p.pt_local;

        sortedOut.insert({std::abs(d), new_p});
    }
    out.paired_pt2pl.clear();
    append_from_sorted(sortedOut, out, ratio);
    sortedOut.clear();

    // ===========================================================
    // point-to-plane => point to closest point on line
    // ===========================================================
    for (const auto& p : in.paired_pt2ln)
    {
        const auto  pt_g = relPose.composePoint(p.pt_local);
        const auto& ln   = p.ln_global;

        const mrpt::math::TPoint3D c = ln.closestPointTo(pt_g);
        const double               d = (c - pt_g).norm();

        mrpt::tfest::TMatchingPair new_p;

        new_p.globalIdx = 0;  // dummy
        new_p.localIdx  = 0;

        new_p.global = c;
        new_p.local  = p.pt_local;

        sortedOut.insert({std::abs(d), new_p});
    }
    out.paired_pt2ln.clear();
    append_from_sorted(sortedOut, out, ratio);

    return out;
}
