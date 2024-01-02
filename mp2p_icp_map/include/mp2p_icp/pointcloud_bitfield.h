/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   pointcloud_bitfield.h
 * @brief  A bit field with a bool for each metric_map_t entity.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */
#pragma once

#include <mp2p_icp/layer_name_t.h>
#include <mp2p_icp/metricmap.h>

#include <cstdint>
#include <cstdlib>
#include <map>
#include <optional>
#include <set>
#include <vector>

namespace mp2p_icp
{
/** \addtogroup mp2p_icp_map_grp
 * @{
 */

/** A bit field with a bool for each metric_map_t entity.
 *  Useful, for example, to keep track of which elements have already been
 * matched during the matching pipeline.
 */
struct pointcloud_bitfield_t
{
    pointcloud_bitfield_t()  = default;
    ~pointcloud_bitfield_t() = default;

    struct DenseOrSparseBitField
    {
       public:
        DenseOrSparseBitField()  = default;
        ~DenseOrSparseBitField() = default;

        void assign(size_t numElements, bool dense)
        {
            if (dense)
            {
                sparse_.clear();
                if (!dense_) dense_.emplace();
                dense_->assign(numElements, false);
            }
            else
            {
                dense_.reset();
                sparse_.clear();
            }
        }

        [[nodiscard]] bool operator[](const size_t id) const
        {
            if (dense_.has_value())
                return dense_.value()[id];
            else
                return sparse_.count(id) != 0;
        }
        void mark_as_set(const size_t id)
        {
            if (dense_.has_value())
                dense_.value()[id] = true;
            else
                sparse_.insert(id);
        }

       private:
        std::optional<std::vector<bool>> dense_;
        std::set<uint64_t>               sparse_;
    };

    /** @name Data fields
     * @{ */
    std::map<layer_name_t, DenseOrSparseBitField> point_layers;
    std::vector<bool>                             lines;
    std::vector<bool>                             planes;
    /** @} */

    void initialize_from(const metric_map_t& pc)
    {
        // Points:
        // Done in this way to avoid avoidable memory reallocations.
        for (const auto& kv : pc.layers)
        {
            ASSERT_(kv.second);
            auto* nn = mp2p_icp::MapToNN(*kv.second, false /*dont throw*/);
            if (!nn) continue;
            point_layers[kv.first].assign(
                nn->nn_index_count(), nn->nn_has_indices_or_ids());
        }
        std::set<layer_name_t> layersToRemove;
        for (auto& kv : point_layers)
        {
            if (pc.layers.count(kv.first) == 0) layersToRemove.insert(kv.first);
        }
        for (const auto& ly : layersToRemove) point_layers.erase(ly);

        // Lines:
        lines.assign(pc.lines.size(), false);

        // planes:
        planes.assign(pc.planes.size(), false);
    }
};

/** @} */

}  // namespace mp2p_icp
