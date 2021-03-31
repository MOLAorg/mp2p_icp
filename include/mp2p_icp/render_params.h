/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   render_params.h
 * @brief  Render parameters for the different geometric entities
 * @author Jose Luis Blanco Claraco
 * @date   March 26, 2021
 */
#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/opengl/opengl_frwds.h>

#include <optional>
#include <vector>

namespace mp2p_icp
{
/** \addtogroup mp2p_icp_grp
 * @{
 */

/** Used in pointcloud_t::get_visualization() */
struct render_params_planes_t
{
    render_params_planes_t() = default;

    bool              visible     = true;
    double            halfWidth   = 1.0;
    double            gridSpacing = 0.25;
    mrpt::img::TColor color{0xff, 0xff, 0xff, 0xff};
};

/** Used in pointcloud_t::get_visualization() */
struct render_params_lines_t
{
    render_params_lines_t() = default;

    bool              visible = true;
    mrpt::img::TColor color{0xff, 0x00, 0x00, 0xff};
    double            length = 20.0;  //!< all lines with same length
    std::optional<std::vector<double>> lengths;  //!< individual lengths
};

/** Used in pointcloud_t::get_visualization() */
struct render_params_points_t
{
    render_params_points_t() = default;

    bool visible = true;
};

/** Used in pointcloud_t::get_visualization() */
struct render_params_t
{
    render_params_t() = default;

    render_params_planes_t planes;
    render_params_lines_t  lines;
    render_params_points_t points;
};

/** @} */

}  // namespace mp2p_icp
