/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Generator.h
 * @brief  Base virtual class for point cloud filters
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#pragma once

#include <mp2p_icp/Parameterizable.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

#include <regex>
#include <stdexcept>

namespace mp2p_icp_filters
{
/** \addtogroup mp2p_icp_filters_grp
 *  @{ */

/** Used in Generator
 */
struct NotImplementedError : public std::runtime_error
{
    // NotImplementedError() = default;
    template <typename T>
    NotImplementedError(T v) : std::runtime_error(v)
    {
    }
};

/** Generic base class providing process(), converting a generic
 *  mrpt::obs::CObservation into a metric_map_t with just a point cloud layer
 *  (named `raw`).
 *
 * This metric_map_t can optionally then be passed through one or more
 * FilterBase implementations to detect features, decimate it, etc.
 *
 * Specializations of Generator may exist and could be implemented to exploit
 * the structured information in the original CObservation data to be more
 * efficient in detecting features (e.g. corners, etc.).

 * Internally, different signatures exists for:
 * - 2D LiDAR range scans (mrpt::obs::CObservation2DRangeScan)
 * - 3D Velodyne scans (mrpt::obs::CObservationVelodyneScan)
 * - 3D RGBD camera images (mrpt::obs::CObservation3DRangeScan)
 * - Generic 2D/3D point clouds (mrpt::obs::CObservationPointCloud)
 * - Generic multi-beam rotating (or flash) Lidars
 *   (mrpt::obs::CObservationRotatingScan)
 *
 * Derived classes may implement all or only one of those methods. An
 * exception NotImplementedError will be thrown if an non-implemented method is
 * called.
 *
 * \note process() is not required to be thread (multientry) safe.
 *
 * A set of generators can be loaded from a YAML file and applied together using
 * mp2p_icp_filters::apply_generators().
 *
 */
class Generator : public mrpt::rtti::CObject,  // RTTI support
                  public mrpt::system::COutputLogger,  // Logging support
                  public mp2p_icp::Parameterizable  // Dynamic parameters
{
    DEFINE_MRPT_OBJECT(Generator, mp2p_icp_filters)

   public:
    Generator();

    /** \name Generator API
     *  @{ */

    /** Loads, from a YAML configuration block, all the common, and
     * implementation-specific parameters.
     * If you redefine this method, remember calling this method on the parent
     * class.
     */
    virtual void initialize(const mrpt::containers::yaml& cfg_block);

    /** See docs above for Generator.
     * This method dispatches the observation by type to the corresponding
     * virtual method
     */
    virtual bool process(
        const mrpt::obs::CObservation& input_raw, mp2p_icp::metric_map_t& inOut,
        const std::optional<mrpt::poses::CPose3D>& robotPose = std::nullopt) const;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, Generator& parent);

        /** The map layer name where the observation will be loaded into.
         *  Default: "raw" */
        std::string target_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** If empty (default), a point cloud layer will be generated of type
         *  mrpt::maps::CSimplePointCloud, and the observation will be inserted
         * via the virtual method mrpt::maps::CMetricMap::insertObservation()
         *
         * Alternatively, if this variable *or* `metric_map_definition` are set,
         * a custom metric map class can be defined via a
         * mrpt::maps::CMultiMetricMap initializer list.
         * If the CMultiMetricMap defines multiple metric maps, the first one
         * only will be taken to generate the new layer.
         *
         * This variable may be set to an external `.ini` file in the format
         * expected by mrpt::maps::TSetOfMetricMapInitializers. The recomended
         * alternative is to directly define the map setting in the YAML file.
         *
         * Refer to [example pipelines](https://docs.mola-slam.org/mp2p_icp/).
         *
         */
        std::string metric_map_definition_ini_file;

        /** Read metric_map_definition_ini_file. Defining a custom metric map
         * via this YAML structure is preferred, since parameterizable variables
         * are supported in this way, while external `.ini` files must contain
         * fixed static values only.
         */
        mrpt::containers::yaml metric_map_definition;

        /** Sensor observation class names to process. Default = ".*" (any).
         *  Example: use "mrpt::obs::CObservation2DRangeScan" if you only want
         * 2D lidar scans to be processed.
         */
        std::string process_class_names_regex = ".*";

        /** Sensor labels to process. Default = ".*" (any).
         *  Examples: "LIDAR", "FRONT_KINECT", "LIDAR_.*"
         */
        std::string process_sensor_labels_regex = ".*";

        bool throw_on_unhandled_observation_class = false;
    };

    Parameters params_;

    /** @} */

   protected:
    // To be overrided in derived classes, if implemented:
    /** Process a 2D lidar scan. \return false if not implemented */
    virtual bool filterScan2D(
        const mrpt::obs::CObservation2DRangeScan& pc, mp2p_icp::metric_map_t& out,
        const std::optional<mrpt::poses::CPose3D>& robotPose) const;
    /** Process a depth camera observation. \return false if not implemented */
    virtual bool filterScan3D(
        const mrpt::obs::CObservation3DRangeScan& pc, mp2p_icp::metric_map_t& out,
        const std::optional<mrpt::poses::CPose3D>& robotPose) const;
    /** Process a 3D lidar scan. \return false if not implemented   */
    virtual bool filterVelodyneScan(
        const mrpt::obs::CObservationVelodyneScan& pc, mp2p_icp::metric_map_t& out,
        const std::optional<mrpt::poses::CPose3D>& robotPose) const;
    /** Process a 2D/3D point-cloud. \return false if not implemented  */
    virtual bool filterPointCloud(
        const mrpt::maps::CPointsMap& pc, const mrpt::poses::CPose3D& sensorPose,
        mp2p_icp::metric_map_t& out, const std::optional<mrpt::poses::CPose3D>& robotPose) const;
    /** Process a 3D lidar scan. \return false if not implemented   */
    virtual bool filterRotatingScan(
        const mrpt::obs::CObservationRotatingScan& pc, mp2p_icp::metric_map_t& out,
        const std::optional<mrpt::poses::CPose3D>& robotPose) const;

    bool       initialized_ = false;
    std::regex process_class_names_regex_;
    std::regex process_sensor_labels_regex_;

   private:
    bool implProcessDefault(
        const mrpt::obs::CObservation& o, mp2p_icp::metric_map_t& out,
        const std::optional<mrpt::poses::CPose3D>& robotPose = std::nullopt) const;

    bool implProcessCustomMap(
        const mrpt::obs::CObservation& o, mp2p_icp::metric_map_t& out,
        const std::optional<mrpt::poses::CPose3D>& robotPose = std::nullopt) const;

    void internalLoadUserPlugin(const std::string& moduleToLoad) const;
};

/** A set of generators  */
using GeneratorSet = std::vector<Generator::Ptr>;

/** Applies a set of generators to a given input raw observation(s) and
 *  generates a metric_map_t.
 *
 *  \note The former contents on the output metric_map_t object are untouched,
 *  so calling this function several times can be used to accumulate point cloud
 * elements from different sensors.
 * \return true if any of the generators actually processed "obs"
 */
bool apply_generators(
    const GeneratorSet& generators, const mrpt::obs::CObservation& obs,
    mp2p_icp::metric_map_t&                    output,
    const std::optional<mrpt::poses::CPose3D>& robotPose = std::nullopt);

/// \overload (functional version returning the metric_map_t)
[[nodiscard]] mp2p_icp::metric_map_t apply_generators(
    const GeneratorSet& generators, const mrpt::obs::CObservation& obs,
    const std::optional<mrpt::poses::CPose3D>& robotPose = std::nullopt);

/** \overload (version with an input CSensoryFrame)
 *  \return true if any of the generators actually processed any of the
 *          observations in the SF.
 */
bool apply_generators(
    const GeneratorSet& generators, const mrpt::obs::CSensoryFrame& sf,
    mp2p_icp::metric_map_t&                    output,
    const std::optional<mrpt::poses::CPose3D>& robotPose = std::nullopt);

/** \overload (functional version returning the metric_map_t) */
[[nodiscard]] mp2p_icp::metric_map_t apply_generators(
    const GeneratorSet& generators, const mrpt::obs::CSensoryFrame& sf,
    const std::optional<mrpt::poses::CPose3D>& robotPose = std::nullopt);

/** Creates a set of generators from a YAML configuration block (a sequence).
 *  Returns an empty generators set for an empty or null yaml node.
 *  Refer to YAML file examples.
 *  Returned generators are already initialize()'d.
 */
[[nodiscard]] GeneratorSet generators_from_yaml(
    const mrpt::containers::yaml&       c,
    const mrpt::system::VerbosityLevel& vLevel = mrpt::system::LVL_INFO);

/** \overload Taking a YAML filename as input.
 *  The file must contain with a top entry named `generators` with the sequence
 *  of generator descriptors.
 *  Returns an empty generators set for an empty or null yaml node.
 *  Refer to YAML file examples.
 *  Returned generators are already initialize()'d.
 */
[[nodiscard]] GeneratorSet generators_from_yaml_file(
    const std::string&                  filename,
    const mrpt::system::VerbosityLevel& vLevel = mrpt::system::LVL_INFO);

/** @} */

}  // namespace mp2p_icp_filters
