/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Generator.cpp
 * @brief  Base virtual class for point cloud filters
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp_filters/Generator.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/system/filesystem.h>

IMPLEMENTS_MRPT_OBJECT(Generator, mrpt::rtti::CObject, mp2p_icp_filters)

using namespace mp2p_icp_filters;

Generator::Generator() : mrpt::system::COutputLogger("Generator") {}

void Generator::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_OPT(c, target_layer);
    MCP_LOAD_OPT(c, metric_map_definition_ini_file);
    MCP_LOAD_OPT(c, process_class_names_regex);
    MCP_LOAD_OPT(c, process_sensor_labels_regex);
    MCP_LOAD_OPT(c, throw_on_unhandled_observation_class);
}

void Generator::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);

    process_class_names_regex_ = std::regex(params_.process_class_names_regex);
    process_sensor_labels_regex_ =
        std::regex(params_.process_sensor_labels_regex);

    initialized_ = true;
    MRPT_END
}

void Generator::process(
    const mrpt::obs::CObservation& o, mp2p_icp::metric_map_t& out,
    const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    MRPT_START
    using namespace mrpt::obs;

    ASSERTMSG_(
        initialized_,
        "initialize() must be called once before using process().");

    const auto obsClassName = o.GetRuntimeClass()->className;

    MRPT_LOG_DEBUG_FMT(
        "Processing observation type='%s' label='%s'", obsClassName,
        o.sensorLabel.c_str());

    // default: use point clouds:
    if (params_.metric_map_definition_ini_file.empty())
    {
        bool processed = false;

        // user-given filters: Done *AFTER* creating the map, if needed.
        if (!std::regex_match(obsClassName, process_class_names_regex_)) return;
        if (!std::regex_match(o.sensorLabel, process_sensor_labels_regex_))
            return;

        // load lazy-load from disk:
        o.load();

        if (auto oRS = dynamic_cast<const CObservationRotatingScan*>(&o); oRS)
            processed = filterRotatingScan(*oRS, out, robotPose);
        else if (auto o0 = dynamic_cast<const CObservationPointCloud*>(&o); o0)
        {
            ASSERT_(o0->pointcloud);
            processed = filterPointCloud(
                *o0->pointcloud, o0->sensorPose, out, robotPose);
        }
        else if (auto o1 = dynamic_cast<const CObservation2DRangeScan*>(&o); o1)
            processed = filterScan2D(*o1, out, robotPose);
        else if (auto o2 = dynamic_cast<const CObservation3DRangeScan*>(&o); o2)
            processed = filterScan3D(*o2, out, robotPose);
        else if (auto o3 = dynamic_cast<const CObservationVelodyneScan*>(&o);
                 o3)
            processed = filterVelodyneScan(*o3, out, robotPose);

        // done?
        if (processed)
        {
            // o.unload();  // DON'T! We don't know who else is using the data
            return;  // we are done.
        }

        // Create if new: Append to existing layer, if already existed.
        mrpt::maps::CPointsMap::Ptr outPc;
        if (auto itLy = out.layers.find(params_.target_layer);
            itLy != out.layers.end())
        {
            outPc =
                std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(itLy->second);
            if (!outPc)
                THROW_EXCEPTION_FMT(
                    "Layer '%s' must be of point cloud type.",
                    params_.target_layer.c_str());
        }
        else
        {
            outPc = mrpt::maps::CSimplePointsMap::Create();
            out.layers[params_.target_layer] = outPc;
        }

        if (!outPc) outPc = mrpt::maps::CSimplePointsMap::Create();

        // Observation format:

        // Special case:
        // unproject 3D points, if needed:
        if (auto obs3D =
                dynamic_cast<const mrpt::obs::CObservation3DRangeScan*>(&o);
            obs3D && obs3D->points3D_x.empty())
        {
            mrpt::maps::CSimplePointsMap tmpMap;

            mrpt::obs::T3DPointsProjectionParams pp;
            pp.takeIntoAccountSensorPoseOnRobot = true;
            pp.robotPoseInTheWorld              = robotPose;
            const_cast<mrpt::obs::CObservation3DRangeScan*>(obs3D)
                ->unprojectInto(tmpMap, pp);

            outPc->insertAnotherMap(&tmpMap, mrpt::poses::CPose3D::Identity());
        }
        else
        {
            // General case:
            const bool insertDone = o.insertObservationInto(*outPc, robotPose);

            if (!insertDone && params_.throw_on_unhandled_observation_class)
            {
                THROW_EXCEPTION_FMT(
                    "Observation of type '%s' could not be converted into a "
                    "point cloud, and none of the specializations handled it, "
                    "so I do not know what to do with this observation!",
                    obsClassName);
            }
        }

        // o.unload();  // DON'T! We don't know who else is using the data
    }
    else
    {
        // Use a user-defined custom map type:
        ASSERT_FILE_EXISTS_(params_.metric_map_definition_ini_file);

        mrpt::config::CConfigFile cfg(params_.metric_map_definition_ini_file);
        mrpt::maps::TSetOfMetricMapInitializers mapInits;
        mapInits.loadFromConfigFile(cfg, "map");

        mrpt::maps::CMultiMetricMap theMap;
        theMap.setListOfMaps(mapInits);

        ASSERT_(theMap.maps.size() >= 1);

        // Create if new: Append to existing layer, if already existed.
        mrpt::maps::CMetricMap::Ptr outMap = theMap.maps.at(0);

        if (auto itLy = out.layers.find(params_.target_layer);
            itLy != out.layers.end())
        {
            if (!outMap->GetRuntimeClass()->derivedFrom(
                    itLy->second->GetRuntimeClass()))
            {
                THROW_EXCEPTION_FMT(
                    "Layer '%s' already existed, but it has class '%s' while "
                    "it was expected to be '%s'.",
                    params_.target_layer.c_str(),
                    itLy->second->GetRuntimeClass()->className,
                    outMap->GetRuntimeClass()->className);
            }

            outMap = itLy->second;
        }
        else
        {
            // insert new layer:
            out.layers[params_.target_layer] = outMap;
        }

        ASSERT_(outMap);

        // user-given filters: Done *AFTER* creating the map, if needed.
        if (!std::regex_match(obsClassName, process_class_names_regex_)) return;
        if (!std::regex_match(o.sensorLabel, process_sensor_labels_regex_))
            return;

        // Observation format:
        o.load();

        // General case:
        const bool insertDone = o.insertObservationInto(*outMap, robotPose);

        if (!insertDone && params_.throw_on_unhandled_observation_class)
        {
            THROW_EXCEPTION_FMT(
                "Observation of type '%s' could not be converted inserted into "
                "the map of type '%s', so I do not know what to do with this "
                "observation!",
                obsClassName, outMap->GetRuntimeClass()->className);
        }

        // o.unload();  // DON'T! We don't know who else is using the data
    }

    MRPT_END
}

bool Generator::filterScan2D(  //
    [[maybe_unused]] const mrpt::obs::CObservation2DRangeScan&  pc,
    [[maybe_unused]] mp2p_icp::metric_map_t&                    out,
    [[maybe_unused]] const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    return false;  // Not implemented
}

bool Generator::filterVelodyneScan(  //
    [[maybe_unused]] const mrpt::obs::CObservationVelodyneScan& pc,
    [[maybe_unused]] mp2p_icp::metric_map_t&                    out,
    [[maybe_unused]] const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    return false;  // Not implemented
}

bool Generator::filterScan3D(  //
    [[maybe_unused]] const mrpt::obs::CObservation3DRangeScan&  pc,
    [[maybe_unused]] mp2p_icp::metric_map_t&                    out,
    [[maybe_unused]] const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    return false;  // Not implemented
}

bool Generator::filterPointCloud(  //
    const mrpt::maps::CPointsMap& pc, const mrpt::poses::CPose3D& sensorPose,
    mp2p_icp::metric_map_t&                    out,
    const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPc;
    if (auto itLy = out.layers.find(params_.target_layer);
        itLy != out.layers.end())
    {
        outPc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(itLy->second);
        if (!outPc)
            THROW_EXCEPTION_FMT(
                "Layer '%s' must be of point cloud type.",
                params_.target_layer.c_str());
    }
    else
    {
        // Make a new layer of the same type than the input cloud:
        outPc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
            pc.GetRuntimeClass()->ptrCreateObject());
        ASSERT_(outPc);

        MRPT_LOG_DEBUG_FMT(
            "[filterPointCloud] Created new layer '%s' of type '%s'",
            params_.target_layer.c_str(), outPc->GetRuntimeClass()->className);

        out.layers[params_.target_layer] = outPc;
    }

    const mrpt::poses::CPose3D p =
        robotPose ? robotPose.value() + sensorPose : sensorPose;

    outPc->insertAnotherMap(&pc, p);

    return true;
}

bool Generator::filterRotatingScan(  //
    [[maybe_unused]] const mrpt::obs::CObservationRotatingScan& pc,
    [[maybe_unused]] mp2p_icp::metric_map_t&                    out,
    [[maybe_unused]] const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    return false;  // Not implemented
}

void mp2p_icp_filters::apply_generators(
    const GeneratorSet& generators, const mrpt::obs::CObservation& obs,
    mp2p_icp::metric_map_t&                    output,
    const std::optional<mrpt::poses::CPose3D>& robotPose)
{
    ASSERT_(!generators.empty());
    for (const auto& g : generators)
    {
        ASSERT_(g.get() != nullptr);
        g->process(obs, output, robotPose);
    }
}

mp2p_icp::metric_map_t mp2p_icp_filters::apply_generators(
    const GeneratorSet& generators, const mrpt::obs::CObservation& obs,
    const std::optional<mrpt::poses::CPose3D>& robotPose)
{
    mp2p_icp::metric_map_t pc;
    apply_generators(generators, obs, pc, robotPose);
    return pc;
}

mp2p_icp::metric_map_t mp2p_icp_filters::apply_generators(
    const GeneratorSet& generators, const mrpt::obs::CSensoryFrame& sf,
    const std::optional<mrpt::poses::CPose3D>& robotPose)
{
    mp2p_icp::metric_map_t pc;
    apply_generators(generators, sf, pc, robotPose);
    return pc;
}

void mp2p_icp_filters::apply_generators(
    const GeneratorSet& generators, const mrpt::obs::CSensoryFrame& sf,
    mp2p_icp::metric_map_t&                    output,
    const std::optional<mrpt::poses::CPose3D>& robotPose)
{
    ASSERT_(!generators.empty());
    for (const auto& g : generators)
    {
        ASSERT_(g.get() != nullptr);
        for (const auto& obs : sf)
        {
            if (!obs) continue;
            g->process(*obs, output, robotPose);
        }
    }
}

GeneratorSet mp2p_icp_filters::generators_from_yaml(
    const mrpt::containers::yaml& c, const mrpt::system::VerbosityLevel& vLevel)
{
    if (c.isNullNode()) return {};

    ASSERT_(c.isSequence());

    GeneratorSet generators;

    for (const auto& entry : c.asSequence())
    {
        const auto& e = entry.asMap();

        const auto sClass = e.at("class_name").as<std::string>();
        auto       o      = mrpt::rtti::classFactory(sClass);
        ASSERT_(o);

        auto f = std::dynamic_pointer_cast<Generator>(o);
        ASSERTMSG_(
            f, mrpt::format(
                   "`%s` class seems not to be derived from Generator",
                   sClass.c_str()));

        f->setMinLoggingLevel(vLevel);

        f->initialize(e.at("params"));
        generators.push_back(f);
    }

    return generators;
}

GeneratorSet mp2p_icp_filters::generators_from_yaml_file(
    const std::string& filename, const mrpt::system::VerbosityLevel& vLevel)
{
    const auto yamlContent = mrpt::containers::yaml::FromFile(filename);

    ASSERT_(
        yamlContent.has("generators") &&
        yamlContent["generators"].isSequence());

    return generators_from_yaml(yamlContent["generators"], vLevel);
}
