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
 * @file   Generator.cpp
 * @brief  Base virtual class for point cloud filters
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
#include <mola_imu_preintegration/ImuTransformer.h>
#endif

#include <mp2p_icp/load_plugin.h>
#include <mp2p_icp/pointcloud_sanity_check.h>
#include <mp2p_icp_filters/Generator.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/cpu.h>
#include <mrpt/core/get_env.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

#include "Generator_view_vector_impl.h"

IMPLEMENTS_MRPT_OBJECT(Generator, mrpt::rtti::CObject, mp2p_icp_filters)

using namespace mp2p_icp_filters;

Generator::Generator() : mrpt::system::COutputLogger("Generator") {}

void Generator::Parameters::load_from_yaml(const mrpt::containers::yaml& c, Generator& parent)
{
    using namespace std::string_literals;

    MCP_LOAD_OPT(c, target_layer);
    MCP_LOAD_OPT(c, metric_map_definition_ini_file);
    MCP_LOAD_OPT(c, process_class_names_regex);
    MCP_LOAD_OPT(c, process_sensor_labels_regex);
    MCP_LOAD_OPT(c, throw_on_unhandled_observation_class);
    MCP_LOAD_OPT(c, generate_view_vector);

    MCP_LOAD_OPT(c, name);
    if (!name.empty())
    {
        parent.mrpt::system::COutputLogger::setLoggerName(name);
    }

    if (c.has("metric_map_definition"))
    {
        // Store values here:
        auto& trg = metric_map_definition;

        const std::set<std::string> allowedTopKeys = {"class", "plugin"};

        ASSERT_(c["metric_map_definition"].isMap());
        const auto mmd = c["metric_map_definition"].asMap();
        for (const auto& [k, v] : mmd)
        {
            const auto key = k.as<std::string>();
            if (v.isNullNode())
            {
                continue;  // ignore
            }
            if (v.isScalar())
            {
                if (allowedTopKeys.count(key) != 0)
                {
                    // simple key=value:
                    ASSERT_(v.isScalar());
                    trg[key] = v.as<std::string>();
                    continue;
                }

                THROW_EXCEPTION_FMT("scalar key '%s' not allowed here.", key.c_str());
            }
            // if should then be a map
            if (!v.isMap())
            {
                THROW_EXCEPTION_FMT("key '%s' must be either a scalar or a map", key.c_str());
            }

            trg[key]     = mrpt::containers::yaml::Map();
            auto& newMap = trg.asMap().at(key).asMap();

            for (const auto& [kk, vv] : v.asMap())
            {
                const std::string val = vv.as<std::string>();
                // Special case: handle parameterizable formulas:
                if (val.substr(0, 3) == "$f{"s)
                {
                    ASSERTMSG_(val.back() == '}', "Missing '}' in '$f{' formula");

                    const auto ks = kk.as<std::string>();

                    auto [it, exist]    = newMap.insert({ks, .0});
                    double& placeholder = *std::any_cast<double>(&it->second.asScalar());

                    parent.parseAndDeclareParameter(val.substr(3, val.size() - 4), placeholder);
                }
                else
                {
                    // regular entry:
                    newMap[kk.as<std::string>()] = val;
                }
            }
        }

        std::stringstream ss;
        ss << "Using this metric_map_definition:\n"s << metric_map_definition;
        parent.logStr(mrpt::system::LVL_DEBUG, ss.str());
    }
}

void Generator::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);

    process_class_names_regex_   = std::regex(params.process_class_names_regex);
    process_sensor_labels_regex_ = std::regex(params.process_sensor_labels_regex);

    initialized_ = true;
    MRPT_END
}

bool Generator::process(
    const mrpt::obs::CObservation& o, mp2p_icp::metric_map_t& out,
    const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    MRPT_START

    Parameterizable::checkAllParametersAreRealized();

    ASSERTMSG_(initialized_, "initialize() must be called once before using process().");

    const auto obsClassName = o.GetRuntimeClass()->className;

    MRPT_LOG_DEBUG_FMT(
        "Processing observation type='%s' label='%s' stamp=%.04f", obsClassName,
        o.sensorLabel.c_str(), mrpt::Clock::toDouble(o.timestamp));

    mrpt::maps::CPointsMap* outPc = nullptr;

    const auto lambdaGetOutputLayer = [&]()
    {
        outPc = nullptr;
        if (auto itLy = out.layers.find(params.target_layer); itLy != out.layers.end())
        {
            outPc = dynamic_cast<mrpt::maps::CPointsMap*>(itLy->second.get());
        }
    };

    lambdaGetOutputLayer();  // try to get the layer if it existed already
    const size_t ptsBefore = outPc ? outPc->size() : 0;

    // Actual insertion into map:
    const bool actualInserted = [&]()
    {
        if (params.metric_map_definition_ini_file.empty() && params.metric_map_definition.empty())
        {
            // default: use point clouds:
            return implProcessDefault(o, out, robotPose);
        }
        return implProcessCustomMap(o, out, robotPose);
    }();
    if (!actualInserted)
    {
        return false;
    }

    lambdaGetOutputLayer();  // try it again if it was created in the functions above

    // Add view vectors for the newly-inserted points.
    // The sensor origin in the output frame is p.translation():
    if (outPc != nullptr)
    {
        const auto p =
            (robotPose.has_value() ? robotPose->asTPose() : mrpt::math::TPose3D::Identity()) +
            o.sensorPose();
        addViewVectorField(*outPc, ptsBefore, p.translation());

        const bool sanityPassed = mp2p_icp::pointcloud_sanity_check(*outPc);
        ASSERT_(sanityPassed);

        MRPT_LOG_DEBUG_FMT(
            "[process] Ended with layer '%s' having: %s", params.target_layer.c_str(),
            outPc->asString().c_str());
    }

    return true;

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
    const mrpt::obs::CObservationVelodyneScan& pc, mp2p_icp::metric_map_t& out,
    const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    mrpt::maps::CPointsMap::Ptr outPc = GetOrCreatePointLayer(
        out, params.target_layer, false /*does not allow empty name*/,
        "mrpt::maps::CGenericPointsMap" /* creation class if not existing */);
    ASSERT_(outPc);

    auto m = std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(outPc);
    ASSERTMSG_(
        m,
        "Output layer must be of type mrpt::maps::CGenericPointsMap for the "
        "specialized filterVelodyneScan() generator.");

    m->insertObservation(pc, robotPose);

    return true;  // implemented
}

#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
class ImuTransformerManager
{
   public:
    static mola::imu::ImuTransformer& getInstance(const std::string& sensorLabel)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto                        it = transformers_.find(sensorLabel);
        if (it == transformers_.end())
        {
            auto [newIt, _] = transformers_.emplace(sensorLabel, mola::imu::ImuTransformer());
            return newIt->second;
        }
        return it->second;
    }

   private:
    static std::mutex                                       mutex_;
    static std::map<std::string, mola::imu::ImuTransformer> transformers_;
};

std::mutex                                       ImuTransformerManager::mutex_;
std::map<std::string, mola::imu::ImuTransformer> ImuTransformerManager::transformers_;
#endif

bool Generator::processIMU(const mrpt::obs::CObservationIMU& imu_raw) const
{
#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
    auto* parameterSource = const_cast<mp2p_icp::ParameterSource*>(attachedSource());
    if (!parameterSource)
    {
        return false;  // No parameter source attached, nothing to do.
    }

    const double t = mrpt::Clock::toDouble(imu_raw.timestamp);

    auto& imu_transformer = ImuTransformerManager::getInstance(imu_raw.sensorLabel);

    // Convert IMU readings to vehicle frame of reference:
    const auto imu = imu_transformer.process(imu_raw);

    // gyroscope data:
    if (imu.has(mrpt::obs::IMU_WX) && imu.has(mrpt::obs::IMU_WY) && imu.has(mrpt::obs::IMU_WZ))
    {
        parameterSource->localVelocityBuffer.add_angular_velocity(
            t,
            {imu.get(mrpt::obs::IMU_WX), imu.get(mrpt::obs::IMU_WY), imu.get(mrpt::obs::IMU_WZ)});
    }
    // accelerometer data:
    if (imu.has(mrpt::obs::IMU_X_ACC) && imu.has(mrpt::obs::IMU_Y_ACC) &&
        imu.has(mrpt::obs::IMU_Z_ACC))
    {
        parameterSource->localVelocityBuffer.add_linear_acceleration(
            t, {imu.get(mrpt::obs::IMU_X_ACC), imu.get(mrpt::obs::IMU_Y_ACC),
                imu.get(mrpt::obs::IMU_Z_ACC)});
    }

    return true;  // implemented
#else
    (void)imu_raw;
    return false;  // Not implemented
#endif
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
    mp2p_icp::metric_map_t& out, const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPc;
    if (auto itLy = out.layers.find(params.target_layer); itLy != out.layers.end())
    {
        outPc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(itLy->second);
        if (!outPc)
        {
            THROW_EXCEPTION_FMT(
                "Layer '%s' must be of point cloud type.", params.target_layer.c_str());
        }
    }
    else
    {
        // Make a new layer of the same type than the input cloud:
        outPc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
            pc.GetRuntimeClass()->ptrCreateObject());
        ASSERT_(outPc);

        MRPT_LOG_DEBUG_FMT(
            "[filterPointCloud] Created new layer '%s' of type '%s'", params.target_layer.c_str(),
            outPc->GetRuntimeClass()->className);

        out.layers[params.target_layer] = outPc;
    }

    const mrpt::poses::CPose3D p = robotPose ? robotPose.value() + sensorPose : sensorPose;

    outPc->registerPointFieldsFrom(pc);
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

// ---------------------------------------------------------------------------
// Scalar kernel – always compiled, used as fallback and for the tail.
// ---------------------------------------------------------------------------
namespace mp2p_icp_filters::internal
{
void addViewVectors_scalar(
    float* vx, float* vy, float* vz, const float* px, const float* py, const float* pz, size_t N,
    float sx, float sy, float sz)
{
    for (size_t i = 0; i < N; i++)
    {
        const float dx      = sx - px[i];
        const float dy      = sy - py[i];
        const float dz      = sz - pz[i];
        const float nsq     = dx * dx + dy * dy + dz * dz;
        const float inv_nrm = (nsq > 1e-20f) ? (1.0f / std::sqrt(nsq)) : 0.0f;
        vx[i]               = dx * inv_nrm;
        vy[i]               = dy * inv_nrm;
        vz[i]               = dz * inv_nrm;
    }
}
}  // namespace mp2p_icp_filters::internal

void Generator::addViewVectorField(
    mrpt::maps::CPointsMap& pc, size_t firstNewPtIdx,
    const mrpt::math::TPoint3D& sensorOrigin) const
{
    auto* genPc = dynamic_cast<mrpt::maps::CGenericPointsMap*>(&pc);
    if (!genPc)
    {
        if (params.generate_view_vector)
        {
            MRPT_LOG_DEBUG(
                "[addViewVectorField] Output layer is not CGenericPointsMap; skipping view vector "
                "generation.");
        }
        return;
    }

    // Exit now if we are not generating view vectors, not without first checking consistency:
    if (!params.generate_view_vector)
    {
        for (const char* const fname : {"view_x", "view_y", "view_z"})
        {
            if (genPc->hasPointField(fname))
            {
                THROW_EXCEPTION(
                    "Consistency check didn't pass: generate_view_vector=false, but the map layer "
                    "already has view vector fields. Continuing would leave the cloud in an "
                    "inconsistent state.");
            }
        }
        // All look ok.
        return;
    }

    const size_t nTotal = pc.size();
    ASSERT_LE_(firstNewPtIdx, nTotal);
    if (firstNewPtIdx >= nTotal)
    {
        return;
    }

    // Register each field if absent; if already present, resize to cover nTotal.
    // registerField_float() initialises the new vector to size() zeros, so registering
    // after insertAnotherMap() gives us correctly-sized storage for all points.
    for (const char* const fname : {"view_x", "view_y", "view_z"})
    {
        if (!genPc->hasPointField(fname))
        {
            genPc->registerField_float(fname);
        }
        else
        {
            // Field existed from a prior append; extend it with zeros for new slots.
            auto* fv = genPc->getPointsBufferRef_float_field(fname);
            if (fv->size() < nTotal)
            {
                fv->resize(nTotal, 0.0f);
            }
        }
    }

    // Direct buffer access: one hash lookup per field, not one per point.
    float* vx_data = genPc->getPointsBufferRef_float_field("view_x")->data() + firstNewPtIdx;
    float* vy_data = genPc->getPointsBufferRef_float_field("view_y")->data() + firstNewPtIdx;
    float* vz_data = genPc->getPointsBufferRef_float_field("view_z")->data() + firstNewPtIdx;

    const float* px_data = pc.getPointsBufferRef_x().data() + firstNewPtIdx;
    const float* py_data = pc.getPointsBufferRef_y().data() + firstNewPtIdx;
    const float* pz_data = pc.getPointsBufferRef_z().data() + firstNewPtIdx;

    const float  sx = static_cast<float>(sensorOrigin.x);
    const float  sy = static_cast<float>(sensorOrigin.y);
    const float  sz = static_cast<float>(sensorOrigin.z);
    const size_t N  = nTotal - firstNewPtIdx;

    // Runtime-dispatch to the best available SIMD implementation.
    // The SIMD translation units are compiled with their own -mavx / -msse2 flags;
    // the main library is compiled without those flags.
#if defined(MP2P_HAS_AVX)
    if (mrpt::cpu::supports(mrpt::cpu::feature::AVX))
    {
        internal::addViewVectors_avx(
            vx_data, vy_data, vz_data, px_data, py_data, pz_data, N, sx, sy, sz);
        return;
    }
#endif
#if defined(MP2P_HAS_SSE2)
    if (mrpt::cpu::supports(mrpt::cpu::feature::SSE2))
    {
        internal::addViewVectors_sse2(
            vx_data, vy_data, vz_data, px_data, py_data, pz_data, N, sx, sy, sz);
        return;
    }
#endif
    internal::addViewVectors_scalar(
        vx_data, vy_data, vz_data, px_data, py_data, pz_data, N, sx, sy, sz);
}

bool mp2p_icp_filters::apply_generators(
    const GeneratorSet& generators, const mrpt::obs::CObservation& obs,
    mp2p_icp::metric_map_t& output, const std::optional<mrpt::poses::CPose3D>& robotPose,
    const mrpt::optional_ref<mrpt::system::CTimeLogger>& profiler)
{
    ASSERT_(!generators.empty());
    bool anyHandled = false;
    for (const auto& g : generators)
    {
        ASSERT_(g.get() != nullptr);

        // Optional profiler:
        std::optional<mrpt::system::CTimeLoggerEntry> tle;
        if (profiler)
        {
            if (g->params.name.empty())
            {
                tle.emplace(*profiler, g->GetRuntimeClass()->className);
            }
            else
            {
                tle.emplace(*profiler, g->params.name);
            }
        }

        bool handled = g->process(obs, output, robotPose);
        anyHandled   = anyHandled || handled;
    }
    return anyHandled;
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

bool mp2p_icp_filters::apply_generators(
    const GeneratorSet& generators, const mrpt::obs::CSensoryFrame& sf,
    mp2p_icp::metric_map_t& output, const std::optional<mrpt::poses::CPose3D>& robotPose)
{
    ASSERT_(!generators.empty());
    bool anyHandled = false;
    for (const auto& g : generators)
    {
        ASSERT_(g.get() != nullptr);
        for (const auto& obs : sf)
        {
            if (!obs)
            {
                continue;
            }
            const bool handled = g->process(*obs, output, robotPose);

            anyHandled = anyHandled || handled;
        }
    }
    return anyHandled;
}

GeneratorSet mp2p_icp_filters::generators_from_yaml(
    const mrpt::containers::yaml& c, const mrpt::system::VerbosityLevel& vLevel)
{
    if (c.isNullNode())
    {
        return {};
    }

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
            f, mrpt::format("`%s` class seems not to be derived from Generator", sClass.c_str()));

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

    ASSERT_(yamlContent.has("generators") && yamlContent["generators"].isSequence());

    return generators_from_yaml(yamlContent["generators"], vLevel);
}

bool Generator::implProcessDefault(
    const mrpt::obs::CObservation& o, mp2p_icp::metric_map_t& out,
    const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    using namespace mrpt::obs;
    using namespace std::string_literals;

    bool processed = false;

    const auto obsClassName = o.GetRuntimeClass()->className;

    // user-given filters: Done *AFTER* creating the map, if needed.
    if (obsClassName == "mrpt::obs::CObservationComment"s ||
        obsClassName == "mrpt::obs::CObservationGPS"s ||
        obsClassName == "mrpt::obs::CObservationRobotPose"s ||
        !std::regex_match(obsClassName, process_class_names_regex_) ||
        !std::regex_match(o.sensorLabel, process_sensor_labels_regex_))
    {
        MRPT_LOG_DEBUG_STREAM(
            "Skipping observation of type '" << obsClassName << "' with label '" << o.sensorLabel
                                             << "'.");
        return false;
    }

    // load lazy-load from disk:
    o.load();

    // Prepare the local velocity buffer to set the reference zero time to the point cloud
    // timestamp, which will later on be adjusted with the offset by FilterAdjustTimestamps
    std::optional<mrpt::Clock::time_point> pointcloud_obs_timestamp;

    if (auto oRS = dynamic_cast<const CObservationRotatingScan*>(&o); oRS)
    {
        processed                = filterRotatingScan(*oRS, out, robotPose);
        pointcloud_obs_timestamp = o.timestamp;
    }
    else if (auto o0 = dynamic_cast<const CObservationPointCloud*>(&o); o0)
    {
        ASSERT_(o0->pointcloud);
        processed = filterPointCloud(*o0->pointcloud, o0->sensorPose, out, robotPose);
        pointcloud_obs_timestamp = o.timestamp;
    }
    else if (auto o1 = dynamic_cast<const CObservation2DRangeScan*>(&o); o1)
    {
        processed                = filterScan2D(*o1, out, robotPose);
        pointcloud_obs_timestamp = o.timestamp;
    }
    else if (auto o2 = dynamic_cast<const CObservation3DRangeScan*>(&o); o2)
    {
        processed                = filterScan3D(*o2, out, robotPose);
        pointcloud_obs_timestamp = o.timestamp;
    }
    else if (auto o3 = dynamic_cast<const CObservationVelodyneScan*>(&o); o3)
    {
        processed                = filterVelodyneScan(*o3, out, robotPose);
        pointcloud_obs_timestamp = o.timestamp;
    }
    else if (auto oIMU = dynamic_cast<const CObservationIMU*>(&o); oIMU)
    {
        processed = processIMU(*oIMU);
    }

#if defined(MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
    if (auto ps = this->attachedSource(); ps != nullptr && pointcloud_obs_timestamp)
    {
        const auto refStamp = mrpt::Clock::toDouble(*pointcloud_obs_timestamp);
        MRPT_LOG_DEBUG_FMT(
            "Setting localVelocityBuffer.set_reference_zero_time() to %.04f", refStamp);

        ps->localVelocityBuffer.set_reference_zero_time(refStamp);
    }
#endif

    // done?
    if (processed)
    {
        // o.unload();  // DON'T! We don't know who else is using the data
        return true;  // we are done.
    }

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPc = GetOrCreatePointLayer(
        out, params.target_layer, false /*does not allow empty name*/,
        "mrpt::maps::CSimplePointsMap" /* creation class if not existing */);

    ASSERT_(outPc);

    MRPT_LOG_DEBUG_FMT(
        "Using output layer '%s' of type '%s'", params.target_layer.c_str(),
        outPc->GetRuntimeClass()->className);

    // Observation format:

    // Special case:
    // unproject 3D points, if needed:
    if (auto obs3D = dynamic_cast<const mrpt::obs::CObservation3DRangeScan*>(&o);
        obs3D && obs3D->points3D_x.empty())
    {
        mrpt::maps::CSimplePointsMap tmpMap;

        mrpt::obs::T3DPointsProjectionParams pp;
        pp.takeIntoAccountSensorPoseOnRobot = true;
        pp.robotPoseInTheWorld              = robotPose;
        const_cast<mrpt::obs::CObservation3DRangeScan*>(obs3D)->unprojectInto(tmpMap, pp);

        outPc->insertAnotherMap(&tmpMap, mrpt::poses::CPose3D::Identity());

        return true;
    }

    // General case:
    const bool insertDone = o.insertObservationInto(*outPc, robotPose);

    if (!insertDone && params.throw_on_unhandled_observation_class)
    {
        THROW_EXCEPTION_FMT(
            "Observation of type '%s' could not be converted into a "
            "point cloud, and none of the specializations handled it, "
            "so I do not know what to do with this observation!",
            obsClassName);
    }
    return insertDone;

    // o.unload();  // DON'T! We don't know who else is using the data
}

bool Generator::implProcessCustomMap(
    const mrpt::obs::CObservation& o, mp2p_icp::metric_map_t& out,
    const std::optional<mrpt::poses::CPose3D>& robotPose) const
{
    using namespace std::string_literals;

    const auto obsClassName = o.GetRuntimeClass()->className;

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CMetricMap::Ptr outMap;

    if (auto itLy = out.layers.find(params.target_layer); itLy != out.layers.end())
    {
        outMap = itLy->second;
    }
    else
    {
        // insert new layer:
        mrpt::maps::TSetOfMetricMapInitializers mapInits;

        const std::string cfgPrefix = "map"s;

        // Create from either a INI file or the YAML version of it:
        if (!params.metric_map_definition_ini_file.empty())
        {
            // Load from INI file
            // ------------------------------
            ASSERT_FILE_EXISTS_(params.metric_map_definition_ini_file);
            mrpt::config::CConfigFile cfg(params.metric_map_definition_ini_file);
            mapInits.loadFromConfigFile(cfg, cfgPrefix);
        }
        else
        {
            // Load from YAML file (with parameterizable values)
            // ------------------------------------------------------
            ASSERT_(!params.metric_map_definition.empty());
            const auto& c = params.metric_map_definition;

            // Build an in-memory INI file with the structure expected by
            // loadFromConfigFile():
            mrpt::config::CConfigFileMemory cfg;

            ASSERT_(c.has("class"));
            const std::string mapClass = c["class"].as<std::string>();
            cfg.write(cfgPrefix, mapClass + "_count"s, "1");

            // optional plugin module?
            if (c.has("plugin"))
            {
                const auto moduleToLoad = c["plugin"].as<std::string>();
                MRPT_LOG_DEBUG_STREAM("About to load user-defined plugin: " << moduleToLoad);
                mp2p_icp::load_plugin(moduleToLoad, this);
            }

            // fill the rest sub-sections:
            for (const auto& [k, v] : c.asMap())
            {
                if (!v.isMap())
                {
                    continue;
                }
                const auto keyVal   = k.as<std::string>();
                const auto sectName = cfgPrefix + "_"s + mapClass + "_00_"s + keyVal;
                for (const auto& [kk, vv] : v.asMap())
                {
                    ASSERT_(kk.isScalar());
                    ASSERT_(vv.isScalar());

                    cfg.write(sectName, kk.as<std::string>(), vv.as<std::string>());
                }
            }

            MRPT_LOG_DEBUG_STREAM(
                "Built INI-like block for layer '" << params.target_layer << "':\n"
                                                   << cfg.getContent());

            // parse it:
            mapInits.loadFromConfigFile(cfg, cfgPrefix);
        }

        // create the map:
        mrpt::maps::CMultiMetricMap theMap;
        theMap.setListOfMaps(mapInits);

        ASSERT_(theMap.maps.size() >= 1);
        outMap = theMap.maps.at(0);

        out.layers[params.target_layer] = outMap;
    }

    ASSERT_(outMap);

    // user-given filters: Done *AFTER* creating the map, if needed.
    if (obsClassName == "mrpt::obs::CObservationComment"s ||
        !std::regex_match(obsClassName, process_class_names_regex_) ||
        !std::regex_match(o.sensorLabel, process_sensor_labels_regex_))
    {
        MRPT_LOG_DEBUG_STREAM(
            "Skipping observation of type '" << obsClassName << "' with label '" << o.sensorLabel
                                             << "'.");
        return false;
    }

    // Observation format:
    o.load();

    // Use virtual insert method:
    const bool insertDone = o.insertObservationInto(*outMap, robotPose);

    if (!insertDone && params.throw_on_unhandled_observation_class)
    {
        THROW_EXCEPTION_FMT(
            "Observation of type '%s' could not be converted inserted into "
            "the map of type '%s', so I do not know what to do with this "
            "observation!",
            obsClassName, outMap->GetRuntimeClass()->className);
    }

    // o.unload();  // DON'T! We don't know who else is using the data
    return insertDone;  // handled
}
