project(mp2p_icp_filters LANGUAGES CXX)

# -----------------------
# define lib:
# ---------------------------------------------------------------------------
# SIMD support detection for optimized operations.
# On x86/x86_64 we compile separate translation units with -mavx / -msse2
# so that the rest of the library can be compiled without those flags.
# That also makes Eigen3 included headers not affected.
# ---------------------------------------------------------------------------
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|i686|AMD64|amd64)$")
  set(MP2P_ARCH_INTEL_COMPATIBLE TRUE)
else()
  set(MP2P_ARCH_INTEL_COMPATIBLE FALSE)
endif()

set(LIB_SRCS
  src/FilterAbsoluteTimestamp.cpp
  src/FilterAdjustTimestamps.cpp
  src/FilterBase.cpp
  src/FilterBoundingBox.cpp
  src/FilterByExpression.cpp
  src/FilterByIntensity.cpp
  src/FilterByRange.cpp
  src/FilterByRing.cpp
  src/FilterClear.cpp
  src/FilterCurvature.cpp
  src/FilterDecimate.cpp
  src/FilterDecimateAdaptive.cpp
  src/FilterDecimateVoxels.cpp
  src/FilterDeleteLayer.cpp
  src/FilterDeskew.cpp
  src/FilterEdgesPlanes.cpp
  src/FilterFartherPointSampling.cpp
  src/FilterMerge.cpp
  src/FilterMLS.cpp
  src/FilterNormalizeIntensity.cpp
  src/FilterPoleDetector.cpp
  src/FilterRemoveByVoxelOccupancy.cpp
  src/FilterRemovePointCloudField.cpp
  src/FilterRenameLayer.cpp
  src/FilterSOR.cpp
  src/FilterVoxelSlice.cpp
  src/FilterVoxelSOR.cpp
  src/Generator.cpp
  src/Generator_view_vector_sse2.cpp
  src/Generator_view_vector_avx.cpp
  src/GeneratorEdgesFromCurvature.cpp
  src/GeneratorEdgesFromRangeImage.cpp
  src/GetOrCreatePointLayer.cpp
  src/PointCloudToVoxelGrid.cpp
  src/PointCloudToVoxelGridSingle.cpp
  src/sm2mm.cpp
  #
  src/register.cpp # This must be last
)

set(LIB_PUBLIC_HDRS
  include/mp2p_icp_filters/FilterAbsoluteTimestamp.h
  include/mp2p_icp_filters/FilterAdjustTimestamps.h
  include/mp2p_icp_filters/FilterBase.h
  include/mp2p_icp_filters/FilterBoundingBox.h
  include/mp2p_icp_filters/FilterByExpression.h
  include/mp2p_icp_filters/FilterByIntensity.h
  include/mp2p_icp_filters/FilterByRange.h
  include/mp2p_icp_filters/FilterByRing.h
  include/mp2p_icp_filters/FilterClear.h
  include/mp2p_icp_filters/FilterCurvature.h
  include/mp2p_icp_filters/FilterDecimate.h
  include/mp2p_icp_filters/FilterDecimateAdaptive.h
  include/mp2p_icp_filters/FilterDecimateVoxels.h
  include/mp2p_icp_filters/FilterDeleteLayer.h
  include/mp2p_icp_filters/FilterDeskew.h
  include/mp2p_icp_filters/FilterEdgesPlanes.h
  include/mp2p_icp_filters/FilterFartherPointSampling.h
  include/mp2p_icp_filters/FilterMerge.h
  include/mp2p_icp_filters/FilterMLS.h
  include/mp2p_icp_filters/FilterNormalizeIntensity.h
  include/mp2p_icp_filters/FilterPoleDetector.h
  include/mp2p_icp_filters/FilterRemoveByVoxelOccupancy.h
  include/mp2p_icp_filters/FilterRenameLayer.h
  include/mp2p_icp_filters/FilterRemovePointCloudField.h
  include/mp2p_icp_filters/FilterSOR.h
  include/mp2p_icp_filters/FilterVoxelSlice.h
  include/mp2p_icp_filters/FilterVoxelSOR.h
  include/mp2p_icp_filters/Generator.h
  include/mp2p_icp_filters/GeneratorEdgesFromCurvature.h
  include/mp2p_icp_filters/GeneratorEdgesFromRangeImage.h
  include/mp2p_icp_filters/GetOrCreatePointLayer.h
  include/mp2p_icp_filters/PointCloudToVoxelGrid.h
  include/mp2p_icp_filters/PointCloudToVoxelGridSingle.h
  include/mp2p_icp_filters/sm2mm.h
)


if (mola_imu_preintegration_FOUND)
  set(imu_pub_link_ mola::mola_imu_preintegration)
  set(imu_cmake_dep_ mola_imu_preintegration)
endif()

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mola::mp2p_icp_map
    mola::mp2p_icp_common
  PRIVATE_LINK_LIBRARIES
    ${imu_pub_link_}
    tsl::robin_map
  CMAKE_DEPENDENCIES
    mp2p_icp
    mp2p_icp_common
    ${imu_cmake_dep_}
)

# We want C++20 for mp2p_icp_filters, to capture structured bindings (FilterDeskew.cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)

# ---------------------------------------------------------------------------
# Per-file SIMD compiler flags for the view-vector kernel.
# We compile each SIMD variant in its own translation unit so the rest of
# the library is not forced to enable AVX/SSE globally.
# ---------------------------------------------------------------------------
if(MP2P_ARCH_INTEL_COMPATIBLE)
  if(MSVC)
    # MSVC: SSE2 is on by default for x64; AVX needs /arch:AVX.
    set_source_files_properties(src/Generator_view_vector_sse2.cpp
      PROPERTIES COMPILE_DEFINITIONS "MP2P_HAS_SSE2=1")
    set_source_files_properties(src/Generator_view_vector_avx.cpp
      PROPERTIES
        COMPILE_FLAGS "/arch:AVX"
        COMPILE_DEFINITIONS "MP2P_HAS_AVX=1")
  else()
    # GCC / Clang
    set_source_files_properties(src/Generator_view_vector_sse2.cpp
      PROPERTIES
        COMPILE_FLAGS "-msse2"
        COMPILE_DEFINITIONS "MP2P_HAS_SSE2=1")
    set_source_files_properties(src/Generator_view_vector_avx.cpp
      PROPERTIES
        COMPILE_FLAGS "-mavx"
        COMPILE_DEFINITIONS "MP2P_HAS_AVX=1")
  endif()

  # Inform Generator.cpp (the dispatcher) which kernels are available.
  set_source_files_properties(src/Generator.cpp
    PROPERTIES COMPILE_DEFINITIONS "MP2P_HAS_SSE2=1;MP2P_HAS_AVX=1")
endif()


if (TBB_FOUND AND MP2PICP_USE_TBB)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MP2P_HAS_TBB)
  target_link_libraries(${PROJECT_NAME} PRIVATE TBB::tbb)
endif()

if (mola_imu_preintegration_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
endif()
