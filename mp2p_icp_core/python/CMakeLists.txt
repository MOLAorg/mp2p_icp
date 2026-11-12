# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2P) ICP C++ library
#
# Python bindings: the extension module `mp2p_icp._core` plus the pure Python
# package `mp2p_icp`. Silently skipped if pybind11 is not found.
#
# Options:
#  - MP2P_ICP_PYTHON_INSTALL_DIR: install dir of the `mp2p_icp` package,
#    relative to CMAKE_INSTALL_PREFIX. Default: lib/pythonX.Y/site-packages,
#    as colcon expects.
# ------------------------------------------------------------------------------

set(PYBIND11_FINDPYTHON ON)
# Development.Module: extension modules do not need libpython, which some
# Python builds (e.g. the manylinux images) do not ship.
find_package(Python3 QUIET COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG QUIET)
if (NOT pybind11_FOUND OR NOT Python3_FOUND)
  return()
endif()

if (NOT MP2P_ICP_PYTHON_INSTALL_DIR)
  set(MP2P_ICP_PYTHON_INSTALL_DIR
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif()

message(STATUS "mp2p_icp: Python bindings enabled (pybind11 ${pybind11_VERSION}), install dir: ${MP2P_ICP_PYTHON_INSTALL_DIR}")

pybind11_add_module(_core MODULE
  src/mp2p_icp_py.cpp
  src/py_common.cpp
  src/py_common.h
  src/py_filters.cpp
  src/py_icp.cpp
  src/py_map.cpp
)
mola_set_target_build_options(_core)
target_compile_definitions(_core PRIVATE MP2P_ICP_VERSION_STRING="${MP2P_ICP_VERSION}")
target_link_libraries(_core PRIVATE mp2p_icp_filters mp2p_icp mp2p_icp_map mp2p_icp_common)

# Stage the package (Python sources plus the extension) in the build tree, so
# it can be imported and tested before installing:
set(_staging_dir "${CMAKE_CURRENT_BINARY_DIR}/python_staging")
set_target_properties(_core PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${_staging_dir}/mp2p_icp")
add_custom_target(mp2p_icp_python_sources ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/mp2p_icp" "${_staging_dir}/mp2p_icp"
  COMMENT "Staging the mp2p_icp Python package"
)

install(TARGETS _core LIBRARY DESTINATION "${MP2P_ICP_PYTHON_INSTALL_DIR}/mp2p_icp")
install(DIRECTORY mp2p_icp DESTINATION "${MP2P_ICP_PYTHON_INSTALL_DIR}"
  PATTERN "__pycache__" EXCLUDE
)

# Tests (pytest), run against the staged package, if the test dependencies and
# the MRPT Python modules are available:
if (MP2PICP_BUILD_TESTING)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import pytest, numpy, mrpt.maps, mrpt.topography"
    RESULT_VARIABLE _pytest_result OUTPUT_QUIET ERROR_QUIET
  )
  if (_pytest_result EQUAL 0)
    add_test(
      NAME python_mp2p_icp
      COMMAND "${Python3_EXECUTABLE}" -m pytest -q -p no:cacheprovider "${CMAKE_CURRENT_SOURCE_DIR}/tests"
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )
    set_tests_properties(python_mp2p_icp PROPERTIES
      LABELS "python_bindings"
      ENVIRONMENT_MODIFICATION
        "PYTHONPATH=path_list_prepend:${_staging_dir};PYTHONDONTWRITEBYTECODE=set:1"
    )
  else()
    message(STATUS "mp2p_icp: pytest, numpy or the mrpt Python modules not found, Python tests disabled")
  endif()
endif()
