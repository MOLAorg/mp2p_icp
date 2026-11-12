# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2P) ICP C++ library
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco, contributors (AUTHORS)
# All rights reserved.
# Released under BSD 3-Clause License. See COPYING.
# ------------------------------------------------------------------------------

# Makes the mola_common CMake package installed into the SDK relocatable:
#  - Its config file adds the mola_common source dir, an absolute path of the
#    build machine, to CMAKE_MODULE_PATH: remove it (the config also adds its
#    own dir, which is all that is needed).
#  - Its version file has an empty version (".."): set the one in package.xml.
#
# Usage: cmake -DSDK_DIR=<install prefix> -DSOURCE_DIR=<mola_common sources>
#          -P relocate_mola_common.cmake

set(_dir "${SDK_DIR}/share/mola_common/cmake")

file(READ "${_dir}/mola_common-config.cmake" _cfg)
string(REGEX REPLACE "\nlist\\(APPEND CMAKE_MODULE_PATH /[^\n]*\\)" "" _cfg "${_cfg}")
file(WRITE "${_dir}/mola_common-config.cmake" "${_cfg}")

file(READ "${SOURCE_DIR}/package.xml" _xml)
if (NOT _xml MATCHES "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>")
  message(FATAL_ERROR "No version found in ${SOURCE_DIR}/package.xml")
endif()
set(_version "${CMAKE_MATCH_1}")
file(READ "${_dir}/mola_common-config-version.cmake" _ver)
string(REPLACE "set(PACKAGE_VERSION \"..\")" "set(PACKAGE_VERSION \"${_version}\")" _ver "${_ver}")
file(WRITE "${_dir}/mola_common-config-version.cmake" "${_ver}")
