#!/bin/bash
# Build dependencies of the mp2p-icp wheel inside the manylinux_2_28 images
# (AlmaLinux 8), run once per image by cibuildwheel ("before-all").
# MRPT, Eigen and oneTBB come from the C++ SDK of the mrpt3 wheel, and
# mola_common is downloaded by the superbuild (CMakeLists.txt).
set -euo pipefail

dnf install -y epel-release
dnf install -y ccache
