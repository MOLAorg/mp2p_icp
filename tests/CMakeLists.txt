# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2) ICP C++ library
#
# Copyright (C) 2018-2019, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under FreeBSD 3. See LICENSE file
# ------------------------------------------------------------------------------

mola_add_executable(
    TARGET  tests-mp2_icp
	SOURCES tests-mp2_icp.cpp
	LINK_LIBRARIES
	    mp2_icp
)
add_test(P2P2_OLAE tests-mp2_icp)
