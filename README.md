# `mp2_icp`

A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++. 

License: New BSD 3-Clause

## Introduction

This library provides:
 * A generic data type to store raw or processed point clouds, e.g. segmented, discrete extracted features.
   Note that filtering point clouds is intentionally left outside of the scope of this library.
   See [MOLA](https://github.com/MOLAorg/mola) for possible implementations.
 * The definition of a uniform API for matching those generic point clouds.
 * Implementations/wrappers of different ICP algorithms under such uniform API.

## Implemented ICP methods

 * OLAE-ICP: A novel algorithm that can recover the optimal attitude from a set
 of point-to-point, line-to-line, and plane-to-plane pairings.
 * MultiCloudICP: Align point clouds layers, using classic Horn's closed-form
 optimal quaternion solution.
 Relies on the implementation in [`<mrpt/tfest/se3.h>`](http://mrpt.ual.es/reference/devel/group__mrpt__tfest__grp.html).

## Building

Write me!

## References

The OLAE-ICP method is described in:

        Jose-Luis Blanco-Claraco. OLAE-ICP: Robust and fast alignment of geometric features with the optimal linear attitude estimator, Arxiv 2019.

