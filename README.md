# `mp2p_icp`
A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++. 

License: New BSD 3-Clause

The OLAE-ICP method is described in:

```
Jose-Luis Blanco-Claraco. "OLAE-ICP: Robust and fast alignment of geometric
features with the optimal linear attitude estimator", Arxiv 2019.
```

![mp2p_pairings](docs/mp2p_pairings.png)

## Introduction

This library provides:
 * [`mp2p_icp::pointcloud_t`](https://docs.mola-slam.org/latest/): A generic
   data type to store raw or processed point clouds, e.g. segmented, discrete
   extracted features. Note that filtering point clouds is intentionally left
   outside of the scope of this library.
   See [MOLA](https://github.com/MOLAorg/mola) for possible implementations.
 * [`mp2p_icp::ICP_Base`](https://docs.mola-slam.org/latest/): A uniform API
   for matching those generic point clouds.
 * Implementations/wrappers of different ICP algorithms under such uniform API.
 * The library exposes both, complete iterative ICP algorithms, and the
 underlying optimal transformation estimators which are run at each ICP iteration.

## Implemented Optimal Transformation methods
 * `optimal_tf_olae()`: A novel algorithm that can recover the optimal attitude from a set
    of point-to-point, line-to-line, and plane-to-plane pairings.
 * `optimal_tf_horn()`: Classic Horn's closed-form optimal quaternion solution.
    Relies on the implementation in [`<mrpt/tfest/se3.h>`](http://mrpt.ual.es/reference/devel/group__mrpt__tfest__grp.html).
 * `optimal_gauss_newton()`: Simple non-linear optimizer to find the SE(3)
    optimal transformation for these pairings: point-to-point, point-to-plane.

## Implemented ICP methods

 * `ICP_OLAE`: ICP for point clouds, planes, and lines. Uses `optimal_tf_olae()`.
 * `ICP_Horn_MultiCloud`: Align point clouds layers, using classic Horn's
    closed-form optimal quaternion solution.

## Building

### Requisites
 * A C++17 compiler. Tested with gcc-7, MSVC 2017.
 * Eigen3
 * CMake >= 3.4
 * MRPT >=1.9.9

Install all the dependencies in Ubuntu with:

```
# MRPT >1.9.9, for now from this PPA (or build from sources if preferred):
sudo add-apt-repository ppa:joseluisblancoc/mrpt
sudo apt update
sudo apt install libmrpt-dev

# Rest of dependencies:
sudo apt install build-essential cmake libeigen3-dev
```

### Build

```
cmake -H. -Bbuild
cd build
cmake --build .   # or make
```

### Run the tests

```
make test
```
