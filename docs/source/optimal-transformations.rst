.. _optimal-transformations:

===========================
Optimal transformations
===========================


Gauss-Newton solver
--------------------
``mp2p_icp::optimal_gauss_newton()``: A non-linear iterative solver to find the SE(3)
optimal transformation given a set of pairings. 

Key features: 

- Supports multiple geometric primitives: point-to-point, point-to-plane, point-to-line, etc.
- Robust kernels.
- Optional a priori term.

TO-DO: Write a proper paper with the details!

Horn's quaternion solver
---------------------------
``mp2p_icp::optimal_tf_horn()``: The classic Horn's closed-form optimal quaternion solution.
Relies on the implementation in `<mrpt/tfest/se3.h> <http://mrpt.ual.es/reference/devel/group__mrpt__tfest__grp.html>`_.

OLAE solver
-----------------
``mp2p_icp::optimal_tf_olae()``: A novel algorithm that can recover the optimal attitude and translation from a set
of point-to-point, line-to-line, and plane-to-plane pairings.

Introduced in :cite:p:`blanco2018olae`.
