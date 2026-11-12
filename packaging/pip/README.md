# mp2p_icp for Python

Point cloud registration (ICP), metric maps and point cloud processing
pipelines from [mp2p_icp](https://github.com/MOLAorg/mp2p_icp), the C++
libraries behind the [MOLA](https://github.com/MOLAorg/mola) LiDAR odometry
and mapping framework.

```bash
pip install mp2p-icp
```

```python
import mp2p_icp

# Metric maps (*.mm files): named layers of point clouds and other maps
m = mp2p_icp.MetricMap.from_file("map.mm")
print(m)                                        # contents summary
fields = mp2p_icp.layer_to_numpy(m.layers["raw"])  # x, y, z, intensity, ring, t... as NumPy
print(fields["x"].shape, list(fields))

# New layers from NumPy arrays: (N,3) or a dict of per-point fields
keep = fields["z"] > 0.5
m.layers["high"] = {name: a[keep] for name, a in fields.items()}
m.save("out.mm")
```

Register two point clouds (here, PLY files) with a bundled ICP configuration
(`mp2p_icp.list_configs()`), or with any mp2p_icp YAML pipeline:

```python
local = mp2p_icp.load_ply("scan.ply")
reference = mp2p_icp.load_ply("map.ply")
res = mp2p_icp.align(local, reference, "3d-point2plane",
                     initial_guess=[0, 0, 0, 0, 0, 0],  # x y z yaw pitch roll
                     variables={"ICP_THRESHOLD": 1.0})
print(res.optimal_tf.mean, res.quality)
T = res.optimal_pose_matrix  # 4x4 NumPy array
```

Render a map into an image, off-screen (needs an OpenGL/EGL capable system,
but no display):

```python
img = mp2p_icp.render(m, 1280, 720, colormap="jet", output="map.png")  # (H,W,3) uint8
```

Also included: filter and generator pipelines (`FilterPipeline`,
`Generators`, `simplemap_to_metricmap()`), PLY, PCD, LAS and text point cloud
files, and conversions in `mp2p_icp.interop` to and from pandas, Parquet,
Open3D, PyVista and laspy. Their optional dependencies can be installed with
`pip install mp2p-icp[pandas]`, `[open3d]`, `[pyvista]`, `[las]`, or `[all]`.

Point clouds are [`mrpt3`](https://pypi.org/project/mrpt3/) objects
(`mrpt.maps.CGenericPointsMap`...) and poses are `mrpt.poses` / `mrpt.math`
types, so both packages work together. This wheel uses the MRPT libraries of
the installed `mrpt3` wheel, which is a dependency.

Requirements: Linux (glibc 2.28 or newer), x86_64 or aarch64, Python 3.10 to
3.14. `render()` also needs the system OpenGL libraries (see the `mrpt3`
package description).

See the [documentation](https://docs.mola-slam.org/latest/module-mp2p-icp.html)
and the [Python API guide](https://github.com/MOLAorg/mp2p_icp/blob/develop/docs/source/mp2p_icp_python.rst).

## Other ways to install mp2p_icp

mp2p_icp is also available as ROS 2 packages (`ros-$ROS_DISTRO-mp2p-icp`),
which include these Python bindings and the command line tools (`icp-run`,
`sm2mm`, `mm-viewer`...). Avoid mixing this wheel with those Python bindings in
the same environment.
