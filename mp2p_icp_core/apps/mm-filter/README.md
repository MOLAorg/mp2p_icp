# mm-filter

A CLI tool to apply a pipeline to an input metric map (`*.mm`), saving the result as another metric map file.

Refer to [mp2p_icp docs online](https://docs.mola-slam.org/latest/module-mp2p-icp.html) for the list of possible filters and their parameters.

# Examples

## Extracts a bounding box from a given point map

```yaml
# Save this in a file 'filter.yaml' and run with:
# mm-filter -i input.mm -o output.mm -p filter.yaml
#
filters:
  - class_name: mp2p_icp_filters::FilterBoundingBox
    #plugin: 'libcustom_filter.so' # Import additional custom user-defined classes (absolute path or search in LD_LIBRARY_PATH)
    params:
      input_pointcloud_layer: 'points_map'
      inside_pointcloud_layer: 'map_section'   # Points inside the bbox
      #outside_pointcloud_layer: 'map_outside_section' # Points outside of the bbox
      bounding_box_min: [-200, -200, 0.10]
      bounding_box_max: [ 200,  200, 1.5]
```

## Generate a 2D gridmap from a 3D voxelmap

See: [demos/mm-filter_voxelmap_to_gridmap.yaml](../../demos/mm-filter_voxelmap_to_gridmap.yaml).

## Create a new layer of an arbitrary map class, then merge points into it

Pipeline files may optionally include a `generators:` section (same syntax as in
[sm2mm pipelines](https://docs.mola-slam.org/latest/app_sm2mm.html)). Since `mm-filter`
runs over an already-built `.mm` file, there is no observation stream to feed the
generators with; instead, each generator that defines a `metric_map_definition` block is
used just once, to create its `target_layer` as an empty instance of that map class if it
does not already exist in the input map (existing layers are left untouched). Generators
without a `metric_map_definition` (the default point-cloud generation mode) are ignored.

This makes it possible to insert an existing point cloud layer into a new layer of a
custom `mrpt::maps::CMetricMap`-derived class, e.g. `mola::KeyframePointCloudMap` for
localization-only GICP, directly from `mm-filter`, without going through `sm2mm`:

See: [demos/mm-filter_create_keyframe_map_layer.yaml](../../demos/mm-filter_create_keyframe_map_layer.yaml).

