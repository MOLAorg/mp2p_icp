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
    params:
      input_pointcloud_layer: 'points_map'
      inside_pointcloud_layer: 'map_section'   # Points inside the bbox
      #outside_pointcloud_layer: 'map_outside_section' # Points outside of the bbox
      bounding_box_min: [-200, -200, 0.10]
      bounding_box_max: [ 200,  200, 1.5]
```

## Generate a 2D gridmap from a 3D voxelmap

See: [demos/mm-filter_voxelmap_to_gridmap.yaml](../../demos/mm-filter_voxelmap_to_gridmap.yaml).

