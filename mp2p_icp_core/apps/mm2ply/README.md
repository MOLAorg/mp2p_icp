# mm2ply

A command-line tool to export the layers of a MOLA metric map (*.mm) file as PLY point cloud files.

## Overview

`mm2ply` is part of the MOLA project. It converts metric map files into the widely-supported PLY format, making it easy to visualize and process point cloud data in various 3D viewers and processing tools.

## Features

- Exports all point cloud layers from a metric map file
- Selectively exports specific fields in a custom order
- Supports both ASCII and binary PLY formats
- Preserves all point fields (coordinates, colors, intensities, etc.)
- Automatic field name mapping for standard color attributes
- Generic field handling for custom point cloud attributes

## Installation

This tool is built as part of the MOLA framework. Please refer to the [MOLA documentation](https://github.com/MOLAorg/mola) for build and installation instructions.

## Usage

```bash
mm2ply -i <input.mm> [-o <output_prefix>] [-b] [--export-fields <field1,field2,...>]
```

### Arguments

- `-i, --input <file.mm>` (required): Input metric map file
- `-o, --output <prefix>` (optional): Prefix for output PLY files. If not specified, uses the input filename without extension
- `-b, --binary` (optional): Export in binary format instead of ASCII (default: ASCII)
- `--export-fields <field1,field2,...>` (optional): Comma-separated list of fields to export in the specified order. If not provided, all available fields will be exported. Spaces around commas are allowed

### Examples

Export a metric map to ASCII PLY files:

```bash
mm2ply -i mymap.mm
```

Export with a custom output prefix:

```bash
mm2ply -i mymap.mm -o processed/map
```

Export in binary format for smaller file sizes:

```bash
mm2ply -i mymap.mm -b
```

Export only specific fields in a custom order:

```bash
mm2ply -i mymap.mm --export-fields "x,y,z,intensity"
```

Export selected fields in binary format:

```bash
mm2ply -i mymap.mm -b --export-fields "x, y, z, color_r, color_g, color_b"
```

Export only 2D coordinates and intensity:

```bash
mm2ply -i mymap.mm --export-fields "x,y,intensity"
```

## Output

The tool creates separate PLY files for each point cloud layer in the metric map. Files are named using the pattern:

```
<prefix>_<layer_name>.ply
```

For example, if your map contains layers named "raw" and "filtered", you'll get:
- `mymap_raw.ply`
- `mymap_filtered.ply`

## Field Selection

When using the `--export-fields` option:

- Fields must be specified as a comma-separated list
- Spaces around commas are allowed (e.g., `"x, y, z"` or `"x,y,z"`)
- Fields will be exported in the exact order specified
- All specified fields must exist in the point cloud, or an error will be raised
- The tool validates field availability and reports available fields if a requested field is not found
- Color fields (`color_r`, `color_g`, `color_b`) are automatically mapped to PLY standard names (`red`, `green`, `blue`) in the output file

## Supported Point Fields

The tool automatically exports all point attributes, including:

- **Coordinates**: x, y, z (float)
- **Colors**: Automatically maps `color_r/color_rf` → `red`, `color_g/color_gf` → `green`, `color_b/color_bf` → `blue`
- **Custom float fields**: Exported as float properties (e.g., intensity, normals, curvature)
- **Custom double fields**: Exported as double properties (e.g., time, timestamps) (MRPT ≥ 2.15.3)
- **Custom uint16 fields**: Exported as ushort properties (e.g., ring, color channels) (MRPT ≥ 2.15.3)
- **Custom uint8 fields**: Exported as uchar properties (MRPT ≥ 2.15.3)

The specific fields exported depend on the point cloud type in each layer. Use `--export-fields` to select only the fields you need.