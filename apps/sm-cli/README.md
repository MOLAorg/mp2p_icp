# sm-cli

A CLI tool to inspect, visualize, or modify [simple maps](https://docs.mrpt.org/reference/latest/class_mrpt_maps_CSimpleMap.html) `*.simplemap` (from a SLAM mapping session).

```bash
$ sm-cli --help

Available commands:
    sm-cli cut                Cut part of a .simplemap file into a new file.
    sm-cli export-keyframes   Export KF poses as TUM format.
    sm-cli export-rawlog      Export KFs as rawlog for inspection.
    sm-cli info               Analyze a .simplemap file.
    sm-cli join               Join two or more .simplemap files into one.
    sm-cli level              Makes a .simplemap file level (horizontal).
    sm-cli trim               Extracts part of a .simplemap inside a given box.
    sm-cli --version          Shows program version.
    sm-cli --help             Shows this information.

Or use `sm-cli <COMMAND> --help` for further options
```
