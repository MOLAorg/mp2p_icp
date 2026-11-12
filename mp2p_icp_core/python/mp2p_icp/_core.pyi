"""
Python bindings of the mp2p_icp C++ libraries (use the `mp2p_icp` package)
"""
from __future__ import annotations
import collections.abc
import mrpt.img
import mrpt.maps
import mrpt.math
import mrpt.poses
import mrpt.serialization
import mrpt.topography
import mrpt.viz
import typing
__all__: list[str] = ['ColorMode', 'FilterPipeline', 'Generators', 'Georeferencing', 'GravityPrior', 'ICP', 'IterTermReason', 'IterationDetails', 'IterationInfo', 'LayersView', 'LogRecord', 'MetricMap', 'PT_LAYER_PLANE_CENTROIDS', 'PT_LAYER_RAW', 'Pairings', 'Parameters', 'PlanePatch', 'RenderParams', 'RenderParamsLines', 'RenderParamsPlanes', 'RenderParamsPointLayer', 'RenderParamsPoints', 'Results', 'apply_filter_pipeline', 'apply_generators', 'filter_pipeline_from_yaml', 'generators_from_yaml', 'icp_pipeline_from_yaml', 'layer_from_numpy', 'layer_points_view', 'layer_to_numpy', 'layer_visualization', 'load_plugins', 'load_xyz_file', 'load_yaml', 'save_las', 'save_ply', 'save_txt', 'simplemap_to_metricmap']
class Georeferencing:
    """
    Placement of a metric map with respect to the Earth: the geodetic coordinates of the ENU frame origin, and the pose of the map frame in that ENU frame.
    """
    @staticmethod
    def from_dict(d: typing.Any) -> Georeferencing | None:
        """
        Builds a georeferencing from a dict, as returned by as_dict().
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, geo_coord: mrpt.topography.TGeodeticCoords, T_enu_to_map: mrpt.poses.CPose3DPDFGaussian) -> None:
        """
        Constructor from its two fields.
        """
    def __repr__(self) -> str:
        ...
    def as_dict(self) -> typing.Any:
        """
        Returns the georeferencing as a dict, in the YAML format used by mp2p_icp tools.
        """
    @property
    def T_enu_to_map(self) -> mrpt.poses.CPose3DPDFGaussian:
        """
        Pose of the map frame in the ENU frame (mrpt.poses.CPose3DPDFGaussian)
        """
    @T_enu_to_map.setter
    def T_enu_to_map(self, arg0: mrpt.poses.CPose3DPDFGaussian) -> None:
        ...
    @property
    def geo_coord(self) -> mrpt.topography.TGeodeticCoords:
        """
        Geodetic coordinates (WGS-84) of the ENU frame origin (mrpt.topography.TGeodeticCoords)
        """
    @geo_coord.setter
    def geo_coord(self, arg0: mrpt.topography.TGeodeticCoords) -> None:
        ...
class PlanePatch:
    """
    A plane patch: an infinite plane, plus the centroid of its points.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, plane: mrpt.math.TPlane, centroid: mrpt.math.TPoint3D) -> None:
        """
        Constructor from a plane and its centroid.
        """
    def __repr__(self) -> str:
        ...
    @property
    def area(self) -> float:
        """
        Surface area [m2], 0 if unknown
        """
    @area.setter
    def area(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def centroid(self) -> mrpt.math.TPoint3D:
        """
        Centroid (mrpt.math.TPoint3D)
        """
    @centroid.setter
    def centroid(self, arg0: mrpt.math.TPoint3D) -> None:
        ...
    @property
    def num_points(self) -> int:
        """
        Number of points the patch was fitted from, 0 if unknown
        """
    @num_points.setter
    def num_points(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def plane(self) -> mrpt.math.TPlane:
        """
        The plane (mrpt.math.TPlane)
        """
    @plane.setter
    def plane(self, arg0: mrpt.math.TPlane) -> None:
        ...
class ColorMode:
    """
    Point cloud coloring mode, for RenderParamsPointLayer.colorMode
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def autoBoundingBoxOutliersPercentile(self) -> float | None:
        """
        Optional percentile of outliers ignored to compute the field limits
        """
    @autoBoundingBoxOutliersPercentile.setter
    def autoBoundingBoxOutliersPercentile(self, arg0: typing.SupportsFloat | typing.SupportsIndex | None) -> None:
        ...
    @property
    def colorMap(self) -> mrpt.img.TColormap:
        """
        Color map (mrpt.img.TColormap)
        """
    @colorMap.setter
    def colorMap(self, arg0: mrpt.img.TColormap) -> None:
        ...
    @property
    def colorMapMaxCoord(self) -> float | None:
        """
        Optional fixed field value mapped to the color map end
        """
    @colorMapMaxCoord.setter
    def colorMapMaxCoord(self, arg0: typing.SupportsFloat | typing.SupportsIndex | None) -> None:
        ...
    @property
    def colorMapMinCoord(self) -> float | None:
        """
        Optional fixed field value mapped to the color map start
        """
    @colorMapMinCoord.setter
    def colorMapMinCoord(self, arg0: typing.SupportsFloat | typing.SupportsIndex | None) -> None:
        ...
    @property
    def keep_original_cloud_color(self) -> bool:
        """
        Use the RGB or intensity channels of the cloud, if present
        """
    @keep_original_cloud_color.setter
    def keep_original_cloud_color(self, arg0: bool) -> None:
        ...
    @property
    def recolorizeByField(self) -> str | None:
        """
        If set, the point field used to colorize points (e.g. 'z')
        """
    @recolorizeByField.setter
    def recolorizeByField(self, arg0: str | None) -> None:
        ...
class RenderParamsPointLayer:
    """
    Rendering options for one point layer
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def backFacingMaxAngleNormals_deg(self) -> float:
        """
        For hideBackFacing, with normals [deg]
        """
    @backFacingMaxAngleNormals_deg.setter
    def backFacingMaxAngleNormals_deg(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def backFacingMaxAngleView_deg(self) -> float:
        """
        For hideBackFacing, with view vectors [deg]
        """
    @backFacingMaxAngleView_deg.setter
    def backFacingMaxAngleView_deg(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def color(self) -> mrpt.img.TColor:
        """
        Fixed color for all points (mrpt.img.TColor). Ignored if colorMode is set.
        """
    @color.setter
    def color(self, arg0: mrpt.img.TColor) -> None:
        ...
    @property
    def colorMode(self) -> ColorMode | None:
        """
        Optional ColorMode, which overrides `color`
        """
    @colorMode.setter
    def colorMode(self, arg0: ColorMode | None) -> None:
        ...
    @property
    def force_alpha_channel(self) -> bool:
        """
        Overwrite the alpha channel, even for RGBA clouds
        """
    @force_alpha_channel.setter
    def force_alpha_channel(self, arg0: bool) -> None:
        ...
    @property
    def hideBackFacing(self) -> bool:
        """
        Hide points seen from behind (needs normal or view-vector fields)
        """
    @hideBackFacing.setter
    def hideBackFacing(self, arg0: bool) -> None:
        ...
    @property
    def pointSize(self) -> float:
        """
        Point size, in pixels
        """
    @pointSize.setter
    def pointSize(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def render_voxelmaps_as_points(self) -> bool:
        """
        Render voxel maps as points instead of cubes
        """
    @render_voxelmaps_as_points.setter
    def render_voxelmaps_as_points(self, arg0: bool) -> None:
        ...
    @property
    def render_voxelmaps_free_space(self) -> bool:
        """
        Render the free space of voxel maps
        """
    @render_voxelmaps_free_space.setter
    def render_voxelmaps_free_space(self, arg0: bool) -> None:
        ...
class RenderParamsPoints:
    """
    Rendering options for the point layers
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def allLayers(self) -> RenderParamsPointLayer:
        """
        Options for all layers, used if perLayer is empty
        """
    @allLayers.setter
    def allLayers(self, arg0: RenderParamsPointLayer) -> None:
        ...
    @property
    def perLayer(self) -> dict[str, RenderParamsPointLayer]:
        """
        If not empty, only these layers are rendered, each with its options. It is returned as a copy: assign a whole dict to change it.
        """
    @perLayer.setter
    def perLayer(self, arg0: collections.abc.Mapping[str, RenderParamsPointLayer]) -> None:
        ...
    @property
    def visible(self) -> bool:
        """
        Render points at all
        """
    @visible.setter
    def visible(self, arg0: bool) -> None:
        ...
class RenderParamsPlanes:
    """
    Rendering options for planes
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def color(self) -> mrpt.img.TColor:
        """
        mrpt.img.TColor
        """
    @color.setter
    def color(self, arg0: mrpt.img.TColor) -> None:
        ...
    @property
    def gridSpacing(self) -> float:
        """
        Grid spacing [m]
        """
    @gridSpacing.setter
    def gridSpacing(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def halfWidth(self) -> float:
        """
        Half width [m]
        """
    @halfWidth.setter
    def halfWidth(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def visible(self) -> bool:
        """
        Render planes
        """
    @visible.setter
    def visible(self, arg0: bool) -> None:
        ...
class RenderParamsLines:
    """
    Rendering options for lines
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def color(self) -> mrpt.img.TColor:
        """
        mrpt.img.TColor
        """
    @color.setter
    def color(self, arg0: mrpt.img.TColor) -> None:
        ...
    @property
    def length(self) -> float:
        """
        Length of all lines [m]
        """
    @length.setter
    def length(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def lengths(self) -> list[float] | None:
        """
        Optional individual lengths [m]
        """
    @lengths.setter
    def lengths(self, arg0: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex] | None) -> None:
        ...
    @property
    def visible(self) -> bool:
        """
        Render lines
        """
    @visible.setter
    def visible(self, arg0: bool) -> None:
        ...
class RenderParams:
    """
    Rendering options for MetricMap.get_visualization()
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def lines(self) -> RenderParamsLines:
        """
        RenderParamsLines
        """
    @lines.setter
    def lines(self, arg0: RenderParamsLines) -> None:
        ...
    @property
    def planes(self) -> RenderParamsPlanes:
        """
        RenderParamsPlanes
        """
    @planes.setter
    def planes(self, arg0: RenderParamsPlanes) -> None:
        ...
    @property
    def points(self) -> RenderParamsPoints:
        """
        RenderParamsPoints
        """
    @points.setter
    def points(self, arg0: RenderParamsPoints) -> None:
        ...
class MetricMap(mrpt.serialization.CSerializable):
    """
    Container of named layers (point clouds, voxel maps, grid maps...), lines and planes, with optional id, label, metadata and georeferencing. This is mp2p_icp::metric_map_t, stored in `.mm` files.
    """
    @staticmethod
    def from_file(path: typing.Any) -> MetricMap:
        """
        Loads a map from a `.mm` file. Raises an exception on any error.
        """
    def __copy__(self) -> MetricMap:
        """
        Shallow copy: a new map whose layers are the same objects as in this one.
        """
    def __deepcopy__(self, memo: typing.Any) -> MetricMap:
        ...
    def __getstate__(self) -> bytes:
        ...
    def __init__(self) -> None:
        """
        Creates an empty map.
        """
    def __repr__(self) -> str:
        ...
    def __setstate__(self, arg0: bytes) -> None:
        ...
    def __str__(self) -> str:
        ...
    def clear(self) -> None:
        """
        Clears all contents.
        """
    def contents_summary(self) -> str:
        """
        Returns a string summarizing the map contents.
        """
    def copy(self) -> MetricMap:
        """
        Returns a deep copy of the map, with copies of all its layers.
        """
    def empty(self) -> bool:
        """
        True if all layers, lines and planes are empty.
        """
    def get_visualization(self, params: RenderParams | None = None) -> mrpt.viz.CSetOfObjects:
        """
        Returns an mrpt.viz.CSetOfObjects with a 3D view of the map, with optional RenderParams.
        """
    def load(self, path: typing.Any) -> None:
        """
        Loads the map from a `.mm` file, replacing its contents. Raises an exception on any error.
        """
    def merge_with(self, other: MetricMap, other_relative_pose: typing.Any = None) -> None:
        """
        Merges the contents of another map into this one, with an optional relative pose of `other` (any pose accepted by the ICP functions). Point layers with the same name are merged; others are added.
        """
    def point_layer(self, name: str) -> mrpt.maps.CPointsMap:
        """
        Returns a layer as an mrpt.maps.CPointsMap, or raises if it is not a point cloud.
        """
    def save(self, path: typing.Any, compression: str = 'zstd', level: typing.Any = None) -> None:
        """
        Saves the map to a `.mm` file. `compression`: 'zstd' (default), 'gzip' or 'none'; `level`: optional compression level.
        """
    def size(self) -> int:
        """
        Overall number of elements (points, voxels, lines, planes).
        """
    def size_points_only(self) -> int:
        """
        Overall number of points, including all layers.
        """
    @property
    def georeferencing(self) -> Georeferencing | None:
        """
        Optional Georeferencing. It is returned as a copy: assign it to change it.
        """
    @georeferencing.setter
    def georeferencing(self, arg0: Georeferencing | None) -> None:
        ...
    @property
    def id(self) -> int | None:
        """
        Optional numeric ID (int or None)
        """
    @id.setter
    def id(self, arg0: typing.SupportsInt | typing.SupportsIndex | None) -> None:
        ...
    @property
    def label(self) -> str | None:
        """
        Optional label (str or None)
        """
    @label.setter
    def label(self, arg0: str | None) -> None:
        ...
    @property
    def layers(self) -> LayersView:
        """
        Dict-like view of the map layers (mrpt.maps.CMetricMap), by name.
        """
    @property
    def lines(self) -> list[mrpt.math.TLine3D]:
        """
        3D lines, as a list of mrpt.math.TLine3D. It is returned as a copy: assign a whole list to change it.
        """
    @lines.setter
    def lines(self, arg0: collections.abc.Sequence[mrpt.math.TLine3D]) -> None:
        ...
    @property
    def metadata(self) -> typing.Any:
        """
        User metadata, as a dict. It is returned as a copy: assign a whole dict to change it.
        """
    @metadata.setter
    def metadata(self, arg1: typing.Any) -> None:
        ...
    @property
    def planes(self) -> list[PlanePatch]:
        """
        Plane patches, as a list of PlanePatch. It is returned as a copy: assign a whole list to change it.
        """
    @planes.setter
    def planes(self, arg0: collections.abc.Sequence[PlanePatch]) -> None:
        ...
class LayersView:
    """
    Dict-like view of the layers of a MetricMap: names (str) to mrpt.maps.CMetricMap. Assigning a NumPy array (N,3) or a dict of field arrays creates a CGenericPointsMap.
    """
    def __contains__(self, name: str) -> bool:
        ...
    def __delitem__(self, name: str) -> None:
        ...
    def __getitem__(self, name: str) -> mrpt.maps.CMetricMap:
        ...
    def __iter__(self) -> typing.Any:
        ...
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def __setitem__(self, name: str, layer: typing.Any) -> None:
        ...
    def get(self, name: str, default: typing.Any = None) -> typing.Any:
        """
        Returns a layer, or `default` if it does not exist.
        """
    def items(self) -> list[tuple[str, mrpt.maps.CMetricMap]]:
        """
        Returns a list of (name, layer) tuples.
        """
    def keys(self) -> list[str]:
        """
        Returns a list with the layer names.
        """
    def values(self) -> list[mrpt.maps.CMetricMap]:
        """
        Returns a list with the layers.
        """
class IterTermReason:
    """
    Reason why ICP iterations ended
    
    Members:
    
      Undefined
    
      NoPairings
    
      SolverError
    
      MaxIterations
    
      Stalled
    
      QualityCheckpointFailed
    
      HookRequest
    """
    HookRequest: typing.ClassVar[IterTermReason]
    MaxIterations: typing.ClassVar[IterTermReason]
    NoPairings: typing.ClassVar[IterTermReason]
    QualityCheckpointFailed: typing.ClassVar[IterTermReason]
    SolverError: typing.ClassVar[IterTermReason]
    Stalled: typing.ClassVar[IterTermReason]
    Undefined: typing.ClassVar[IterTermReason]
    __members__: typing.ClassVar[dict[str, IterTermReason]]
    @typing.overload
    def __eq__(self, other: IterTermReason) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: IterTermReason) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Parameters(mrpt.serialization.CSerializable):
    """
    ICP parameters: termination criteria, debug files, quality checkpoints
    """
    @staticmethod
    def from_dict(d: typing.Any) -> Parameters:
        """
        Builds parameters from a dict, with the format of the `params` block of ICP YAML files.
        """
    def __copy__(self) -> Parameters:
        ...
    def __deepcopy__(self, memo: typing.Any) -> Parameters:
        ...
    def __init__(self) -> None:
        """
        Default parameters.
        """
    def __repr__(self) -> str:
        ...
    def as_dict(self) -> typing.Any:
        """
        Returns the parameters as a dict.
        """
    @property
    def debugFileNameFormat(self) -> str:
        """
        File name pattern of the debug files
        """
    @debugFileNameFormat.setter
    def debugFileNameFormat(self, arg0: str) -> None:
        ...
    @property
    def debugPrintIterationProgress(self) -> bool:
        """
        Print the progress of each iteration
        """
    @debugPrintIterationProgress.setter
    def debugPrintIterationProgress(self, arg0: bool) -> None:
        ...
    @property
    def decimationDebugFiles(self) -> int:
        """
        With generateDebugFiles, save 1 out of N log files
        """
    @decimationDebugFiles.setter
    def decimationDebugFiles(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def decimationIterationDetails(self) -> int:
        """
        With saveIterationDetails, keep 1 out of N iterations
        """
    @decimationIterationDetails.setter
    def decimationIterationDetails(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def force_final_pairings_for_quality(self) -> bool:
        """
        Run a last matching pass at the final thresholds before evaluating quality
        """
    @force_final_pairings_for_quality.setter
    def force_final_pairings_for_quality(self, arg0: bool) -> None:
        ...
    @property
    def freezePairingsAfterIteration(self) -> int:
        """
        If >0, pairings are not recomputed after this iteration
        """
    @freezePairingsAfterIteration.setter
    def freezePairingsAfterIteration(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def generateDebugFiles(self) -> bool:
        """
        Save a .icplog file for each align() call
        """
    @generateDebugFiles.setter
    def generateDebugFiles(self, arg0: bool) -> None:
        ...
    @property
    def maxIterations(self) -> int:
        """
        Maximum number of ICP iterations
        """
    @maxIterations.setter
    def maxIterations(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def minAbsStep_rot(self) -> float:
        """
        Stop when all rotation corrections are below this [rad]
        """
    @minAbsStep_rot.setter
    def minAbsStep_rot(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def minAbsStep_trans(self) -> float:
        """
        Stop when all translation corrections are below this [m]
        """
    @minAbsStep_trans.setter
    def minAbsStep_trans(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def quality_checkpoints(self) -> dict[int, float]:
        """
        dict {iteration: minimum quality}: abort if the quality is lower at that iteration
        """
    @quality_checkpoints.setter
    def quality_checkpoints(self, arg0: collections.abc.Mapping[typing.SupportsInt | typing.SupportsIndex, typing.SupportsFloat | typing.SupportsIndex]) -> None:
        ...
    @property
    def saveIterationDetails(self) -> bool:
        """
        Store the pairings and pose of each iteration in the LogRecord
        """
    @saveIterationDetails.setter
    def saveIterationDetails(self, arg0: bool) -> None:
        ...
class Pairings:
    """
    Pairings between geometric entities of two maps
    """
    def __init__(self) -> None:
        """
        Empty pairings.
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def contents_summary(self) -> str:
        """
        Summary of the pairings
        """
    def empty(self) -> bool:
        """
        True if there are no pairings
        """
    def get_visualization(self, local_wrt_global: typing.Any = None) -> mrpt.viz.CSetOfObjects:
        """
        Returns an mrpt.viz.CSetOfObjects with the pairings, for the given pose of the local map
        """
    def pt2pt_to_numpy(self) -> dict:
        """
        Returns the point-to-point pairings as a dict of NumPy arrays: 'global' and 'local' (N,3) point coordinates, and 'global_idx' and 'local_idx' point indices.
        """
    def size(self) -> int:
        """
        Overall number of pairings
        """
    @property
    def num_cov2cov(self) -> int:
        """
        Number of covariance-to-covariance pairings
        """
    @property
    def num_ln2ln(self) -> int:
        """
        Number of line-to-line pairings
        """
    @property
    def num_pl2pl(self) -> int:
        """
        Number of plane-to-plane pairings
        """
    @property
    def num_pt2ln(self) -> int:
        """
        Number of point-to-line pairings
        """
    @property
    def num_pt2pl(self) -> int:
        """
        Number of point-to-plane pairings
        """
    @property
    def num_pt2pt(self) -> int:
        """
        Number of point-to-point pairings
        """
    @property
    def potential_pairings(self) -> int:
        """
        Maximum number of pairings the matchers could have found
        """
class Results:
    """
    Results of ICP.align()
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    @property
    def finalPairings(self) -> Pairings:
        """
        Pairings of the last iteration
        """
    @finalPairings.setter
    def finalPairings(self, arg0: Pairings) -> None:
        ...
    @property
    def gravity_information_share(self) -> float:
        """
        Share of the rotational information from the gravity prior, <0 if none
        """
    @gravity_information_share.setter
    def gravity_information_share(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def nIterations(self) -> int:
        """
        Number of executed iterations
        """
    @nIterations.setter
    def nIterations(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def optimalScale(self) -> float:
        """
        Scale, if estimated
        """
    @optimalScale.setter
    def optimalScale(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def optimal_pose_matrix(self) -> typing.Any:
        """
        The mean of optimal_tf as a 4x4 homogeneous matrix (NumPy array)
        """
    @property
    def optimal_tf(self) -> mrpt.poses.CPose3DPDFGaussian:
        """
        Pose of the local map with respect to the global one, with its covariance (mrpt.poses.CPose3DPDFGaussian)
        """
    @optimal_tf.setter
    def optimal_tf(self, arg0: mrpt.poses.CPose3DPDFGaussian) -> None:
        ...
    @property
    def quality(self) -> float:
        """
        Registration quality, in [0,1]
        """
    @quality.setter
    def quality(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def terminationReason(self) -> IterTermReason:
        """
        Why iterations ended
        """
    @terminationReason.setter
    def terminationReason(self, arg0: IterTermReason) -> None:
        ...
class GravityPrior:
    """
    Gravity ('verticality') observation for ICP: constrains roll and pitch only
    """
    def __init__(self) -> None:
        """
        Default: up=(0,0,1) in both frames, sigma 0.02 rad.
        """
    @property
    def sigma_rad(self) -> float:
        """
        Tilt standard deviation [rad]
        """
    @sigma_rad.setter
    def sigma_rad(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def up_body(self) -> mrpt.math.TPoint3D:
        """
        Measured 'up' direction in the local (body) frame (mrpt.math.TPoint3D)
        """
    @up_body.setter
    def up_body(self, arg0: mrpt.math.TPoint3D) -> None:
        ...
    @property
    def up_map(self) -> mrpt.math.TPoint3D:
        """
        'up' direction in the global map frame
        """
    @up_map.setter
    def up_map(self, arg0: mrpt.math.TPoint3D) -> None:
        ...
class IterationInfo:
    """
    Information passed to the ICP iteration hook, see ICP.set_iteration_hook()
    """
    def __repr__(self) -> str:
        ...
    @property
    def iteration(self) -> int:
        """
        Iteration index, from 0
        """
    @property
    def num_pairings(self) -> int:
        """
        Number of pairings in this iteration
        """
    @property
    def pose(self) -> mrpt.poses.CPose3D:
        """
        Current pose estimate (mrpt.poses.CPose3D)
        """
    @property
    def potential_pairings(self) -> int:
        """
        Maximum number of pairings the matchers could have found
        """
class IterationDetails:
    """
    Pose and pairings of one ICP iteration, in a LogRecord
    """
    @property
    def optimalPose(self) -> mrpt.poses.CPose3D:
        """
        mrpt.poses.CPose3D
        """
    @optimalPose.setter
    def optimalPose(self, arg0: mrpt.poses.CPose3D) -> None:
        ...
    @property
    def pairings(self) -> Pairings:
        """
        Pairings
        """
    @pairings.setter
    def pairings(self, arg0: Pairings) -> None:
        ...
class LogRecord(mrpt.serialization.CSerializable):
    """
    Inputs and outputs of one ICP run, as saved in `.icplog` files (see Parameters.generateDebugFiles), or filled by ICP.align(..., log=LogRecord()).
    """
    @staticmethod
    def from_file(path: typing.Any) -> LogRecord:
        """
        Loads an `.icplog` file. Raises an exception on any error.
        """
    def __init__(self) -> None:
        """
        Empty record.
        """
    def __repr__(self) -> str:
        ...
    def save(self, path: typing.Any) -> None:
        """
        Saves the record to an `.icplog` file.
        """
    @property
    def dynamicVariables(self) -> dict[str, float]:
        """
        Values of the dynamic variables (dict)
        """
    @dynamicVariables.setter
    def dynamicVariables(self, arg0: collections.abc.Mapping[str, typing.SupportsFloat | typing.SupportsIndex]) -> None:
        ...
    @property
    def gravityPrior(self) -> GravityPrior | None:
        """
        Optional GravityPrior
        """
    @gravityPrior.setter
    def gravityPrior(self, arg0: GravityPrior | None) -> None:
        ...
    @property
    def icpParameters(self) -> Parameters:
        """
        ICP Parameters
        """
    @icpParameters.setter
    def icpParameters(self, arg0: Parameters) -> None:
        ...
    @property
    def icpResult(self) -> Results:
        """
        ICP Results
        """
    @icpResult.setter
    def icpResult(self, arg0: Results) -> None:
        ...
    @property
    def initialGuessLocalWrtGlobal(self) -> mrpt.math.TPose3D:
        """
        Initial guess (mrpt.math.TPose3D)
        """
    @initialGuessLocalWrtGlobal.setter
    def initialGuessLocalWrtGlobal(self, arg0: mrpt.math.TPose3D) -> None:
        ...
    @property
    def iterationsDetails(self) -> dict[int, IterationDetails] | None:
        """
        Optional dict {iteration: IterationDetails}, if Parameters.saveIterationDetails
        """
    @iterationsDetails.setter
    def iterationsDetails(self, arg0: collections.abc.Mapping[typing.SupportsInt | typing.SupportsIndex, IterationDetails] | None) -> None:
        ...
    @property
    def pcGlobal(self) -> MetricMap:
        """
        The global map (MetricMap or None)
        """
    @pcGlobal.setter
    def pcGlobal(self, arg1: MetricMap) -> None:
        ...
    @property
    def pcLocal(self) -> MetricMap:
        """
        The local map (MetricMap or None)
        """
    @pcLocal.setter
    def pcLocal(self, arg1: MetricMap) -> None:
        ...
    @property
    def prior(self) -> mrpt.poses.CPose3DPDFGaussianInf | None:
        """
        Optional pose prior (mrpt.poses.CPose3DPDFGaussianInf)
        """
    @prior.setter
    def prior(self, arg0: mrpt.poses.CPose3DPDFGaussianInf | None) -> None:
        ...
class ICP:
    """
    An ICP pipeline (matchers, solvers, quality evaluators) with its default parameters. Create it with ICP.from_yaml() or icp_pipeline_from_yaml(). Not thread-safe: do not call align() on the same object from several threads at once.
    """
    @staticmethod
    def from_yaml(config: typing.Any, variables: typing.Any = None, verbosity: str = 'INFO') -> ICP:
        """
        Creates an ICP pipeline from a configuration: a YAML file path, a dict, or an mrpt.containers.YAML node, with the format of mp2p_icp ICP YAML files. `${VAR}` and `${VAR|default}` tokens are expanded from `variables` (dict) and the environment.
        """
    def __repr__(self) -> str:
        ...
    def align(self, local_map: MetricMap, global_map: MetricMap, initial_guess: typing.Any = None, parameters: Parameters = None, prior: mrpt.poses.CPose3DPDFGaussianInf | None = None, gravity_prior: GravityPrior | None = None, log: LogRecord = None) -> Results:
        """
        Aligns two maps: finds the pose of `local_map` with respect to `global_map`, starting from `initial_guess` (TPose3D, CPose3D, 4x4 matrix, or [x,y,z,yaw,pitch,roll]; default: identity). `parameters` overrides the default ones. `prior` is an optional mrpt.poses.CPose3DPDFGaussianInf (information in the SE(3) tangent space of its mean), `gravity_prior` an optional GravityPrior. If a LogRecord is passed as `log`, it is filled with the inputs and outputs of this run. Returns Results.
        """
    def set_iteration_hook(self, hook: typing.Any) -> None:
        """
        Sets a function called at each ICP iteration with an IterationInfo; if it returns True, iterations stop (terminationReason=HookRequest). Exceptions raised in it stop align() and are re-raised. Pass None to remove it.
        """
    def update_variables(self, variables: collections.abc.Mapping[str, typing.SupportsFloat | typing.SupportsIndex]) -> None:
        """
        Sets the values of variables used in parameter formulas of the pipeline modules (e.g. `threshold: '2.0*SCALE'`). ICP_ITERATION is always defined by align().
        """
    @property
    def class_name(self) -> str:
        """
        C++ class of the ICP algorithm
        """
    @property
    def parameters(self) -> Parameters:
        """
        Default Parameters for align(), loaded from the `params` block of the configuration.
        """
    @parameters.setter
    def parameters(self, arg1: Parameters) -> None:
        ...
    @property
    def variables(self) -> dict[str, float]:
        """
        Current values of the formula variables (dict)
        """
class FilterPipeline:
    """
    A sequence of mp2p_icp_filters filters, applied in order and in place to a MetricMap. Create it with filter_pipeline_from_yaml().
    """
    @staticmethod
    def from_yaml(config: typing.Any, variables: typing.Any = None, verbosity: str = 'INFO') -> FilterPipeline:
        """
        Creates a filter pipeline from a YAML file path, a dict, or an mrpt.containers.YAML node: either a list of filters (each with `class_name` and `params`), or a map with a `filters` list. `${VAR}` tokens are expanded from `variables` and the environment.
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def apply(self, map: MetricMap) -> None:
        """
        Applies all filters, in order, to the map (in place).
        """
    def update_variables(self, variables: collections.abc.Mapping[str, typing.SupportsFloat | typing.SupportsIndex]) -> None:
        """
        Sets the values of variables used in parameter formulas of the filters, e.g. `max_range: '2*RANGE'`.
        """
    @property
    def filters(self) -> list[tuple[str, str]]:
        """
        List of (class_name, name) tuples, one per filter
        """
    @property
    def variables(self) -> dict[str, float]:
        """
        Current values of the formula variables (dict)
        """
class Generators:
    """
    A set of mp2p_icp_filters generators, which convert MRPT observations into MetricMap layers. Create it with generators_from_yaml().
    """
    @staticmethod
    def from_yaml(config: typing.Any, variables: typing.Any = None, verbosity: str = 'INFO') -> Generators:
        """
        Creates generators from a YAML file path, a dict, or an mrpt.containers.YAML node: either a list of generators (each with `class_name` and `params`), or a map with a `generators` list.
        """
    def __len__(self) -> int:
        ...
    def apply(self, observation: typing.Any, robot_pose: typing.Any = None, output: MetricMap = None) -> MetricMap:
        """
        Converts an mrpt.obs.CObservation or CSensoryFrame into map layers, with an optional robot pose. The layers are added to `output` if given, or to a new MetricMap. Returns the map.
        """
    def update_variables(self, variables: collections.abc.Mapping[str, typing.SupportsFloat | typing.SupportsIndex]) -> None:
        """
        Sets the values of variables used in parameter formulas.
        """
def apply_filter_pipeline(pipeline: FilterPipeline, map: MetricMap) -> None:
    """
    Applies a FilterPipeline to a MetricMap, in place.
    """
def apply_generators(generators: Generators, observation: typing.Any, robot_pose: typing.Any = None, output: MetricMap = None) -> MetricMap:
    """
    Applies Generators to an observation, see Generators.apply().
    """
def filter_pipeline_from_yaml(config: typing.Any, variables: typing.Any = None, verbosity: str = 'INFO') -> FilterPipeline:
    """
    Creates a FilterPipeline, see FilterPipeline.from_yaml().
    """
def generators_from_yaml(config: typing.Any, variables: typing.Any = None, verbosity: str = 'INFO') -> Generators:
    """
    Creates Generators, see Generators.from_yaml().
    """
def icp_pipeline_from_yaml(config: typing.Any, variables: typing.Any = None, verbosity: str = 'INFO') -> tuple[ICP, Parameters]:
    """
    Creates an ICP pipeline from a configuration (see ICP.from_yaml()). Returns a tuple (ICP, Parameters), where Parameters is the same object as ICP.parameters.
    """
def layer_from_numpy(data: typing.Any) -> mrpt.maps.CGenericPointsMap:
    """
    Creates an mrpt.maps.CGenericPointsMap from a dict {field_name: 1-D array} or from an array of shape (N,3) or (N,2). 'x' and 'y' are mandatory; 'z' defaults to zeros. Coordinates are stored as float32; other fields keep their dtype: float32, float64, uint8, uint16 or uint32.
    """
def layer_points_view(layer: mrpt.maps.CMetricMap) -> mrpt.maps.CPointsMap:
    """
    Returns the layer itself if it is an mrpt.maps.CPointsMap, or a copy of the point cloud it provides otherwise (e.g. voxel maps).
    """
def layer_to_numpy(layer: mrpt.maps.CMetricMap) -> dict:
    """
    Returns a copy of all the per-point fields of a point cloud layer, as a dict {field_name: 1-D NumPy array}. Keys are 'x', 'y', 'z' (float32), then the float32, float64, uint16, uint8 and uint32 fields. Layers that are not point clouds, but provide one (e.g. voxel maps), are converted through that point cloud view.
    """
def layer_visualization(layer: mrpt.maps.CMetricMap, params: RenderParamsPointLayer | None = None) -> mrpt.viz.CSetOfObjects:
    """
    Returns an mrpt.viz.CSetOfObjects with a 3D view of a single layer, with optional RenderParamsPointLayer.
    """
def load_plugins(modules: str = 'libmola_metric_maps.so') -> None:
    """
    Loads one or more (comma separated) shared libraries with additional MRPT classes, e.g. map layer types or ICP pipeline modules. Default: libmola_metric_maps.so, needed to load maps built with MOLA.
    """
def load_xyz_file(path: typing.Any) -> mrpt.maps.CSimplePointsMap:
    """
    Loads a point cloud from a text file with one 'x y z' point per row, optionally gzip-compressed (.gz). Returns an mrpt.maps.CSimplePointsMap.
    """
def load_yaml(config: typing.Any, variables: typing.Any = None) -> typing.Any:
    """
    Loads a YAML configuration (file path, dict or mrpt.containers.YAML node) into Python dicts and lists, expanding `${VAR}` and `${VAR|default}` tokens from `variables` (dict) and the environment, as all mp2p_icp configuration loaders do.
    """
def save_las(layer: mrpt.maps.CMetricMap, path: typing.Any, fields: typing.Any = None, system_id: str = 'mp2p_icp', generating_software: str = 'MOLA mp2p_icp', transform: typing.Any = None, geodetic: bool = False, georeferencing: Georeferencing | None = None) -> None:
    """
    Saves a point cloud layer as LAS 1.4 (point format 8, plus extra dimensions for non-standard fields). With `geodetic=True`, points are written as WGS-84 longitude/latitude/height using `georeferencing`.
    """
def save_ply(layer: mrpt.maps.CMetricMap, path: typing.Any, binary: bool = True, fields: typing.Any = None, ignore_missing_fields: bool = False, transform: typing.Any = None) -> None:
    """
    Saves a point cloud layer as PLY, with all its fields (or those in `fields`, in that order). Colors are written as red, green, blue. `transform` optionally transforms the points (e.g. georeferencing.T_enu_to_map.mean, to export in the ENU frame).
    """
def save_txt(layer: mrpt.maps.CMetricMap, path: typing.Any, header: bool = True, fields: typing.Any = None, transform: typing.Any = None) -> None:
    """
    Saves a point cloud layer as a space-separated text file, one point per row, with an optional header line with the field names.
    """
def simplemap_to_metricmap(simplemap: typing.Any, pipeline: typing.Any = None, *, variables: typing.Any = None, custom_variables: collections.abc.Mapping[str, typing.SupportsFloat | typing.SupportsIndex] = {}, verbosity: str = 'INFO', show_progress_bar: bool = False, start_index: typing.SupportsInt | typing.SupportsIndex | None = None, end_index: typing.SupportsInt | typing.SupportsIndex | None = None, decimate_every_nth_frame: typing.SupportsInt | typing.SupportsIndex | None = None, decimate_maximum_frame_count: typing.SupportsInt | typing.SupportsIndex | None = None, throw_on_missing_external_files: bool = True, georeferencing: Georeferencing | None = None, lazy_load_base_dir: typing.Any = None) -> MetricMap:
    """
    Builds a MetricMap from a simplemap (keyframes with poses and raw observations), as the `sm2mm` tool does. `simplemap` is an mrpt.maps.CSimpleMap or a file path; `pipeline` is an sm2mm YAML file path, dict or YAML node, with `generators`, `filters` and `final_filters` (default: one generator, no filters). `variables` expands `${VAR}` tokens in the pipeline; `custom_variables` defines extra numeric variables for parameter formulas. `lazy_load_base_dir` is the directory of externally stored observations (default: `<simplemap name>_Images`, if it exists).
    """
PT_LAYER_PLANE_CENTROIDS: str = 'plane_centroids'
PT_LAYER_RAW: str = 'raw'
__version__: str = '3.1.0'
