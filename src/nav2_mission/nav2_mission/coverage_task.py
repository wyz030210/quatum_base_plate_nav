"""Frozen coverage requests and the unified handler's existing bridge adapter."""
import copy
import math

from .coverage.models import Point2D, RegionSnapshot
from .coverage.region_editor import validate_region

# Catalog of the local planner parameters; values describe the reference vehicle profile.
# Execution reads the authoritative values under a lease, never writes fixed settings.
PLANNER_DEFAULTS = {
    'coordinates_in_cartesian_frame': True, 'robot_width': 0.68, 'robot_length': 0.88,
    'operation_width': 0.60, 'swath_spacing': 1.50, 'min_turning_radius': 0.95,
    'linear_curv_change': 200.0, 'validate_path_bounds': True, 'path_boundary_margin': 0.03,
    'default_allow_overlap': False, 'default_headland_type': 'CONSTANT',
    'default_headland_width': 1.50, 'default_path_continuity_type': 'CONTINUOUS',
    'default_path_type': 'DUBIN', 'default_route_type': 'BOUSTROPHEDON',
    'default_swath_angle_type': 'SET_ANGLE', 'default_swath_angle': 1.57,
    'default_swath_type': 'NUMBER', 'default_turn_point_distance': 0.05,
    'default_generate_decomp': False, 'default_decomp_type': 'NONE',
    'default_decomp_split_angle': 0.0, 'default_step_angle': 0.017453,
    'default_max_swaths_for_global_route': 300, 'default_spiral_n': 4,
    'default_custom_order': [], 'default_tsp_redirect_swaths': True,
    'default_tsp_time_limit': 1, 'default_tsp_search_for_optimum': False,
    'default_tsp_d_tol': 0.0001, 'default_reduce_path': False, 'default_reduce_min_dist': 0.1,
}


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def prepare_coverage(request):
    if not isinstance(request, dict) or set(request) - {'frame_id', 'polygons', 'planner'}:
        raise ValueError('unsupported coverage request fields')
    frame = request.get('frame_id', 'map')
    if not isinstance(frame, str) or not frame.strip():
        raise ValueError('coverage frame_id must be nonempty')
    planner = {}
    changes = request.get('planner', {})
    if not isinstance(changes, dict) or set(changes) - {'operation_width', 'default_swath_angle'}:
        raise ValueError('unsupported coverage planner parameters')
    for key, value in changes.items():
        if not finite(value):
            raise ValueError(f'{key} must be finite')
        planner[key] = float(value)
    if planner.get('operation_width', 1.0) <= 0:
        raise ValueError('operation_width must be positive')
    if not -math.pi <= planner.get('default_swath_angle', 0.0) <= math.pi:
        raise ValueError('default_swath_angle must be radians within [-pi, pi]')
    polygons = copy.deepcopy(request.get('polygons'))
    region = coverage_region(polygons)
    polygon = validate_region(region, planner.get('operation_width'))
    rings = [polygon.exterior, *polygon.interiors]
    return {'frame_id': frame, 'polygons': [list(map(list, ring.coords)) for ring in rings],
            'planner': planner}


def coverage_region(polygons):
    """Decode the shared outer-first rings without changing the selected region."""
    if not isinstance(polygons, list) or not polygons:
        raise ValueError('coverage requires an outer polygon followed by optional holes')
    rings = []
    for ring in polygons:
        if not isinstance(ring, list):
            raise ValueError('polygon points must be a list')
        if any(not isinstance(point, (list, tuple)) or len(point) != 2 for point in ring):
            raise ValueError('polygon coordinates must be finite x/y pairs')
        rings.append(tuple(Point2D(*point) for point in ring))
    return RegionSnapshot(rings[0], tuple(rings[1:]))


class CoverageTask:
    def __init__(self, bridge):
        self.bridge = bridge

    def prepare(self, request):
        return prepare_coverage(request)

    def begin(self, task_id, config, now):
        return self.bridge.start(task_id, config)
