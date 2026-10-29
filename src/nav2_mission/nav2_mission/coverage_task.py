"""Frozen coverage requests and the unified handler's existing bridge adapter."""
import copy
import math

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
    polygons = copy.deepcopy(request.get('polygons'))
    if not isinstance(polygons, list) or len(polygons) != 1:
        raise ValueError('rectangle coverage requires one outer polygon and no holes')
    points = polygons[0]
    if not isinstance(points, list):
        raise ValueError('rectangle points must be a list')
    if len(points) == 5 and points[0] == points[-1]:
        points.pop()
    if len(points) != 4:
        raise ValueError('rectangle requires four corners')
    for point in points:
        if (not isinstance(point, (list, tuple)) or len(point) != 2 or
                any(not finite(v) for v in point)):
            raise ValueError('rectangle coordinates must be finite x/y pairs')
    edges = [(points[(i+1) % 4][0]-p[0], points[(i+1) % 4][1]-p[1])
             for i, p in enumerate(points)]
    lengths = [math.hypot(*e) for e in edges]
    if not all(finite(value) for edge in edges for value in edge) or not all(
            finite(length) for length in lengths):
        raise ValueError('rectangle dimensions must be finite')
    if min(lengths) <= 0:
        raise ValueError('rectangle must have positive dimensions')
    for i, (x, y) in enumerate(edges):
        nx, ny = edges[(i+1) % 4]
        ox, oy = edges[(i+2) % 4]
        if (abs((x/lengths[i])*(nx/lengths[(i+1) % 4]) +
                (y/lengths[i])*(ny/lengths[(i+1) % 4])) > 1e-8 or
                math.hypot(x+ox, y+oy) > 1e-8*max(lengths)):
            raise ValueError('coverage polygon must be a simple rectangle')
    return {'frame_id': frame, 'polygons': [points + [copy.deepcopy(points[0])]],
            'planner': planner}


class CoverageTask:
    def __init__(self, bridge):
        self.bridge = bridge

    def prepare(self, request):
        return prepare_coverage(request)

    def begin(self, task_id, config, now):
        return self.bridge.start(task_id, config)
