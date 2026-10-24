"""Metric mono cylinder fitting from a learned image mask and native pixel edges.

The known 24 mm diameter supplies scale. A support plane is used only before
pickup. When both ends are clipped, axial translation is unobservable; the
caller must supply its previously observed, kinematically propagated prior.
No simulator state, segmentation labels or depth is accessed here.
"""
from dataclasses import dataclass
import cv2
import numpy as np
from scipy.optimize import least_squares

from .geometry import calibration, project


@dataclass
class Edges:
    left: np.ndarray
    right: np.ndarray
    center: np.ndarray
    axis: np.ndarray
    normal: np.ndarray
    minimum: float
    maximum: float
    full_length: bool
    cap_samples: list
    rms_px: float
    sample_count: int


def sample(image, points):
    points = np.asarray(points, np.float32)
    return cv2.remap(image, points[:, 0:1], points[:, 1:2], cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_REPLICATE).ravel()


def gradient_edge(image, origin, direction, expected, sign, radius=12., outer=0):
    coordinates = np.arange(expected-radius, expected+radius+.01, .25)
    points = origin + coordinates[:, None]*direction
    height, width = image.shape
    # The edge itself can be fully visible even when its search window crosses
    # the image border. Clip the search, never synthesize out-of-frame pixels.
    inside = ((points[:,0] >= 2) & (points[:,0] <= width-3)
              & (points[:,1] >= 2) & (points[:,1] <= height-3))
    coordinates, points = coordinates[inside], points[inside]
    if len(coordinates) < 5:
        return None
    values = sample(image, points)
    raw_gradient=np.gradient(values,.25)
    gradient = np.abs(raw_gradient) if sign==0 else raw_gradient*sign
    # Penalize distant texture gradients while retaining subpixel edge peaks.
    score = gradient if outer else gradient / (1 + ((coordinates-expected)/radius)**2)
    if outer:
        # Rib texture can be stronger than the silhouette. End caps use the
        # outermost resolved transition, not the strongest internal rib.
        positive=np.flatnonzero(gradient>=max(.012,.1*float(gradient.max())))
        groups=np.split(positive,np.flatnonzero(np.diff(positive)>1)+1)
        groups=[group for group in groups if len(group)>=3]
        if not groups:return None
        group=groups[0] if outer<0 else groups[-1]
        i=int(group[np.argmax(gradient[group])])
    else:i = int(score.argmax())
    if gradient[i] < .012 or i == 0 or i == len(score)-1:
        return None
    denominator = score[i-1]-2*score[i]+score[i+1]
    correction = .5*(score[i-1]-score[i+1])/denominator if abs(denominator) > 1e-8 else 0.
    return float(coordinates[i] + np.clip(correction, -.5, .5)*.25), float(gradient[i])


def fit_edges(image, probability):
    if image.dtype != np.uint8 or image.ndim != 2 or probability.shape != image.shape:
        raise ValueError('Mono8 pixels and same-size detector probabilities required')
    mask = (probability > .5).astype(np.uint8)
    count, components, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        raise ValueError('No detected steel component')
    largest = 1 + stats[1:, cv2.CC_STAT_AREA].argmax()
    if stats[largest, cv2.CC_STAT_AREA] < 100:
        raise ValueError('Steel component too small')
    ys, xs = np.where(components == largest)
    points = np.column_stack((xs, ys)).astype(float)
    center = points.mean(0)
    _, vectors = np.linalg.eigh(np.cov(points.T))
    u = vectors[:, -1]
    if u[1] < 0 or (abs(u[1]) < .1 and u[0] < 0):
        u = -u
    n = np.array([-u[1], u[0]])
    # Include disconnected visible pieces along the same detected rod.
    accepted = np.flatnonzero(stats[:,cv2.CC_STAT_AREA] >= max(80, .03*stats[largest,cv2.CC_STAT_AREA]))
    accepted = accepted[accepted != 0]
    all_y, all_x = np.where(np.isin(components,accepted))
    all_points = np.column_stack((all_x, all_y)).astype(float)
    transverse = (all_points-center) @ n
    width = max(np.ptp((points-center) @ n), 8.)
    points = all_points[np.abs(transverse) < width]
    t = (points-center) @ u; v = (points-center) @ n
    minimum, maximum = np.percentile(t, [.05, 99.95])
    if maximum-minimum < 70:
        raise ValueError('Insufficient visible steel length')
    lo, hi = np.percentile(v, [.5, 99.5])
    blurred = cv2.GaussianBlur(image.astype(np.float32)/255., (0, 0), .7)
    pairs = []
    for value in np.linspace(minimum+8, maximum-8, 100):
        origin = center+value*u
        coordinates = np.arange(lo-3, hi+3, .5)
        ps = sample(probability.astype(np.float32), origin+coordinates[:, None]*n)
        hits = np.flatnonzero(ps > .5)
        if len(hits) < 6:
            continue
        left, right = coordinates[hits[0]], coordinates[hits[-1]]
        if right-left < 4:
            continue
        search = min(16., max(6., .45*(right-left)))
        le = gradient_edge(blurred, origin, n, left, -1, search)
        re = gradient_edge(blurred, origin, n, right, 1, search)
        if le and re and re[0]-le[0] >= 4:
            pairs.append((value, le[0], re[0], min(le[1], re[1])))
    pairs = np.asarray(pairs)
    if len(pairs) < 12:
        raise ValueError('Insufficient bilateral image edges')
    def line(values):
        # A mask may bleed into the background at one end of a cropped rod.
        # Least-squares initialization gives those high-leverage samples enough
        # influence to retain a tilted false boundary. A repeated-median slope
        # seed resists a minority of corrupt edge samples at either image end.
        t = pairs[:, 0]
        separation = t[:, None]-t[None, :]
        slopes = np.divide(values[:, None]-values[None, :], separation,
                           out=np.full(separation.shape, np.nan), where=separation != 0)
        slope = np.median(np.nanmedian(slopes, axis=1))
        intercept = np.median(values-slope*t)
        residual = values-(slope*t+intercept)
        keep = np.abs(residual) <= max(.65, 2.5*np.median(np.abs(residual)))
        if keep.sum() < max(12, .5*len(pairs)):
            raise ValueError('Steel boundary lacks majority image support')
        for _ in range(4):
            a, b = np.polyfit(pairs[keep, 0], values[keep], 1, w=np.sqrt(pairs[keep, 3]))
            residual = values - (a*pairs[:, 0]+b)
            threshold = max(.65, 2.5*np.median(np.abs(residual[keep])))
            keep = np.abs(residual) <= threshold
            if keep.sum() < 12:
                raise ValueError('Steel boundary fit is inconsistent')
        normal = n-a*u
        return np.r_[normal, -normal @ center-b], float(np.sqrt(np.mean(residual[keep]**2))), (a, b)
    left, lrms, left_ab = line(pairs[:, 1])
    right, rrms, right_ab = line(pairs[:, 2])
    height, width_px = image.shape
    bounds = np.percentile(points,[.5,99.5],axis=0)
    full = (bounds[0,0] > 4 and bounds[1,0] < width_px-5
            and bounds[0,1] > 4 and bounds[1,1] < height-5)
    caps = []
    if full:
        for side, value, sign in ((0, minimum, -1), (1, maximum, 1)):
            lower = left_ab[0]*value+left_ab[1]
            upper = right_ab[0]*value+right_ab[1]
            for fraction in (-.3, 0., .3):
                across = (lower+upper)/2 + fraction*(upper-lower)
                edge = gradient_edge(blurred, center+across*n, u, value, sign, 40.,-1 if side==0 else 1)
                if edge:
                    caps.append((side, across, edge[0]))
    return Edges(left, right, center, u, n, minimum, maximum, full, caps,
                 max(lrms, rrms), len(pairs))


def plane_normals(edges, K):
    a, b = K.T @ edges.left, K.T @ edges.right
    a /= np.linalg.norm(a); b /= np.linalg.norm(b)
    if np.dot(a, b) < 0:
        b = -b
    if np.linalg.norm(np.cross(a, b)) < .004:
        raise ValueError('Cylinder width too small to resolve metric scale')
    return a, b


def cylinder_hull(center, axis, K, T, radius=.012, length=.6):
    axis = np.asarray(axis)/np.linalg.norm(axis)
    seed = np.array([0., 0., 1.]) if abs(axis[2]) < .9 else np.array([1., 0., 0.])
    a = np.cross(axis, seed); a /= np.linalg.norm(a)
    b = np.cross(axis, a)
    angles = np.linspace(0, 2*np.pi, 48, endpoint=False)
    ring = radius*(np.cos(angles)[:, None]*a+np.sin(angles)[:, None]*b)
    vertices = np.vstack((center-.5*length*axis+ring, center+.5*length*axis+ring))
    pixels, depth = project(vertices, K, T)
    if np.any(depth < .02):
        raise ValueError('Cylinder is behind or too near the camera')
    return cv2.convexHull(pixels.astype(np.float32)).reshape(-1, 2).astype(float)


def contour_intersections(hull, edges, across):
    xy = hull-edges.center
    t, v = xy @ edges.axis, xy @ edges.normal
    nt, nv = np.roll(t, -1), np.roll(v, -1)
    hit = (np.minimum(v, nv) <= across) & (np.maximum(v, nv) >= across) & (np.abs(nv-v) > 1e-8)
    intersections = t[hit]+(across-v[hit])*(nt[hit]-t[hit])/(nv[hit]-v[hit])
    if len(intersections) < 2:
        nearest = np.argsort(np.abs(v-across))[:2]
        return min(t[nearest])-10, max(t[nearest])+10
    return float(intersections.min()), float(intersections.max())


def resting_pose(image, probability, K, T_frame_optical, *, plane_height=.522, radius=.012,
                 prior_center=None,prior_axis=None):
    K, T = calibration(K, T_frame_optical)
    edges = fit_edges(image, probability)
    tracked=prior_center is not None
    if tracked:
        # Previous *visual* support measurements predict where each cap should
        # appear after the calibrated wrist moves. Both caps are measured anew;
        # the predictor never substitutes for missing image evidence. Tray and
        # machine backgrounds can invert cap contrast, so either polarity is
        # valid inside this bounded eight-pixel search, unlike a global rib scan.
        if prior_axis is None:raise ValueError('Visual cap predictor requires its observed axis')
        hull=cylinder_hull(prior_center,prior_axis,K,T,radius)
        height,width=image.shape
        edges.full_length=bool((hull.min(0)>4).all() and (hull.max(0)<[width-5,height-5]).all())
        edges.cap_samples=[]
        blurred=cv2.GaussianBlur(image.astype(np.float32)/255.,(0,0),.7)
        extents=(hull-edges.center)@edges.axis
        for side,value in enumerate((extents.min(),extents.max())):
            origin=edges.center+value*edges.axis
            limits=[-(line[:2]@origin+line[2])/(line[:2]@edges.normal) for line in (edges.left,edges.right)]
            for fraction in (-.3,0.,.3):
                across=np.mean(limits)+fraction*(limits[1]-limits[0])
                expected=contour_intersections(hull,edges,across)[side]
                edge=gradient_edge(blurred,edges.center+across*edges.normal,edges.axis,expected,0,8.)
                if edge:edges.cap_samples.append((side,across,edge[0]))
    if not edges.full_length or len(edges.cap_samples) < 4:
        raise ValueError(f'Both steel ends must be observed before pickup (full={edges.full_length}, caps={len(edges.cap_samples)})')
    if not all(sum(cap[0]==side for cap in edges.cap_samples)>=2 for side in (0,1)):
        raise ValueError('Both cap transitions require independent image evidence')
    # Initializer only: invert both image ends against the known support plane.
    ends = edges.center + np.array([edges.minimum, edges.maximum])[:, None]*edges.axis
    rays = np.linalg.solve(K, np.column_stack((ends, np.ones(2))).T).T @ T[:3,:3].T
    distances = (plane_height-T[2,3])/rays[:,2]
    if np.any(distances <= 0):
        raise ValueError('Support plane behind the camera')
    endpoints = T[:3,3]+rays*distances[:,None]
    initial = endpoints.mean(0)
    delta = endpoints[1]-endpoints[0]
    yaw = np.arctan2(delta[1], delta[0])
    if tracked:
        initial=np.asarray(prior_center);yaw=np.arctan2(prior_axis[1],prior_axis[0])
    normal1, normal2 = plane_normals(edges, K)
    f = (K[0,0]+K[1,1])/2
    def residual(parameters):
        center = np.r_[parameters[:2], plane_height]
        axis = np.array([np.cos(parameters[2]), np.sin(parameters[2]), 0.])
        copt = (center-T[:3,3]) @ T[:3,:3]
        aopt = axis @ T[:3,:3]
        scale = f/max(copt[2], .05)
        result = [(normal1 @ copt-radius)*scale, (normal2 @ copt+radius)*scale,
                  normal1 @ aopt*.3*scale, normal2 @ aopt*.3*scale]
        hull = cylinder_hull(center, axis, K, T, radius)
        for side, across, target in edges.cap_samples:
            result.append(contour_intersections(hull, edges, across)[side]-target)
        return result
    x0 = np.r_[initial[:2], yaw]
    def jacobian(parameters):
        # diff_step in SciPy is relative to each coordinate. Near y=0 that
        # becomes smaller than the float32 image-hull precision and can make
        # a fully visible tray bar appear to have an inconsistent length.
        increments=np.array([2e-5,2e-5,2e-4])
        columns=[]
        for i,increment in enumerate(increments):
            offset=np.zeros(3);offset[i]=increment
            columns.append((np.asarray(residual(parameters+offset))-np.asarray(residual(parameters-offset)))/(2*increment))
        return np.column_stack(columns)
    result = least_squares(residual, x0, bounds=(x0-np.array([.10,.10,.2]), x0+np.array([.10,.10,.2])),
                           loss='soft_l1', f_scale=1., jac=jacobian, max_nfev=40)
    rms = float(np.sqrt(np.mean(np.square(residual(result.x)))))
    if rms > 2.5:
        raise ValueError(f'Cylinder silhouette residual too large: {rms:.2f}px')
    center = np.r_[result.x[:2], plane_height]
    axis = np.array([np.cos(result.x[2]), np.sin(result.x[2]), 0.])
    if tracked and np.linalg.norm(center-prior_center)>.010:
        raise ValueError('Full-cap image update exceeds bounded supported motion')
    return center, axis, dict(source='mono_known_plane_cylinder', residual_px=rms,
                              visible_edges=edges.sample_count, full_length=True,
                              cap_search='bounded_visual_prediction_both_polarities' if tracked else 'full_initial_survey',
                              axial_position_measured=True)


def tracked_pose(image, probability, K, T_frame_optical, prior_center, prior_axis, *, radius=.012):
    K, T = calibration(K, T_frame_optical)
    edges = fit_edges(image, probability)
    n1, n2 = plane_normals(edges, K)
    axis = np.cross(n1, n2); axis /= np.linalg.norm(axis)
    world_axis = T[:3,:3] @ axis
    if world_axis @ prior_axis < 0:
        axis = -axis; world_axis = -world_axis
    if world_axis @ prior_axis < np.cos(.15):
        raise ValueError('Observed steel axis disagrees with bounded motion prior')
    prior_optical = (np.asarray(prior_center)-T[:3,3]) @ T[:3,:3]
    center_optical = np.linalg.solve(np.stack((n1, n2, axis)),
                                    np.array([radius, -radius, axis @ prior_optical]))
    center = T[:3,:3] @ center_optical + T[:3,3]
    if center_optical[2] < .03 or np.linalg.norm(center-np.asarray(prior_center)) > .06:
        raise ValueError('Visual position update exceeds bounded motion prior')
    return center, world_axis, dict(source='mono_cylinder_edges_with_axial_prior',
        residual_px=edges.rms_px, visible_edges=edges.sample_count,
        full_length=edges.full_length, axial_position_measured=False)


def constrained_pose(image, probability, K, T_frame_optical, prior_center, prior_axis,
                     *, on_support_plane=False, plane_height=.522, radius=.012):
    """Image-corrected support/rigid-grasp prior with explicit monocular limits.

    The side-plane bisector provides precise lateral alignment. Radius-based
    monocular depth is weak at this scale, so it is fused conservatively with
    the independently propagated robot/grasp prior, not recursively integrated.
    Axial slip with both ends occluded remains unobservable and is reported.
    """
    K, T = calibration(K, T_frame_optical)
    edges = fit_edges(image, probability)
    n1, n2 = plane_normals(edges, K)
    h = T[:3,:3] @ (n1+n2)
    h /= np.linalg.norm(h)
    center = np.asarray(prior_center, float).copy()
    axis = np.asarray(prior_axis, float).copy()
    if on_support_plane:
        axis = np.cross(h, [0., 0., 1.]); axis /= np.linalg.norm(axis)
        if axis @ prior_axis < 0:
            axis = -axis
        direction = np.array([-axis[1], axis[0], 0.])
        denominator = h @ direction
        if abs(denominator) < .05:
            raise ValueError('Support-plane view is degenerate')
        center[2] = plane_height
        center -= direction * (h @ (center-T[:3,3])) / denominator
    else:
        axis -= h*(h @ axis)
        axis /= np.linalg.norm(axis)
        center -= h*(h @ (center-T[:3,3]))
        optical_axis = axis @ T[:3,:3]
        prior_optical = (np.asarray(prior_center)-T[:3,3]) @ T[:3,:3]
        raw = np.linalg.solve(np.stack((n1, n2, optical_axis)),
                              [radius, -radius, optical_axis @ prior_optical])
        raw = T[:3,:3] @ raw+T[:3,3]
        radial = raw-center
        radial -= axis*(axis @ radial)
        if np.linalg.norm(radial) > .04:
            raise ValueError('Apparent diameter inconsistent with grasp geometry')
        center += .12*radial
    if np.linalg.norm(center-prior_center) > .05 or axis @ prior_axis < np.cos(.15):
        raise ValueError('Visual update exceeds the bounded pose prior')
    return center, axis, dict(source='mono_edges_support_or_grasp_prior',
        residual_px=edges.rms_px, visible_edges=edges.sample_count,
        full_length=bool(edges.full_length), axial_position_measured=False,
        prior='surveyed_support_plane' if on_support_plane else 'vision_initialized_robot_grasp_kinematics')
