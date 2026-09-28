"""Local URDF kinematics and conservative primitive collision screening.

No ROS/Isaac dependency. Cylinders use covering spheres (capsule envelopes);
boxes use oriented-box SAT. This is sampled screening, not MoveIt validation.
"""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import xacro

SCRIPTS = Path(__file__).resolve().parents[1]
ROOT = SCRIPTS.parents[1]
sys.path.insert(0, str(SCRIPTS))
from rebar_experiment_geometry import rack_boxes
from rebar_tester_geometry import machine_boxes

ARM = ("shoulder_joint", "upperArm_joint", "foreArm_joint",
       "wrist1_joint", "wrist2_joint", "wrist3_joint")


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def origin(element):
    if element is None:
        return np.eye(4)
    return transform([float(v) for v in element.get("xyz", "0 0 0").split()],
                     [float(v) for v in element.get("rpy", "0 0 0").split()])


def expanded_robot():
    """Expand the actual xacro without requiring a ROS package index on macOS."""
    with tempfile.TemporaryDirectory(prefix="rebar_xacro_") as temp:
        directory = Path(temp) / "urdf"
        directory.mkdir()
        for source in (ROOT / "seer_description/urdf").iterdir():
            if source.is_file() and source.suffix in (".urdf", ".xacro"):
                (directory / source.name).write_text(source.read_text().replace(
                    "$(find seer_description)", temp))
        doc = xacro.process_file(str(directory / "composite_robot_finger_mono.urdf.xacro"))
        return ET.fromstring(doc.toxml())


def box(center, size, rotation=None):
    return ("box", np.asarray(center), np.asarray(size) / 2,
            np.eye(3) if rotation is None else rotation)


def spheres(center, radius):
    return ("spheres", np.atleast_2d(center), float(radius))


def cylinder_envelope(matrix, length, radius, step=0.025):
    count = max(2, int(np.ceil(length / step)) + 1)
    points = np.zeros((count, 3))
    points[:, 2] = np.linspace(-length / 2, length / 2, count)
    points = points @ matrix[:3, :3].T + matrix[:3, 3]
    # Cover every cross-section between sampled centres, including end caps.
    covering_radius = np.hypot(radius, length / (count - 1) / 2)
    return spheres(points, covering_radius)


def separation(a, b):
    """Positive means separated; box SAT gives a lower-bound clearance."""
    if a[0] == "spheres" and b[0] == "spheres":
        return float(np.linalg.norm(a[1][:, None, :] - b[1][None, :, :], axis=2).min() - a[2] - b[2])
    if a[0] == "box" and b[0] == "spheres":
        a, b = b, a
    if a[0] == "spheres":
        local = (a[1] - b[1]) @ b[3]
        excess = np.abs(local) - b[2]
        distances = np.linalg.norm(np.maximum(excess, 0), axis=1) + np.minimum(excess.max(axis=1), 0)
        return float(distances.min() - a[2])
    axes = [*a[3].T, *b[3].T]
    axes.extend(np.cross(x, y) for x in a[3].T for y in b[3].T)
    axes = np.asarray([axis / np.linalg.norm(axis) for axis in axes if np.linalg.norm(axis) > 1e-8])
    centre_projection = np.abs(axes @ (a[1] - b[1]))
    radii = np.abs(axes @ a[3]) @ a[2] + np.abs(axes @ b[3]) @ b[2]
    return float((centre_projection - radii).max())


def bounding_sphere(shape):
    if shape[0] == "box":
        return shape[1], np.linalg.norm(shape[2])
    centre = shape[1].mean(axis=0)
    return centre, np.linalg.norm(shape[1] - centre, axis=1).max() + shape[2]


class RobotModel:
    def __init__(self):
        robot = expanded_robot()
        self.joints = []
        self.geometry = {}
        for joint in robot.findall("joint"):
            axis_node = joint.find("axis")
            axis = np.array([float(v) for v in axis_node.get("xyz", "0 0 1").split()]) if axis_node is not None else np.array([0., 0., 1.])
            self.joints.append((joint.get("name"), joint.get("type"),
                                joint.find("parent").get("link"), joint.find("child").get("link"),
                                origin(joint.find("origin")), axis))
        self.lower, self.upper = [], []
        for name in ARM:
            limit = robot.find(f"joint[@name='{name}']/limit")
            self.lower.append(float(limit.get("lower")))
            self.upper.append(float(limit.get("upper")))
        self.lower, self.upper = np.array(self.lower), np.array(self.upper)
        for link in robot.findall("link"):
            shapes = []
            for collision in link.findall("collision"):
                geometry = collision.find("geometry")
                shape = list(geometry)[0]
                if shape.tag == "mesh":
                    raise ValueError(f"Unscreened mesh collision on {link.get('name')}")
                shapes.append((origin(collision.find("origin")), shape.tag, shape.attrib))
            self.geometry[link.get("name")] = shapes
        srdf = ET.parse(ROOT / "seer_aubo_finger_mono_moveit_config/config/seer_aubo_composite.srdf").getroot()
        self.disabled = {frozenset((p.get("link1"), p.get("link2")))
                         for p in srdf.findall("disable_collisions")}
        self.touch = {"gripper_motor_link", "gripper1_link", "gripper2_link"}
        self.moving = {"shoulder_Link", "upperArm_Link", "foreArm_Link", "wrist1_Link",
                       "wrist2_Link", "wrist3_Link", "gripper_motor_link", "gripper1_link",
                       "gripper2_link", "camera_link", "gripper_adapter_link"}
        self.environment = []
        for name, centre, size, rpy in rack_boxes():
            self.environment.append(("rack/" + name, box(centre, size, Rotation.from_euler("xyz", rpy).as_matrix())))
        base_world = transform((6.08, 2.6, 0), (0, 0, -np.pi / 2))
        inverse = np.linalg.inv(base_world)
        for name, centre, size in machine_boxes():
            m = inverse @ transform(centre)
            self.environment.append(("tester/" + name, box(m[:3, 3], size, m[:3, :3])))
        self.environment_bounds = [bounding_sphere(shape) for _, shape in self.environment]

    def frames(self, q):
        values = dict(zip(ARM, q))
        values.update(gripper1_joint=.017, gripper2_joint=.017)
        frames = {"base_footprint": np.eye(4)}
        pending = list(self.joints)
        while pending:
            rest = []
            for name, kind, parent, child, initial, axis in pending:
                if parent not in frames:
                    rest.append((name, kind, parent, child, initial, axis))
                    continue
                motion = np.eye(4)
                value = values.get(name, 0.)
                if kind in ("revolute", "continuous"):
                    motion[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
                elif kind == "prismatic":
                    motion[:3, 3] = axis * value
                frames[child] = frames[parent] @ initial @ motion
            if len(rest) == len(pending):
                raise ValueError("Disconnected URDF")
            pending = rest
        return frames

    def tcp(self, q):
        wrist = self.frames(q)["wrist3_Link"].copy()
        wrist[:3, 3] += wrist[:3, :3] @ np.array([0, 0, .16])
        return wrist

    def ik(self, tcp_xyz, rotation, seed):
        desired_xyz = np.asarray(tcp_xyz) - rotation @ np.array([0, 0, .16])
        def residual(q):
            wrist = self.frames(q)["wrist3_Link"]
            return np.r_[wrist[:3, 3] - desired_xyz,
                         Rotation.from_matrix(rotation @ wrist[:3, :3].T).as_rotvec() * .35]
        result = least_squares(residual, np.clip(seed, self.lower + 1e-8, self.upper - 1e-8),
                               bounds=(self.lower, self.upper), ftol=1e-9, xtol=1e-9,
                               gtol=1e-9, max_nfev=90)
        if np.linalg.norm(residual(result.x)) > 1e-5:
            return None
        # Revolute equivalences must stay near the previous state, within URDF limits.
        q = result.x.copy()
        for i in range(6):
            variants = [q[i] + n * 2 * np.pi for n in range(-2, 3)
                        if self.lower[i] <= q[i] + n * 2 * np.pi <= self.upper[i]]
            q[i] = min(variants, key=lambda x: abs(x - seed[i]))
        return q

    def collision_geometry(self, q):
        frames = self.frames(q)
        bodies = []
        for link, shapes in self.geometry.items():
            for index, (local, kind, attributes) in enumerate(shapes):
                m = frames[link] @ local
                if kind == "box":
                    shape = box(m[:3, 3], [float(v) for v in attributes["size"].split()], m[:3, :3])
                elif kind == "sphere":
                    shape = spheres(m[:3, 3], float(attributes["radius"]))
                else:
                    shape = cylinder_envelope(m, float(attributes["length"]), float(attributes["radius"]))
                bodies.append((link, shape))
        wrist = frames["gripper_motor_link"]
        bar = wrist @ transform((0, 0, .16), (np.pi / 2, 0, 0))
        bodies.append(("payload", cylinder_envelope(bar, .62, .020)))
        return bodies

    def clearance(self, q):
        bodies = self.collision_geometry(q)
        bounds = [bounding_sphere(shape) for _, shape in bodies]
        best = (float("inf"), "")
        for i, (link, a) in enumerate(bodies):
            if link not in self.moving and link != "payload":
                continue
            centre, radius = bounds[i]
            for j, (other, b) in enumerate(bodies[:i]):
                if link == other or frozenset((link, other)) in self.disabled:
                    continue
                if link == "payload" and other in self.touch:
                    continue
                other_centre, other_radius = bounds[j]
                if np.linalg.norm(centre - other_centre) - radius - other_radius > best[0]:
                    continue
                d = separation(a, b)
                if d < best[0]:
                    best = (d, f"{link} / {other}")
            for (other, b), (other_centre, other_radius) in zip(self.environment, self.environment_bounds):
                if np.linalg.norm(centre - other_centre) - radius - other_radius > best[0]:
                    continue
                d = separation(a, b)
                if d < best[0]:
                    best = (d, f"{link} / {other}")
        return best
