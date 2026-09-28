#!/usr/bin/env python3
"""Plan a compact local candidate from the lifted rack bar to front staging.

Run with uv; this script has no network or robot command interface. It exports
sampled joints, TCP poses, clearances and comparison figures for review.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from offline_rebar_model import ARM, ROOT, RobotModel

START_ROTATION = np.diag([1., -1., -1.])
FRONT_ROTATION = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
FRONT_TCP = np.array([-.80, -.08, 1.50])
OLD_STAGING = np.array([-.35, .55, 1.20])


def joint_curve(start, end, schedules, count):
    t = np.linspace(0, 1, count)
    u = 10 * t**3 - 15 * t**4 + 6 * t**5
    # Smooth monotonic progress per joint; all endpoint derivatives are zero.
    progress = u[:, None] / (schedules + (1 - schedules) * u[:, None])
    return start + (end - start) * progress


def metrics(model, joints):
    poses = np.array([model.tcp(q) for q in joints])
    length = float(np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1).sum())
    angles = Rotation.from_matrix(poses[1:, :3, :3] @ poses[:-1, :3, :3].transpose(0, 2, 1)).magnitude()
    return poses, length, float(angles.sum())


def dense_curve(start, end, schedules):
    count = 101
    while True:
        q = joint_curve(start, end, schedules, count)
        if np.max(np.abs(np.diff(q, axis=0))) <= math.radians(.5):
            return q
        count = count * 2 - 1


def save_plot(model, report, joints, poses, clearances, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(13, 9))
    ax = fig.add_subplot(221, projection="3d")
    ax.plot(*poses[:, :3, 3].T, color="#007fa3", lw=2.5, label="Compact candidate TCP")
    legacy = np.array([report["start_tcp"], [.2927, 0, 1.05], OLD_STAGING, FRONT_TCP])
    ax.plot(*legacy.T, "--", color="#e28c22", label="Legacy commanded waypoints")
    for fraction, alpha in ((0, .7), (.25, .4), (.5, .4), (.75, .4), (1, 1)):
        i = round(fraction * (len(joints) - 1))
        frames = model.frames(joints[i])
        names = ("aubo_base_link", "shoulder_Link", "upperArm_Link", "foreArm_Link", "wrist1_Link", "wrist2_Link", "wrist3_Link")
        arm = np.array([frames[name][:3, 3] for name in names])
        ax.plot(*arm.T, color="#798694", alpha=alpha)
        bar = np.array([poses[i, :3, 3] + sign * .3 * poses[i, :3, 1] for sign in (-1, 1)])
        ax.plot(*bar.T, color="#333333", lw=2, alpha=alpha)
    ax.set(xlabel="base X (m)", ylabel="base Y (m)", zlabel="Z (m)", title="Local geometry preview (not Isaac playback)")
    ax.set_box_aspect((1.4, 1, 1.5))
    ax.view_init(elev=23, azim=-56)
    ax.legend(fontsize=8)
    progress = np.linspace(0, 100, len(joints))
    ax = fig.add_subplot(222)
    for i, name in enumerate(ARM):
        ax.plot(progress, np.degrees(joints[:, i]), label=name.replace("_joint", ""))
    ax.set(xlabel="Path progress (%)", ylabel="Joint angle (degrees)", title="Monotonic joints: no reversal or 2-pi unwinding")
    ax.legend(fontsize=8, ncol=2)
    ax = fig.add_subplot(223)
    ax.plot(progress, np.array(clearances) * 1000, color="#007fa3")
    ax.axhline(8, color="#cc5555", ls="--", label="8 mm screening threshold")
    ax.set(xlabel="Path progress (%)", ylabel="Conservative clearance (mm)", title="Sampled primitive screening")
    ax.legend(fontsize=8)
    ax = fig.add_subplot(224)
    vertical = np.abs(poses[:, 2, 1])
    ax.plot(progress, vertical, label="Absolute vertical component of bar axis")
    ax.plot(progress, poses[:, 2, 3], label="TCP Z (m)")
    ax.set(xlabel="Path progress (%)", title="Translation and rotation happen together")
    ax.legend(fontsize=8)
    fig.suptitle(f"Rack -> front vertical pose | TCP {report['metrics']['tcp_length_m']:.3f} m | "
                 f"orientation arc {report['metrics']['orientation_arc_deg']:.1f} degrees", fontsize=15)
    fig.tight_layout()
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-height", type=float, choices=(.90, 1.05), default=.90)
    parser.add_argument("--samples", type=int, default=120)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--source-joints-file", type=Path,
                        help="live exported trajectory JSON containing start_joints_rad")
    parser.add_argument("--target-joints", type=float, nargs=6,
                        help="restrict the endpoint to a previously tested joint branch")
    args = parser.parse_args()
    model = RobotModel()
    source = json.loads((ROOT / "seer_description/test/results/rebar_pose_planner_slot1_20260927.json").read_text())
    seed = np.array(source["stages"][0]["joints_rad"])
    seed = (seed + np.pi) % (2 * np.pi) - np.pi
    start_tcp = np.array([.2927, 0, args.start_height])
    start = model.ik(start_tcp, START_ROTATION, seed)
    if args.source_joints_file:
        start = np.array(json.loads(args.source_joints_file.read_text())["start_joints_rad"])
        start_tcp = model.tcp(start)[:3, 3]
    if start is None:
        raise RuntimeError("Source IK unavailable")
    # The second seed reaches the same front pose with a different shoulder branch.
    targets = []
    seeds = [args.target_joints] if args.target_joints else [
        (2.597, -.357, -1.451, -1.094, -1.026, 0),
        (1.1973, -.8653, -1.4513, -.5861, .3735, 0)]
    for seed in seeds:
        target = np.array(seed) if args.target_joints else model.ik(FRONT_TCP, FRONT_ROTATION, seed)
        if target is None:
            continue
        for i in range(6):
            variants = [target[i] + n * 2 * np.pi for n in range(-2, 3)
                        if model.lower[i] <= target[i] + n * 2 * np.pi <= model.upper[i]]
            target[i] = min(variants, key=lambda angle: abs(angle - start[i]))
        if model.clearance(target)[0] > .008:
            targets.append(target)
    if not targets:
        raise RuntimeError("No screened front target IK")
    rng = np.random.default_rng(20260928)
    valid = []
    attempts = 0
    for target in targets:
        def evaluate(log_schedule):
            nonlocal attempts
            attempts += 1
            schedule = np.exp(log_schedule)
            joints = joint_curve(start, target, schedule, 61)
            clearance = min(model.clearance(q)[0] for q in joints)
            poses, length, angle = metrics(model, joints)
            cost = length + .25 * angle
            if clearance >= .009:
                valid.append((cost, target.copy(), schedule.copy()))
            return cost + 300 * max(0, .009 - clearance)
        for _ in range(args.samples):
            evaluate(rng.uniform(np.log(.2), np.log(5), 6))
        branch = [entry for entry in valid if np.linalg.norm(entry[1] - target) < .01]
        if branch:
            best = min(branch, key=lambda entry: entry[0])
            minimize(evaluate, np.log(best[2]), method="Nelder-Mead",
                     bounds=[(np.log(.12), np.log(8))] * 6,
                     options={"maxiter": 90, "xatol": .01, "fatol": .001})
        print(f"Branch complete: {attempts} evaluations, {len(valid)} feasible", flush=True)
    if not valid:
        raise RuntimeError("No locally screened compact candidate")
    selected = None
    for cost, target, schedules in sorted(valid, key=lambda entry: entry[0]):
        joints = dense_curve(start, target, schedules)
        checked = [model.clearance(q) for q in joints]
        if min(value for value, _ in checked) >= .008:
            selected = (target, schedules, joints, checked)
            break
    if selected is None:
        raise RuntimeError("Dense screening rejected all candidates")
    target, schedules, joints, checked = selected
    poses, length, angle = metrics(model, joints)
    clearance, nearest = min(checked)
    previous_min = .15 + np.linalg.norm(OLD_STAGING - [.2927, 0, 1.05]) + np.linalg.norm(FRONT_TCP - OLD_STAGING)
    report = {
        "status": "local_candidate_not_simulation_validated",
        "scope": "slot 1, base parked at (6.08, 2.60), yaw -90 degrees",
        "start_tcp": start_tcp.tolist(), "target_tcp": FRONT_TCP.tolist(),
        "source_branch": ("Actual Isaac joint feedback saved immediately after pickup" if args.source_joints_file
                          else "IK reconstructed from the saved GUI source-lift branch; not live joint feedback"),
        "joint_names": ARM, "start_joints_rad": start.tolist(), "target_joints_rad": target.tolist(),
        "schedule_parameters": schedules.tolist(), "evaluations": attempts,
        "metrics": {"tcp_length_m": length, "orientation_arc_deg": math.degrees(angle),
                    "joint_total_travel_rad": float(np.abs(np.diff(joints, axis=0)).sum()),
                    "joint_endpoint_lower_bound_rad": float(np.abs(target - start).sum()),
                    "min_screened_clearance_m": clearance, "nearest_pair": nearest,
                    "samples": len(joints), "max_joint_step_deg": float(np.degrees(np.abs(np.diff(joints, axis=0)).max())),
                    "tcp_endpoint_error_m": float(np.linalg.norm(poses[-1, :3, 3] - FRONT_TCP)),
                    "final_bar_vertical_component": float(abs(poses[-1, 2, 1]))},
        "baseline": {"commanded_tcp_translation_lower_bound_from_090_m": float(previous_min),
                     "commanded_orientation_arc_deg": 270.,
                     "note": "Commanded waypoint comparison only; previous OMPL joint trajectory was not recorded."},
        "limits": ["Conservative cylinders, URDF primitive boxes/spheres and SRDF collision exemptions.",
                   "Includes chassis, arm, fingers, wrist camera, carried 0.62 m bar with 20 mm radius, rack and static tester boxes.",
                   "Sampled screening; not continuous FCL/MoveIt validation or Isaac grasp dynamics.",
                   "Other occupied slots, warehouse fixtures and dynamic tester carriages are not included.",
                   "No execution times or success rates are established. Do not execute without fresh state and MoveIt validation."]
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    np.savez_compressed(args.output_dir / "trajectory.npz", joints_rad=joints,
                        tcp_xyz=poses[:, :3, 3], tcp_quat_xyzw=Rotation.from_matrix(poses[:, :3, :3]).as_quat(),
                        clearance_m=[value for value, _ in checked])
    np.savetxt(args.output_dir / "trajectory.csv", np.column_stack((
        np.linspace(0, 1, len(joints)), joints, poses[:, :3, 3],
        Rotation.from_matrix(poses[:, :3, :3]).as_quat(), [value for value, _ in checked])),
        delimiter=",", header="progress," + ",".join(ARM) +
        ",tcp_x,tcp_y,tcp_z,qx,qy,qz,qw,clearance_m", comments="")
    save_plot(model, report, joints, poses, [value for value, _ in checked], args.output_dir / "comparison.png")
    print(json.dumps(report["metrics"], indent=2), flush=True)


if __name__ == "__main__":
    main()
