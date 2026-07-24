#!/usr/bin/env python3
"""Load seer_aubo.usd in Isaac Sim and expose its articulation to ROS 2."""

import argparse
import os
import shutil
import signal
import sys
import tempfile


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run the SEER+AUBO USD with a ROS 2 JointState control bridge."
    )
    parser.add_argument("--usd", required=True, help="Absolute path to seer_aubo.usd")
    parser.add_argument(
        "--robot-prim",
        default="/World/seer_aubo_composite/base_footprint",
        help="USD prim carrying ArticulationRootAPI",
    )
    parser.add_argument(
        "--command-topic",
        default="/isaac_joint_commands",
        help="sensor_msgs/JointState command topic",
    )
    parser.add_argument(
        "--state-topic",
        default="/joint_states",
        help="sensor_msgs/JointState feedback topic",
    )
    parser.add_argument(
        "--cmd-vel-topic",
        default="/cmd_vel",
        help="geometry_msgs/Twist mobile-base command topic",
    )
    parser.add_argument(
        "--odom-topic",
        default="/odom",
        help="nav_msgs/Odometry mobile-base feedback topic",
    )
    parser.add_argument("--wheel-radius", type=float, default=0.1)
    parser.add_argument("--wheel-distance", type=float, default=0.6)
    parser.add_argument(
        "--headless", action="store_true", help="Run Isaac Sim without a window"
    )
    parser.add_argument(
        "--renderer",
        default="RealTimePathTracing",
        choices=("RaytracedLighting", "RealTimePathTracing"),
    )
    return parser.parse_args()


ARGS = parse_args()
USD_PATH = os.path.abspath(os.path.expanduser(ARGS.usd))

if not os.path.isfile(USD_PATH):
    raise SystemExit(f"USD file does not exist: {USD_PATH}")

# Isaac Sim 5.1 ships Jazzy and Humble ROS 2 bridge libraries. The surrounding
# shell normally sources Jazzy first; these defaults also make direct execution
# deterministic.
os.environ.setdefault("ROS_DISTRO", "jazzy")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {
        "renderer": ARGS.renderer,
        "headless": ARGS.headless,
    }
)

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Sdf, UsdPhysics


GRAPH_PATH = "/ROS2ControlGraph"
stop_requested = False
runtime_usd_dir = None


def request_stop(_signum, _frame):
    global stop_requested
    stop_requested = True


def wait_for_stage():
    while stage_utils.is_stage_loading() and simulation_app.is_running():
        simulation_app.update()


def prepare_runtime_usd():
    """Copy the stage, anchor its assets, and disable incompatible legacy prims."""
    global runtime_usd_dir

    runtime_usd_dir = tempfile.mkdtemp(prefix="seer_aubo_isaac_")
    runtime_path = os.path.join(runtime_usd_dir, "seer_aubo_runtime.usd")

    source_layer = Sdf.Layer.FindOrOpen(USD_PATH)
    if source_layer is None or not source_layer.Export(runtime_path):
        raise RuntimeError(f"Could not create runtime USD from: {USD_PATH}")

    runtime_layer = Sdf.Layer.FindOrOpen(runtime_path)
    if runtime_layer is None:
        raise RuntimeError(f"Could not open runtime USD: {runtime_path}")

    source_dir = os.path.dirname(USD_PATH)
    for index, layer_path in enumerate(list(runtime_layer.subLayerPaths)):
        if "://" not in layer_path and not os.path.isabs(layer_path):
            runtime_layer.subLayerPaths[index] = os.path.normpath(
                os.path.join(source_dir, layer_path)
            )

    for asset_path in runtime_layer.GetExternalReferences():
        if "://" not in asset_path and not os.path.isabs(asset_path):
            runtime_layer.UpdateExternalReference(
                asset_path,
                os.path.normpath(os.path.join(source_dir, asset_path)),
            )

    stale_sensor_paths = (
        "/World/seer_aubo_composite/front_lidar_link/microScan3_f",
        "/World/seer_aubo_composite/back_lidar_link/microScan3_b",
    )
    disabled = []
    for prim_path in stale_sensor_paths:
        prim_spec = Sdf.CreatePrimInLayer(runtime_layer, prim_path)
        prim_spec.SetInfo("active", False)
        disabled.append(prim_path)

    legacy_graph_path = "/World/ActionGraph"
    legacy_graph_spec = Sdf.CreatePrimInLayer(runtime_layer, legacy_graph_path)
    legacy_graph_spec.SetInfo("active", False)

    runtime_layer.Save()
    if disabled:
        print(
            "Runtime USD: disabled unavailable Isaac 5.1 SICK sensors:\n  "
            + "\n  ".join(disabled),
            flush=True,
        )
    print(
        f"Runtime USD: disabled legacy graph: {legacy_graph_path}",
        flush=True,
    )
    return runtime_path


def validate_robot(stage):
    robot_prim = stage.GetPrimAtPath(ARGS.robot_prim)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Robot prim was not found: {ARGS.robot_prim}")
    if not robot_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise RuntimeError(
            f"Prim does not have ArticulationRootAPI: {ARGS.robot_prim}"
        )


def create_ros_graph(stage):
    legacy_graph = "/World/ActionGraph"
    legacy_graph_prim = stage.GetPrimAtPath(legacy_graph)
    if legacy_graph_prim.IsValid() and legacy_graph_prim.IsActive():
        legacy_graph_prim.SetActive(False)
        print(f"Runtime USD: disabled legacy graph: {legacy_graph}", flush=True)

    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                (
                    "SubscribeJointState",
                    "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                ),
                (
                    "ArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinear", "omni.graph.nodes.BreakVector3"),
                ("BreakAngular", "omni.graph.nodes.BreakVector3"),
                (
                    "DifferentialController",
                    "isaacsim.robot.wheeled_robots.DifferentialController",
                ),
                (
                    "BaseArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                (
                    "PublishOdomTransform",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                (
                    "ReadJointState.outputs:jointNames",
                    "PublishJointState.inputs:jointNames",
                ),
                (
                    "ReadJointState.outputs:jointPositions",
                    "PublishJointState.inputs:jointPositions",
                ),
                (
                    "ReadJointState.outputs:jointVelocities",
                    "PublishJointState.inputs:jointVelocities",
                ),
                (
                    "ReadJointState.outputs:jointEfforts",
                    "PublishJointState.inputs:jointEfforts",
                ),
                (
                    "ReadJointState.outputs:jointDofTypes",
                    "PublishJointState.inputs:jointDofTypes",
                ),
                (
                    "ReadJointState.outputs:stageMetersPerUnit",
                    "PublishJointState.inputs:stageMetersPerUnit",
                ),
                (
                    "ReadJointState.outputs:sensorTime",
                    "PublishJointState.inputs:sensorTime",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "SubscribeJointState.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "ArticulationController.inputs:execIn",
                ),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                (
                    "SubscribeJointState.outputs:jointNames",
                    "ArticulationController.inputs:jointNames",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishClock.inputs:timeStamp",
                ),
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishOdomTransform.inputs:execIn",
                ),
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                (
                    "OnPlaybackTick.outputs:tick",
                    "BaseArticulationController.inputs:execIn",
                ),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                ("Context.outputs:context", "PublishOdometry.inputs:context"),
                (
                    "Context.outputs:context",
                    "PublishOdomTransform.inputs:context",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishOdometry.inputs:timeStamp",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishOdomTransform.inputs:timeStamp",
                ),
                ("SubscribeTwist.outputs:execOut", "DifferentialController.inputs:execIn"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                ("BreakLinear.outputs:x", "DifferentialController.inputs:linearVelocity"),
                ("BreakAngular.outputs:z", "DifferentialController.inputs:angularVelocity"),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "BaseArticulationController.inputs:velocityCommand",
                ),
                (
                    "ComputeOdometry.outputs:angularVelocity",
                    "PublishOdometry.inputs:angularVelocity",
                ),
                (
                    "ComputeOdometry.outputs:linearVelocity",
                    "PublishOdometry.inputs:linearVelocity",
                ),
                (
                    "ComputeOdometry.outputs:orientation",
                    "PublishOdometry.inputs:orientation",
                ),
                (
                    "ComputeOdometry.outputs:position",
                    "PublishOdometry.inputs:position",
                ),
                (
                    "ComputeOdometry.outputs:orientation",
                    "PublishOdomTransform.inputs:rotation",
                ),
                (
                    "ComputeOdometry.outputs:position",
                    "PublishOdomTransform.inputs:translation",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", ARGS.robot_prim),
                (
                    "ReadJointState.inputs:prim",
                    [usdrt.Sdf.Path(ARGS.robot_prim)],
                ),
                ("SubscribeJointState.inputs:topicName", ARGS.command_topic),
                ("PublishJointState.inputs:topicName", ARGS.state_topic),
                ("SubscribeTwist.inputs:topicName", ARGS.cmd_vel_topic),
                (
                    "DifferentialController.inputs:wheelRadius",
                    ARGS.wheel_radius,
                ),
                (
                    "DifferentialController.inputs:wheelDistance",
                    ARGS.wheel_distance,
                ),
                ("DifferentialController.inputs:maxLinearSpeed", 1.5),
                ("DifferentialController.inputs:maxAngularSpeed", 1.0),
                (
                    "BaseArticulationController.inputs:robotPath",
                    ARGS.robot_prim,
                ),
                (
                    "BaseArticulationController.inputs:jointNames",
                    ["left_wheel_joint", "right_wheel_joint"],
                ),
                (
                    "ComputeOdometry.inputs:chassisPrim",
                    [usdrt.Sdf.Path(ARGS.robot_prim)],
                ),
                ("PublishOdometry.inputs:topicName", ARGS.odom_topic),
                ("PublishOdometry.inputs:chassisFrameId", "base_footprint"),
                ("PublishOdomTransform.inputs:parentFrameId", "odom"),
                (
                    "PublishOdomTransform.inputs:childFrameId",
                    "base_footprint",
                ),
            ],
        },
    )


def main():
    global stop_requested

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    app_utils.enable_extension("isaacsim.ros2.bridge")
    app_utils.enable_extension("isaacsim.robot.wheeled_robots.nodes")
    simulation_app.update()

    runtime_usd_path = prepare_runtime_usd()
    opened, stage = stage_utils.open_stage(runtime_usd_path)
    if not opened or stage is None:
        raise RuntimeError(f"Isaac Sim could not open USD stage: {USD_PATH}")
    wait_for_stage()
    simulation_app.update()

    validate_robot(stage)
    create_ros_graph(stage)
    simulation_app.update()

    stage_utils.set_stage_units(meters_per_unit=1.0)
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    simulation_app.update()

    print(
        "Isaac ROS 2 control is ready:\n"
        f"  USD: {USD_PATH}\n"
        f"  articulation: {ARGS.robot_prim}\n"
        f"  commands: {ARGS.command_topic}\n"
        f"  states: {ARGS.state_topic}\n"
        f"  base commands: {ARGS.cmd_vel_topic}\n"
        f"  odometry: {ARGS.odom_topic}",
        flush=True,
    )

    while simulation_app.is_running() and not stop_requested:
        simulation_app.update()

    app_utils.stop()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Isaac ROS 2 runner failed: {exc}", file=sys.stderr, flush=True)
        raise
    finally:
        simulation_app.close()
        if runtime_usd_dir is not None:
            shutil.rmtree(runtime_usd_dir, ignore_errors=True)
