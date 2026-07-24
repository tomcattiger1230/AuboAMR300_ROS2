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
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni
import omni.graph.core as og
import omni.kit.commands
import usdrt.Sdf
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.rtx import LidarSensor
from pxr import Gf, Sdf, UsdGeom, UsdPhysics


GRAPH_PATH = "/ROS2ControlGraph"
SENSOR_GRAPH_PATH = "/ROS2CameraGraph"
stop_requested = False
runtime_usd_dir = None
sensor_runtimes = []


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


def find_prim_path(stage, prim_name):
    matches = [prim for prim in stage.Traverse() if prim.GetName() == prim_name]
    if not matches:
        raise RuntimeError(f"USD prim named '{prim_name}' was not found")
    minimum_depth = min(str(prim.GetPath()).count("/") for prim in matches)
    shallowest = [
        prim
        for prim in matches
        if str(prim.GetPath()).count("/") == minimum_depth
    ]
    if len(shallowest) != 1:
        paths = ", ".join(str(prim.GetPath()) for prim in matches) or "none"
        raise RuntimeError(
            f"USD prim name '{prim_name}' is ambiguous: {paths}"
        )
    return str(shallowest[0].GetPath())


def configure_stick_robot_drives(stage):
    joint_names = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if prim.IsA(UsdPhysics.Joint)
    }
    if not all(
        name in joint_names for name in ("gripper1_joint", "gripper2_joint")
    ):
        return

    drive_groups = (
        (
            ("left_wheel_joint", "right_wheel_joint"),
            "angular",
            0.0,
            100000.0,
        ),
        (
            (
                "shoulder_joint",
                "upperArm_joint",
                "foreArm_joint",
                "wrist1_joint",
                "wrist2_joint",
                "wrist3_joint",
            ),
            "angular",
            10000.0,
            1000.0,
        ),
        (
            ("gripper1_joint", "gripper2_joint"),
            "linear",
            5000.0,
            500.0,
        ),
    )
    for names, drive_type, stiffness, damping in drive_groups:
        for name in names:
            joint = joint_names.get(name)
            if joint is None:
                raise RuntimeError(f"Stick robot joint was not found: {name}")
            drive = UsdPhysics.DriveAPI.Apply(joint, drive_type)
            drive.CreateStiffnessAttr(stiffness)
            drive.CreateDampingAttr(damping)

    print("Runtime USD: configured stick robot joint drives", flush=True)


def _read_laser_scan_metadata(prim):
    rotation_rate = float(
        prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0
    )
    near_range = float(
        prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0
    )
    far_range = float(
        prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0
    )
    firing_rate = int(
        prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0
    )
    if rotation_rate <= 0 or firing_rate <= 0:
        raise RuntimeError("RTX lidar scan metadata is incomplete")
    return {
        "horizontalFov": 360.0,
        "horizontalResolution": 360.0 * rotation_rate / firing_rate,
        "depthRange": [near_range, far_range],
        "rotationRate": rotation_rate,
        "azimuthRange": [-180.0, 180.0],
    }


def create_lidar_sensors(stage):
    lidar_specs = (
        (
            "front_lidar_link",
            "front_lidar",
            "front_lidar_link",
        ),
        (
            "back_lidar_link",
            "back_lidar",
            "back_lidar_link",
        ),
    )
    created = []
    for link_name, topic_prefix, frame_id in lidar_specs:
        link_path = find_prim_path(stage, link_name)
        lidar_path = f"{link_path}/ros2_lidar"
        success, lidar_prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=lidar_path,
            parent=None,
            translation=Gf.Vec3d(0.0, 0.0, 0.0),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        )
        if not success or not lidar_prim or not lidar_prim.IsValid():
            raise RuntimeError(f"Failed to create RTX lidar: {lidar_path}")
        created.append((lidar_path, topic_prefix, frame_id))
    return created


def attach_lidar_publishers(lidar_specs):
    for lidar_path, topic_prefix, frame_id in lidar_specs:
        sensor = LidarSensor(lidar_path, annotators=[])
        sensor.attach_writer(
            "RtxLidarROS2PublishPointCloud",
            topicName=f"{topic_prefix}/points",
            frameId=frame_id,
        )
        metadata = _read_laser_scan_metadata(
            prim_utils.get_prim_at_path(lidar_path)
        )
        sensor.attach_writer(
            "RtxLidarROS2PublishLaserScan",
            topicName=f"{topic_prefix}/scan",
            frameId=frame_id,
            **metadata,
        )
        sensor_runtimes.append(sensor)


def create_camera_graph(stage):
    camera_mount = find_prim_path(stage, "camera_color_optical_frame")

    camera_path = f"{camera_mount}/ros2_camera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    xform_api = UsdGeom.XformCommonAPI(camera)
    xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    # USD cameras look along local -Z with +Y up. A 180-degree X rotation
    # aligns that convention with the ROS optical frame (+Z forward, +Y down).
    xform_api.SetRotate(
        (180.0, 0.0, 0.0),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetVerticalApertureAttr().Set(15.7)
    camera.GetProjectionAttr().Set("perspective")
    camera.GetFocalLengthAttr().Set(18.0)
    camera.GetFocusDistanceAttr().Set(4.0)

    if stage.GetPrimAtPath(SENSOR_GRAPH_PATH).IsValid():
        stage.RemovePrim(SENSOR_GRAPH_PATH)

    camera_graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": SENSOR_GRAPH_PATH,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                (
                    "CreateRenderProduct",
                    "isaacsim.core.nodes.IsaacCreateRenderProduct",
                ),
                ("PublishRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                (
                    "PublishCameraInfo",
                    "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
                ),
                ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "PublishRgb.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "PublishCameraInfo.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "PublishDepth.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "PublishRgb.inputs:renderProductPath",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "PublishCameraInfo.inputs:renderProductPath",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "PublishDepth.inputs:renderProductPath",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                (
                    "CreateRenderProduct.inputs:cameraPrim",
                    [usdrt.Sdf.Path(camera_path)],
                ),
                ("CreateRenderProduct.inputs:width", 640),
                ("CreateRenderProduct.inputs:height", 480),
                (
                    "PublishRgb.inputs:frameId",
                    "camera_color_optical_frame",
                ),
                (
                    "PublishRgb.inputs:topicName",
                    "camera/color/image_raw",
                ),
                ("PublishRgb.inputs:type", "rgb"),
                (
                    "PublishCameraInfo.inputs:frameId",
                    "camera_color_optical_frame",
                ),
                (
                    "PublishCameraInfo.inputs:topicName",
                    "camera/color/camera_info",
                ),
                (
                    "PublishDepth.inputs:frameId",
                    "camera_color_optical_frame",
                ),
                (
                    "PublishDepth.inputs:topicName",
                    "camera/depth/image_raw",
                ),
                ("PublishDepth.inputs:type", "depth"),
            ],
        },
    )
    og.Controller.evaluate_sync(camera_graph)
    return camera_graph


def limit_camera_publish_rate():
    import omni.syntheticdata._syntheticdata as synthetic_data

    render_product_path = og.Controller.attribute(
        f"{SENSOR_GRAPH_PATH}/CreateRenderProduct.outputs:renderProductPath"
    ).get()
    rgb_rendervar = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        synthetic_data.SensorType.Rgb.name
    )
    depth_rendervar = (
        omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
            synthetic_data.SensorType.DistanceToImagePlane.name
        )
    )
    gate_paths = (
        omni.syntheticdata.SyntheticData._get_node_path(
            rgb_rendervar + "IsaacSimulationGate", render_product_path
        ),
        omni.syntheticdata.SyntheticData._get_node_path(
            depth_rendervar + "IsaacSimulationGate", render_product_path
        ),
        omni.syntheticdata.SyntheticData._get_node_path(
            "PostProcessDispatchIsaacSimulationGate", render_product_path
        ),
    )
    for gate_path in gate_paths:
        og.Controller.attribute(f"{gate_path}.inputs:step").set(3)


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
    app_utils.enable_extension("isaacsim.sensors.experimental.rtx")
    simulation_app.update()

    runtime_usd_path = prepare_runtime_usd()
    opened, stage = stage_utils.open_stage(runtime_usd_path)
    if not opened or stage is None:
        raise RuntimeError(f"Isaac Sim could not open USD stage: {USD_PATH}")
    wait_for_stage()
    simulation_app.update()

    validate_robot(stage)
    configure_stick_robot_drives(stage)
    create_ros_graph(stage)
    lidar_specs = create_lidar_sensors(stage)
    create_camera_graph(stage)
    simulation_app.update()

    stage_utils.set_stage_units(meters_per_unit=1.0)
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    attach_lidar_publishers(lidar_specs)
    limit_camera_publish_rate()
    simulation_app.update()
    app_utils.play()
    simulation_app.update()

    print(
        "Isaac ROS 2 control is ready:\n"
        f"  USD: {USD_PATH}\n"
        f"  articulation: {ARGS.robot_prim}\n"
        f"  commands: {ARGS.command_topic}\n"
        f"  states: {ARGS.state_topic}\n"
        f"  base commands: {ARGS.cmd_vel_topic}\n"
        f"  odometry: {ARGS.odom_topic}\n"
        "  camera: /camera/color/image_raw, /camera/depth/image_raw\n"
        "  lidar scans: /front_lidar/scan, /back_lidar/scan\n"
        "  lidar points: /front_lidar/points, /back_lidar/points",
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
