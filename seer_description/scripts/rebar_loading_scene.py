#!/usr/bin/env python3
"""Load the station and chassis-mounted rack colliders into MoveIt."""
import math
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from rebar_experiment_geometry import SLOT_X, RACK_REBAR_Z, rack_boxes, saddle_boxes


def box_object(identifier, frame, boxes):
    obj = CollisionObject(id=identifier)
    obj.header.frame_id = frame
    obj.operation = CollisionObject.ADD
    for position, size, rpy in boxes:
        obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, size))))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, position)
        roll, pitch, yaw = (angle / 2 for angle in rpy)
        cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                                 math.sin(pitch), math.cos(yaw), math.sin(yaw))
        pose.orientation.x = sr*cp*cy-cr*sp*sy
        pose.orientation.y = cr*sp*cy+sr*cp*sy
        pose.orientation.z = cr*cp*sy-sr*sp*cy
        pose.orientation.w = cr*cp*cy+sr*sp*sy
        obj.primitive_poses.append(pose)
    return obj


def main():
    rclpy.init()
    node = rclpy.create_node("rebar_loading_scene")
    node.declare_parameter("occupied_slots", "")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    try:
        if not client.wait_for_service(timeout_sec=60):
            raise RuntimeError("MoveIt planning scene unavailable")
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        rack = AttachedCollisionObject(link_name="base_link", touch_links=["base_link"])
        rack.object = box_object("onboard_rebar_rack", "base_link",
                                 [(p, s, r) for _, p, s, r in rack_boxes()])
        scene.robot_state.attached_collision_objects = [rack]
        for value in node.get_parameter("occupied_slots").value.split(","):
            if value.strip() in ("", "none"):
                continue
            slot = int(value)
            if slot not in range(1, 5):
                raise ValueError("occupied_slots must be 1–4")
            bar = AttachedCollisionObject(link_name="base_link", touch_links=["base_link"])
            bar.object.id = f"onboard_rebar_slot_{slot}"
            bar.object.header.frame_id = "base_link"
            bar.object.operation = CollisionObject.ADD
            bar.object.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                                    dimensions=[.6, .012])]
            pose = Pose()
            pose.position.x, pose.position.z = SLOT_X[slot-1], RACK_REBAR_Z
            pose.orientation.x = pose.orientation.w = math.sqrt(.5)
            bar.object.primitive_poses = [pose]
            scene.robot_state.attached_collision_objects.append(bar)
        boxes = [((-1.15, 0, .43), (.9, .5, .04), (0, 0, 0))]
        for x in (-1.33, -.97):
            boxes.append(((x, 0, .477), (.1, .06, .054), (0, 0, 0)))
            boxes.extend(saddle_boxes("X", (x, 0, .526), width=.1, radius=.016))
        scene.world.collision_objects = [box_object("rebar_source_station", "world", boxes)]
        future = client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        if not future.done() or not future.result().success:
            raise RuntimeError("MoveIt rejected loading scene")
        node.get_logger().info("Rebar station and four chassis-mounted saddles loaded")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
