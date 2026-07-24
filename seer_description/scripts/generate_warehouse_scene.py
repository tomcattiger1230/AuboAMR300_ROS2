#!/usr/bin/env python3
"""Generate a lightweight, editable warehouse demonstration USD layer."""

from __future__ import annotations

import argparse
from pathlib import Path


def _vec(values):
    return ", ".join(f"{value:g}" for value in values)


def cube(name, position, size, color, collision=True, opacity=None, indent=8):
    pad = " " * indent
    schemas = ' (\n{}    prepend apiSchemas = ["PhysicsCollisionAPI"]\n{})'.format(
        pad, pad
    ) if collision else ""
    lines = [
        f'{pad}def Cube "{name}"{schemas}',
        f"{pad}{{",
        f"{pad}    double size = 1",
        f"{pad}    color3f[] primvars:displayColor = [({_vec(color)})]",
    ]
    if opacity is not None:
        lines.append(f"{pad}    float[] primvars:displayOpacity = [{opacity:g}]")
    if collision:
        lines.append(f"{pad}    bool physics:collisionEnabled = 1")
    lines.extend(
        [
            f"{pad}    double3 xformOp:translate = ({_vec(position)})",
            f"{pad}    double3 xformOp:scale = ({_vec(size)})",
            (
                f'{pad}    uniform token[] xformOpOrder = '
                '["xformOp:translate", "xformOp:scale"]'
            ),
            f"{pad}}}",
        ]
    )
    return "\n".join(lines)


def cylinder(name, position, radius, height, color, indent=8):
    pad = " " * indent
    return "\n".join(
        [
            f'{pad}def Cylinder "{name}" (',
            f'{pad}    prepend apiSchemas = ["PhysicsCollisionAPI"]',
            f"{pad})",
            f"{pad}{{",
            f'{pad}    uniform token axis = "Z"',
            f"{pad}    double radius = {radius:g}",
            f"{pad}    double height = {height:g}",
            f"{pad}    color3f[] primvars:displayColor = [({_vec(color)})]",
            f"{pad}    bool physics:collisionEnabled = 1",
            f"{pad}    double3 xformOp:translate = ({_vec(position)})",
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate"]',
            f"{pad}}}",
        ]
    )


def rack(index, position):
    blue = (0.08, 0.22, 0.48)
    orange = (0.95, 0.38, 0.05)
    wood = (0.48, 0.29, 0.12)
    lines = [
        f'        def Xform "Rack_{index:02d}"',
        "        {",
        f"            double3 xformOp:translate = ({_vec(position)})",
        '            uniform token[] xformOpOrder = ["xformOp:translate"]',
    ]
    for post_index, (x, y) in enumerate(
        ((-1.5, -0.5), (-1.5, 0.5), (1.5, -0.5), (1.5, 0.5)), 1
    ):
        lines.append(
            cube(
                f"Post_{post_index}",
                (x, y, 1.65),
                (0.07, 0.07, 3.3),
                blue,
                indent=12,
            )
        )
    for shelf_index, z in enumerate((0.35, 1.5, 2.7), 1):
        lines.append(
            cube(
                f"Shelf_{shelf_index}",
                (0, 0, z),
                (3.15, 1.05, 0.09),
                orange,
                indent=12,
            )
        )
    for box_index, (x, y, z, sx, sy, sz) in enumerate(
        (
            (-0.9, 0, 0.75, 0.75, 0.75, 0.7),
            (0.1, 0, 0.72, 0.85, 0.7, 0.64),
            (1.0, 0, 1.92, 0.7, 0.72, 0.75),
            (-0.4, 0, 3.02, 1.0, 0.72, 0.55),
        ),
        1,
    ):
        lines.append(
            cube(
                f"Carton_{box_index}",
                (x, y, z),
                (sx, sy, sz),
                wood,
                collision=False,
                indent=12,
            )
        )
    lines.append("        }")
    return "\n".join(lines)


def generate(robot_layer="seer_aubo.usd"):
    header = """#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
    subLayers = [
        @__ROBOT_LAYER__@
    ]
)

over "World"
{
    def Xform "WarehouseDemo"
    {
"""
    header = header.replace("__ROBOT_LAYER__", robot_layer)
    pieces = [header.rstrip()]

    # Building shell and high-contrast navigation markings.
    pieces.extend(
        [
            cube("Floor", (0, 0, -0.1), (24, 18, 0.2), (0.34, 0.36, 0.38)),
            cube("NorthWall", (0, 9.05, 1.5), (24.2, 0.1, 3), (0.72, 0.75, 0.78)),
            cube("WestWall", (-12.05, 0, 1.5), (0.1, 18, 3), (0.72, 0.75, 0.78)),
            cube("EastWall", (12.05, 0, 1.5), (0.1, 18, 3), (0.72, 0.75, 0.78)),
            cube("SouthWallLeft", (-7.5, -9.05, 1.5), (9, 0.1, 3), (0.72, 0.75, 0.78)),
            cube("SouthWallRight", (7.5, -9.05, 1.5), (9, 0.1, 3), (0.72, 0.75, 0.78)),
            cube("CenterLine", (0, 0, 0.006), (0.08, 16, 0.012), (0.95, 0.78, 0.08), collision=False),
            cube("CrossAisle", (0, 0, 0.007), (22, 0.08, 0.014), (0.95, 0.78, 0.08), collision=False),
            cube("RobotSafetyZone", (0, 0, 0.009), (3.5, 3.5, 0.018), (0.12, 0.55, 0.18), collision=False, opacity=0.28),
            cube("LoadingZone", (-7.5, -7.1, 0.009), (7.5, 2.4, 0.018), (0.9, 0.55, 0.05), collision=False, opacity=0.32),
        ]
    )

    rack_positions = [
        (-7.5, 4.4, 0),
        (-3.0, 4.4, 0),
        (3.0, 4.4, 0),
        (7.5, 4.4, 0),
        (-7.5, -4.2, 0),
        (-3.0, -4.2, 0),
        (3.0, -4.2, 0),
        (7.5, -4.2, 0),
    ]
    pieces.extend(rack(index, position) for index, position in enumerate(rack_positions, 1))

    # Loading pallets.
    for pallet_index, (x, y) in enumerate(((-9.5, -7.2), (-7.3, -7.2), (-5.1, -7.2)), 1):
        pieces.append(
            cube(
                f"Pallet_{pallet_index:02d}",
                (x, y, 0.12),
                (1.2, 1.0, 0.24),
                (0.42, 0.23, 0.08),
            )
        )
        pieces.append(
            cube(
                f"PalletLoad_{pallet_index:02d}",
                (x, y, 0.72),
                (0.95, 0.82, 0.95),
                (0.66, 0.42, 0.17),
            )
        )

    # A simple manipulation workcell and conveyor at the east side.
    pieces.extend(
        [
            cube("WorkTable", (6.9, 0, 0.75), (2.2, 1.2, 0.12), (0.16, 0.2, 0.24)),
            cube("WorkTableLegA", (6.05, -0.42, 0.37), (0.12, 0.12, 0.74), (0.16, 0.2, 0.24)),
            cube("WorkTableLegB", (7.75, -0.42, 0.37), (0.12, 0.12, 0.74), (0.16, 0.2, 0.24)),
            cube("WorkTableLegC", (6.05, 0.42, 0.37), (0.12, 0.12, 0.74), (0.16, 0.2, 0.24)),
            cube("WorkTableLegD", (7.75, 0.42, 0.37), (0.12, 0.12, 0.74), (0.16, 0.2, 0.24)),
            cube("WorkpieceA", (6.5, -0.15, 0.94), (0.28, 0.28, 0.28), (0.8, 0.1, 0.08)),
            cylinder("WorkpieceB", (7.1, 0.15, 0.97), 0.18, 0.38, (0.1, 0.45, 0.8)),
            cube("ConveyorBed", (9.2, 0, 0.65), (2.2, 0.9, 0.18), (0.12, 0.14, 0.16)),
            cube("ConveyorLegA", (8.4, 0, 0.3), (0.12, 0.7, 0.6), (0.12, 0.14, 0.16)),
            cube("ConveyorLegB", (10.0, 0, 0.3), (0.12, 0.7, 0.6), (0.12, 0.14, 0.16)),
        ]
    )

    # Static forklift proxy, bollards and charging station provide obstacles.
    pieces.extend(
        [
            cube("ForkliftBody", (-8.4, 1.0, 0.65), (1.5, 0.9, 1.3), (0.95, 0.68, 0.04)),
            cube("ForkliftMast", (-7.55, 1.0, 1.25), (0.16, 0.85, 2.5), (0.1, 0.1, 0.1)),
            cube("ForkliftForkA", (-6.75, 0.7, 0.12), (1.5, 0.12, 0.12), (0.1, 0.1, 0.1)),
            cube("ForkliftForkB", (-6.75, 1.3, 0.12), (1.5, 0.12, 0.12), (0.1, 0.1, 0.1)),
            cube("ChargingStation", (-10.9, 5.9, 0.75), (0.35, 1.5, 1.5), (0.08, 0.32, 0.52)),
        ]
    )
    for bollard_index, (x, y) in enumerate(((5.4, -1.0), (5.4, 1.0), (10.6, -1.0), (10.6, 1.0)), 1):
        pieces.append(cylinder(f"Bollard_{bollard_index}", (x, y, 0.45), 0.1, 0.9, (0.95, 0.72, 0.03)))

    pieces.append(
        """        def Xform "Lighting"
        {
            def DistantLight "Sun"
            {
                float inputs:angle = 0.5
                color3f inputs:color = (0.92, 0.95, 1)
                float inputs:intensity = 1200
                float3 xformOp:rotateXYZ = (25, -30, 15)
                uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]
            }
            def RectLight "Ceiling_A"
            {
                float inputs:height = 4
                float inputs:width = 8
                float inputs:intensity = 1800
                double3 xformOp:translate = (-6, 0, 7)
                float3 xformOp:rotateXYZ = (0, 0, 0)
                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]
            }
            def RectLight "Ceiling_B"
            {
                float inputs:height = 4
                float inputs:width = 8
                float inputs:intensity = 1800
                double3 xformOp:translate = (6, 0, 7)
                float3 xformOp:rotateXYZ = (0, 0, 0)
                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]
            }
        }

        def Camera "OverviewCamera"
        {
            float focalLength = 22
            float horizontalAperture = 20.955
            double3 xformOp:translate = (15, -20, 14)
            float3 xformOp:rotateXYZ = (58, 0, 36)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]
        }
    }
}
"""
    )
    return "\n\n".join(pieces)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "urdf" / "warehouse_demo.usda",
    )
    parser.add_argument(
        "--robot-layer",
        default="seer_aubo.usd",
        help="Robot USD layer referenced by the generated warehouse scene",
    )
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(generate(args.robot_layer), encoding="utf-8")
    print(f"Generated warehouse scene: {args.output}")


if __name__ == "__main__":
    main()
