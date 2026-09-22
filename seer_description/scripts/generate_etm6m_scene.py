#!/usr/bin/env python3
"""Build a dimensioned ETM-6M proxy and a finger-robot workcell scene."""

from __future__ import annotations

import argparse
from pathlib import Path


URDF_DIR = Path(__file__).resolve().parent.parent / "urdf"


def vector(values):
    return ", ".join(f"{value:g}" for value in values)


def shape(kind, name, position, color, *, size=None, radius=None, height=None, collision=True):
    schemas = ' (\n            prepend apiSchemas = ["PhysicsCollisionAPI"]\n        )' if collision else ""
    lines = [
        f'        def {kind} "{name}"{schemas}',
        "        {",
        f"            color3f[] primvars:displayColor = [({vector(color)})]",
    ]
    if kind == "Cube":
        lines += ["            double size = 1", f"            double3 xformOp:scale = ({vector(size)})"]
    else:
        lines += [
            '            uniform token axis = "Z"',
            f"            double radius = {radius:g}",
            f"            double height = {height:g}",
        ]
    if collision:
        lines.append("            bool physics:collisionEnabled = 1")
    lines.append(f"            double3 xformOp:translate = ({vector(position)})")
    order = '["xformOp:translate", "xformOp:scale"]' if kind == "Cube" else '["xformOp:translate"]'
    lines += [f"            uniform token[] xformOpOrder = {order}", "        }"]
    return "\n".join(lines)


def cube(name, position, size, color, collision=True):
    return shape("Cube", name, position, color, size=size, collision=collision)


def cylinder(name, position, radius, height, color, collision=True):
    return shape("Cylinder", name, position, color, radius=radius, height=height, collision=collision)


def equipment_usda(x=2.8, y=0.0, yaw=0.0, name="ETM6M"):
    # The local -X face is the operator side; six stations form a 2 x 3 grid.
    steel = (0.76, 0.80, 0.81)
    dark = (0.10, 0.13, 0.15)
    rim = (0.45, 0.50, 0.52)
    pieces = [f'''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)

over "World"
{{
    def Xform "{name}"
    {{
        custom string sourcePage = "https://yhjtkj.com/list_52/881.html"
        custom string model = "ETM-6M"
        custom string geometryNote = "Approximate visual and collision proxy; outer envelope 0.9 x 1.1 x 0.9 m"
        double3 xformOp:translate = ({vector((x, y, 0))})
        double xformOp:rotateZ = {yaw:g}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ"]''']
    pieces += [
        cube("Cabinet", (0, 0, 0.315), (0.9, 1.1, 0.59), steel),
        cube("FootFL", (-0.36, 0.45, 0.01), (0.07, 0.07, 0.02), dark),
        cube("FootFR", (-0.36, -0.45, 0.01), (0.07, 0.07, 0.02), dark),
        cube("FootRL", (0.36, 0.45, 0.01), (0.07, 0.07, 0.02), dark),
        cube("FootRR", (0.36, -0.45, 0.01), (0.07, 0.07, 0.02), dark),
        cube("FrontPanel", (-0.445, 0, 0.29), (0.008, 1.06, 0.50), (0.88, 0.90, 0.90), False),
        cube("Deck", (0, 0, 0.645), (0.9, 1.1, 0.07), rim),
        cube("RearSplash", (0.405, 0, 0.735), (0.07, 1.1, 0.11), steel),
        cube("LeftSplash", (0, 0.523, 0.735), (0.82, 0.05, 0.11), steel),
        cube("RightSplash", (0, -0.523, 0.735), (0.82, 0.05, 0.11), steel),
        cube("DisplayStand", (0.38, 0, 0.76), (0.09, 0.055, 0.16), rim),
        cube("Display", (0.34, 0, 0.84), (0.07, 0.20, 0.12), dark),
        cube("DisplayGlass", (0.301, 0, 0.84), (0.004, 0.155, 0.09), (0.65, 0.78, 0.82), False),
        cylinder("PressureGauge", (-0.26, -0.45, 0.50), 0.025, 0.012, dark, False),
    ]
    for row, station_x in enumerate((-0.19, 0.15), 1):
        for col, station_y in enumerate((-0.34, 0.0, 0.34), 1):
            prefix = f"Station_{row}_{col}"
            pieces += [
                cylinder(prefix + "_Base", (station_x, station_y, 0.691), 0.0925, 0.022, dark),
                cylinder(prefix + "_Seal", (station_x, station_y, 0.705), 0.078, 0.012, rim, False),
                cylinder(prefix + "_Seat", (station_x, station_y, 0.714), 0.068, 0.007, dark, False),
                cube(prefix + "_Handle", (station_x, station_y, 0.765), (0.02, 0.13, 0.012), rim),
                cube(prefix + "_HandlePostL", (station_x, station_y - 0.055, 0.741), (0.014, 0.014, 0.05), rim),
                cube(prefix + "_HandlePostR", (station_x, station_y + 0.055, 0.741), (0.014, 0.014, 0.05), rim),
            ]
    pieces.append("    }\n}\n")
    return "\n\n".join(pieces)


def scene_usda():
    return '''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
    subLayers = [
        @./warehouse_finger_mono_demo.usda@,
        @./etm6m.usda@
    ]
)
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=URDF_DIR)
    parser.add_argument("--x", type=float, default=2.8, help="Device centre X in world metres")
    parser.add_argument("--y", type=float, default=0.0, help="Device centre Y in world metres")
    parser.add_argument("--yaw", type=float, default=0.0, help="Device yaw in degrees")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "etm6m.usda").write_text(equipment_usda(args.x, args.y, args.yaw), encoding="utf-8")
    (args.output_dir / "warehouse_finger_etm6m_demo.usda").write_text(scene_usda(), encoding="utf-8")
    print(f"Generated ETM-6M scene in {args.output_dir}")


if __name__ == "__main__":
    main()
