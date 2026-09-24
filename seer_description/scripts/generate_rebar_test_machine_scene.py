#!/usr/bin/env python3
"""Generate a photo-based rebar tensile tester proxy for the Isaac workcell."""

from __future__ import annotations

import argparse
from pathlib import Path
from textwrap import indent

from generate_etm6m_scene import cube, cylinder, vector


URDF_DIR = Path(__file__).resolve().parent.parent / "urdf"


def machine_usda(x=4.1, y=2.1, yaw=0.0):
    white = (0.86, 0.88, 0.88)
    silver = (0.46, 0.49, 0.51)
    dark = (0.07, 0.08, 0.09)
    red = (0.72, 0.055, 0.06)
    blue = (0.07, 0.15, 0.25)
    pieces = [f'''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)

over "World"
{{
    def Xform "RebarTestMachine"
    {{
        custom string geometryNote = "Photo-based proxy with commanded upper/lower carriages and jaws; dimensions are estimates"
        custom string reference = "User-supplied photo of dual-column tensile test machine"
        double3 xformOp:translate = ({vector((x, y, 0))})
        double xformOp:rotateZ = {yaw:g}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ"]''']
    pieces += [
        # Open two-column frame. Local -X is the front/service face.
        cube("Cabinet", (0, 0, 0.38), (0.82, 1.28, 0.76), blue),
        cube("CabinetFront", (-0.415, 0, 0.36), (0.008, 1.20, 0.66), (0.11, 0.20, 0.31), False),
        cube("DeckRedTrim", (0, 0, 0.775), (0.88, 1.34, 0.04), red),
        cube("LowerPlaten", (-0.08, 0, 0.83), (0.48, 0.48, 0.08), silver),
        cylinder("LeftColumn", (0.03, 0.51, 1.52), 0.07, 1.43, silver),
        cylinder("RightColumn", (0.03, -0.51, 1.52), 0.07, 1.43, silver),
        cylinder("LeftScrew", (-0.015, 0.51, 1.52), 0.026, 1.36, dark, False),
        cylinder("RightScrew", (-0.015, -0.51, 1.52), 0.026, 1.36, dark, False),
        cube("TopBeam", (0.03, 0, 2.25), (0.72, 1.28, 0.18), white),
        cube("TopCap", (0.03, 0, 2.36), (0.76, 0.64, 0.08), white),
        # Separate floor-standing console, matching the reference photograph.
        cube("ConsoleCabinet", (-0.04, -1.15, 0.49), (0.72, 0.76, 0.98), (0.42, 0.45, 0.47)),
        cube("ConsoleTop", (-0.04, -1.15, 1.015), (0.78, 0.80, 0.07), dark),
        cube("ConsoleRedBand", (-0.405, -1.15, 0.90), (0.012, 0.72, 0.05), red, False),
        cube("MonitorStand", (0.08, -1.15, 1.10), (0.06, 0.05, 0.18), dark),
        cube("Monitor", (0.05, -1.15, 1.27), (0.08, 0.44, 0.30), dark),
        cube("MonitorGlass", (0.005, -1.15, 1.27), (0.004, 0.39, 0.25), (0.60, 0.78, 0.83), False),
        cube("ControlStrip", (-0.405, -1.15, 0.96), (0.015, 0.34, 0.06), silver, False),
    ]
    for name, z, direction in (("UpperCarriage", 1.68, 1), ("LowerCarriage", 1.05, -1)):
        pieces += [
            f'''        def Xform "{name}"
        {{
            double3 xformOp:translate = (-0.08, 0, {z:g})
            uniform token[] xformOpOrder = ["xformOp:translate"]''',
            indent(cube("Crosshead", (0.07, 0, direction * 0.19), (0.52, 1.12, 0.15), white, False), "    "),
            indent(cube("GripHead", (0, 0, direction * 0.085), (0.22, 0.25, 0.09), dark, False), "    "),
            indent(cube("JawLeft", (0, 0.07, 0), (0.15, 0.06, 0.13), dark, False), "    "),
            indent(cube("JawRight", (0, -0.07, 0), (0.15, 0.06, 0.13), dark, False), "    "),
            indent(cube("JawLeftTip", (0, 0.07, -direction * 0.075), (0.10, 0.04, 0.035), silver, False), "    "),
            indent(cube("JawRightTip", (0, -0.07, -direction * 0.075), (0.10, 0.04, 0.035), silver, False), "    "),
            "        }",
        ]
    for index, y in enumerate((-1.31, -1.20, -1.09), 1):
        pieces.append(cube(f"ControlButton_{index}", (-0.418, y, 0.96), (0.012, 0.035, 0.024), red, False))
    pieces.append("    }\n}\n")
    return "\n\n".join(pieces)


def scene_usda():
    return '''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
    subLayers = [
        @./warehouse_finger_etm6m_demo.usda@,
        @./rebar_test_machine.usda@
    ]
)
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=URDF_DIR)
    parser.add_argument("--x", type=float, default=4.1, help="Machine centre X in world metres")
    parser.add_argument("--y", type=float, default=2.1, help="Machine centre Y in world metres")
    parser.add_argument("--yaw", type=float, default=0.0, help="Machine yaw in degrees")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "rebar_test_machine.usda").write_text(machine_usda(args.x, args.y, args.yaw), encoding="utf-8")
    (args.output_dir / "warehouse_finger_etm6m_rebar_tester_demo.usda").write_text(scene_usda(), encoding="utf-8")
    print(f"Generated rebar tester scene in {args.output_dir}")


if __name__ == "__main__":
    main()
