#!/usr/bin/env python3
"""Compose rebar loading with two ETM-6M units and one tensile tester."""

from __future__ import annotations

import argparse
from pathlib import Path

from generate_etm6m_scene import equipment_usda
from generate_rebar_test_machine_scene import machine_usda


URDF_DIR = Path(__file__).resolve().parent.parent / "urdf"


def scene_usda():
    return '''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
    subLayers = [
        @./warehouse_finger_rebar_loading_demo.usda@,
        @./etm6m_lab_1.usda@,
        @./etm6m_lab_2.usda@,
        @./rebar_test_machine_lab.usda@
    ]
)

over "World"
{
    over "Cone" (active = false) {}
    over "WarehouseDemo"
    {
        over "Rack_03" (active = false) {}
        over "Rack_04" (active = false) {}
    }
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=URDF_DIR)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "etm6m_lab_1.usda").write_text(
        equipment_usda(2.2, 4.0, yaw=90, name="ETM6M_1"), encoding="utf-8"
    )
    (args.output_dir / "etm6m_lab_2.usda").write_text(
        equipment_usda(4.0, 4.0, yaw=90, name="ETM6M_2"), encoding="utf-8"
    )
    (args.output_dir / "rebar_test_machine_lab.usda").write_text(
        machine_usda(6.0, 4.0, yaw=90), encoding="utf-8"
    )
    (args.output_dir / "warehouse_finger_rebar_lab_demo.usda").write_text(
        scene_usda(), encoding="utf-8"
    )
    print(f"Generated rebar laboratory scene in {args.output_dir}")


if __name__ == "__main__":
    main()
