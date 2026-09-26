"""CLI glue for the individually runnable rebar mission ROS nodes."""

import sys
import traceback
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from test_rebar_tester_load import TesterLoadTest, build_parser  # noqa: E402


def run_stage(stage):
    parser = build_parser()
    parser.set_defaults(stage=stage)
    args = parser.parse_args()
    if args.stage != stage:
        parser.error(f"this node only runs the {stage} stage")
    if args.plan_only and stage != "plan":
        parser.error("--plan-only is only valid for 01_plan.py")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TesterLoadTest(args)
    try:
        passed = node.run()
    except Exception as exc:
        node.get_logger().error(traceback.format_exc())
        node.record("workflow_error", False, error=str(exc))
        node.write_report()
        passed = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)
