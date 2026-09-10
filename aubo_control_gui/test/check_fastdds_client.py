"""Read feedback, call FK and plan at the current state without executing."""
import json
import time
import rclpy
from aubo_control_gui.motion_client import MotionClient, ARM


def main():
    rclpy.init()
    node = MotionClient()
    if node.get_parameter('enable_motion').value:
        raise RuntimeError('This check must use enable_motion:=false')
    result = {'rmw': rclpy.get_rmw_implementation_identifier()}
    try:
        end = time.monotonic()+30
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.fresh() and node.pose is not None and node.move.server_is_ready():
                break
        result.update(fresh=node.fresh(), moveit=node.move.server_is_ready(), fk=node.pose is not None)
        if not all(result[k] for k in ('fresh','moveit','fk')):
            raise RuntimeError('DDS feedback/FK/MoveIt discovery incomplete')
        result['joints'] = [node.state[n] for n in ARM]
        node.joint_target(result['joints'])
        end = time.monotonic()+20
        while node.busy and time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.05)
        result['plan_error_code'] = node.last_result.result.error_code.val if node.last_result else None
        result['executed'] = False
        if result['plan_error_code'] != 1:
            raise RuntimeError('Plan-only request did not succeed')
    except Exception as exc:
        result['error'] = str(exc)
        raise
    finally:
        print(json.dumps(result, ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
