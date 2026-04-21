import traceback
from robotcontrol import *

ROBOT_IP = "192.168.3.250"
ROBOT_PORT = 8899

ENABLE_MOVE = True

SET_PROFILE_BEFORE_MOVE = True
MOVE_MAXACC = (0.3, 0.3, 0.3, 0.3, 0.3, 0.3)
MOVE_MAXVELC = (0.3, 0.3, 0.3, 0.3, 0.3, 0.3)

# 基座坐标系下的相对位移，先尽量小
RELATIVE_POS_ON_BASE = (-0.1, -0.1, -0.2)   # 先试 2 mm
RELATIVE_ORI_ON_BASE = (1.0, 0.0, 0.0, 0.0) # 这里只占位，实际保持当前姿态


def print_section(title: str):
    print("\n" + "=" * 20 + f" {title} " + "=" * 20)


def main():
    robot = None
    try:
        Auboi5Robot.initialize()

        robot = Auboi5Robot()
        handle = robot.create_context()
        print(f"robot context: {handle}")

        result = robot.connect(ROBOT_IP, ROBOT_PORT)
        if result != RobotErrorType.RobotError_SUCC:
            print("connect failed with error code:", result)
            return

        print("connect robot success")

        # 建议先上电
        startup_ret = robot.robot_startup()
        print("robot_startup return:", startup_ret)

        # 初始化全局运动属性
        robot.init_profile()

        if SET_PROFILE_BEFORE_MOVE:
            robot.set_joint_maxacc(MOVE_MAXACC)
            robot.set_joint_maxvelc(MOVE_MAXVELC)
            print("set move profile success:")
            print("  maxacc =", MOVE_MAXACC)
            print("  maxvelc =", MOVE_MAXVELC)

        # ---------------- 1. 设置基座坐标系 ----------------
        print_section("步骤1：设置基座坐标系")

        base_ret = robot.set_base_coord()
        print("set_base_coord return:", base_ret)
        if base_ret != RobotErrorType.RobotError_SUCC:
            print("set_base_coord failed")
            return

        # ---------------- 2. 读取当前状态 ----------------
        print_section("步骤2：读取当前状态")

        current_waypoint = robot.get_current_waypoint()
        print("current waypoint:")
        print(current_waypoint)

        if current_waypoint is None:
            print("get_current_waypoint failed")
            return

        current_joint = tuple(current_waypoint["joint"])
        current_pos = tuple(current_waypoint["pos"])
        current_ori = tuple(current_waypoint["ori"])

        print("current joint radian:")
        print(current_joint)
        print("current flange position on base (m):")
        print(current_pos)
        print("current flange orientation on base (quat wxyz):")
        print(current_ori)

        # ---------------- 3. 构造目标位姿 ----------------
        print_section("步骤3：构造目标位姿")

        target_pos = (
            current_pos[0] + RELATIVE_POS_ON_BASE[0],
            current_pos[1] + RELATIVE_POS_ON_BASE[1],
            current_pos[2] + RELATIVE_POS_ON_BASE[2],
        )

        # move_line 时先保持当前姿态不变
        target_ori = current_ori

        print("relative position on base (m):", RELATIVE_POS_ON_BASE)
        print("target flange position on base (m):", target_pos)
        print("target flange orientation on base (quat wxyz):", target_ori)

        # ---------------- 4. 逆解得到目标关节 ----------------
        print_section("步骤4：逆解目标关节")

        ik_result = robot.inverse_kin(current_joint, target_pos, target_ori)
        print("inverse_kin result:")
        print(ik_result)

        if ik_result is None or "joint" not in ik_result:
            print("inverse_kin failed，目标位姿不可达")
            return

        target_joint = tuple(ik_result["joint"])
        print("target joint radian:")
        print(target_joint)

        # ---------------- 5. 用 move_line 执行 ----------------
        print_section("步骤5：move_line 执行")

        if ENABLE_MOVE:
            move_ret = robot.move_line(target_joint)
            print("move_line return:", move_ret)

            after_move = robot.get_current_waypoint()
            print("waypoint after move:")
            print(after_move)
        else:
            print("ENABLE_MOVE = False，已跳过机械臂运动。")

    except RobotError as e:
        print("robot error:", e)
    except Exception:
        print("unexpected exception:")
        traceback.print_exc()
    finally:
        if robot is not None and getattr(robot, "connected", False):
            try:
                robot.disconnect()
            except Exception:
                pass
        Auboi5Robot.uninitialize()


if __name__ == "__main__":
    main()