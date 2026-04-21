import traceback
from robotcontrol import *


ROBOT_IP = "192.168.3.250"
ROBOT_PORT = 8899

# 目标关节角，单位 rad
TARGET_JOINT = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# 是否执行运动；先读参数时可改为 False
ENABLE_MOVE = True

# 运动前是否重新设置速度/加速度上限
SET_PROFILE_BEFORE_MOVE = True
MOVE_MAXACC = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
MOVE_MAXVELC = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)


def print_section(title: str):
    print("\n" + "=" * 20 + f" {title} " + "=" * 20)


def main():
    robot = None
    try:
        # logger_init()  # 暂时注释掉，可能导致问题
        Auboi5Robot.initialize()

        robot = Auboi5Robot()
        handle = robot.create_context()
        print(f"robot context: {handle}")

        result = robot.connect(ROBOT_IP, ROBOT_PORT)
        if result != RobotErrorType.RobotError_SUCC:
            print("connect failed with error code:", result)
            return

        print("connect robot success")

        # 初始化全局运动属性
        robot.init_profile()

        # ---------------- 功能1：读取关节现在参数 ----------------
        print_section("功能1：读取关节现在参数")

        joint_status = robot.get_joint_status()
        print("joint status:")
        print(joint_status)

        current_waypoint = robot.get_current_waypoint()
        print("current waypoint:")
        print(current_waypoint)

        if current_waypoint is not None and "joint" in current_waypoint:
            print("current joint radian:")
            print(tuple(current_waypoint["joint"]))

        # ---------------- 功能2：读取关节最大参数 ----------------
        print_section("功能2：读取关节最大参数")

        joint_maxacc = robot.get_joint_maxacc()
        joint_maxvelc = robot.get_joint_maxvelc()

        print("joint max acceleration (rad/s^2):")
        print(joint_maxacc)

        print("joint max velocity (rad/s):")
        print(joint_maxvelc)

        # ---------------- 功能3：移动机械臂 ----------------
        print_section("功能3：移动机械臂")
        print("target joint radian:")
        print(TARGET_JOINT)

        if ENABLE_MOVE:
            if SET_PROFILE_BEFORE_MOVE:
                robot.set_joint_maxacc(MOVE_MAXACC)
                robot.set_joint_maxvelc(MOVE_MAXVELC)
                print("set move profile success:")
                print("  maxacc =", MOVE_MAXACC)
                print("  maxvelc =", MOVE_MAXVELC)

            move_ret = robot.move_joint(TARGET_JOINT, True)
            print("move_joint return:", move_ret)

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
            robot.disconnect()
        Auboi5Robot.uninitialize()


if __name__ == "__main__":
    main()
