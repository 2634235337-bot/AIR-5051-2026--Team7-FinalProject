#!/usr/bin/env python3
import argparse
import sys

import actionlib
import rospy
from sagittarius_object_color_detector.msg import SGRCtrlAction, SGRCtrlGoal, SGRCtrlResult


def result_to_text(code: int) -> str:
    mapping = {
        int(SGRCtrlResult.SUCCESS): "SUCCESS",
        int(SGRCtrlResult.ERROR): "ERROR",
        int(SGRCtrlResult.PREEMPT): "PREEMPT",
        int(SGRCtrlResult.PLAN_NOT_FOUND): "PLAN_NOT_FOUND",
        int(SGRCtrlResult.GRASP_FAILD): "GRASP_FAILD",
    }
    return mapping.get(int(code), f"UNKNOWN({code})")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Smoke test for /<arm_name>/sgr_ctrl using ACTION_TYPE_XYZ."
    )
    parser.add_argument("--arm-name", default="sgr532", help="Arm namespace, default: sgr532")
    parser.add_argument("--x", type=float, default=0.20, help="Target X in meters")
    parser.add_argument("--y", type=float, default=0.00, help="Target Y in meters")
    parser.add_argument("--z", type=float, default=0.10, help="Target Z in meters")
    parser.add_argument("--wait-server", type=float, default=10.0, help="Wait server timeout (s)")
    parser.add_argument("--wait-result", type=float, default=20.0, help="Wait result timeout (s)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    action_name = f"/{args.arm_name}/sgr_ctrl"

    rospy.init_node("sgr_ctrl_smoke_test", anonymous=True)
    cli = actionlib.SimpleActionClient(action_name, SGRCtrlAction)

    print(f"[INFO] waiting for action server: {action_name}")
    ok = cli.wait_for_server(rospy.Duration.from_sec(float(args.wait_server)))
    print(f"[INFO] wait_for_server={ok}")
    if not ok:
        print("[FAIL] action server unavailable")
        return 1

    goal = SGRCtrlGoal()
    goal.action_type = SGRCtrlGoal.ACTION_TYPE_XYZ
    goal.grasp_type = SGRCtrlGoal.GRASP_NONE
    goal.pos_x = float(args.x)
    goal.pos_y = float(args.y)
    goal.pos_z = float(args.z)

    print(
        f"[INFO] sending goal ACTION_TYPE_XYZ to ({goal.pos_x:.4f}, "
        f"{goal.pos_y:.4f}, {goal.pos_z:.4f})"
    )
    cli.send_goal(goal)

    done = cli.wait_for_result(rospy.Duration.from_sec(float(args.wait_result)))
    print(f"[INFO] wait_for_result={done}")
    if not done:
        cli.cancel_goal()
        print("[FAIL] timeout waiting for action result (goal canceled)")
        return 2

    result = cli.get_result()
    if result is None:
        print("[FAIL] result is None")
        return 3

    code = int(result.result)
    text = result_to_text(code)
    print(f"[RESULT] {text}({code})")
    if code != int(SGRCtrlResult.SUCCESS):
        return 4
    return 0


if __name__ == "__main__":
    sys.exit(main())
