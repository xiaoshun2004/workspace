"""CLI entry for controlling the LeBai gripper through a DOBOT Nova.

Examples:
  python main.py --host 192.168.5.1 --selftest
  python main.py --open --force 50 --speed 80
  python main.py --status
"""

import argparse
import sys

from nova_client import NovaClient
from gripper import LebaiGripper


def build_parser():
    parser = argparse.ArgumentParser(
        prog="pc_lebai_upper",
        description="LeBai LMG-90 gripper control via DOBOT Nova",
    )
    parser.add_argument("--host", default="192.168.5.1", help="Nova controller IP")
    parser.add_argument("--port", type=int, default=29999, help="Nova Dashboard port")
    parser.add_argument("--timeout", type=float, default=5.0, help="TCP timeout (s)")

    parser.add_argument("--selftest", action="store_true", help="run read diagnostics")
    parser.add_argument("--open", action="store_true", help="open gripper to fully open")
    parser.add_argument("--close", action="store_true", help="close gripper to fully closed")
    parser.add_argument("--position", type=int, metavar="0-100", help="set target position")
    parser.add_argument("--force", type=int, metavar="0-100", help="set force")
    parser.add_argument("--speed", type=int, metavar="0-100", help="set speed")
    parser.add_argument("--find-stroke", action="store_true", help="trigger find stroke")
    parser.add_argument("--status", action="store_true", help="read current status")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)

    nova = NovaClient(args.host, args.port, timeout=args.timeout)
    try:
        nova.connect()
        print("[conn] {}:{}".format(args.host, args.port))
    except Exception as exc:
        print("[conn] failed: {}".format(exc))
        return 1

    gripper = LebaiGripper(nova)
    try:
        print("[gripper] init ...")
        index = gripper.init()
        print("[gripper] Modbus index = {}".format(index))

        if args.selftest:
            print("[selftest] status = {}".format(gripper.get_status()))
            if gripper.needs_calibration():
                print("[selftest] not calibrated, find stroke ...")
                ok = gripper.find_stroke()
                print("[selftest] find stroke result = {}".format(ok))
            return 0

        if args.force is not None:
            gripper.set_force(args.force)
            print("[cmd] force = {}".format(args.force))
        if args.speed is not None:
            gripper.set_speed(args.speed)
            print("[cmd] speed = {}".format(args.speed))
        if args.position is not None:
            gripper.set_position(args.position)
            print("[cmd] position = {} (done={})".format(args.position, gripper.wait_done()))
        if args.open:
            gripper.open()
            print("[cmd] open -> 100 (done={})".format(gripper.wait_done()))
        if args.close:
            gripper.close()
            print("[cmd] close -> 0 (done={})".format(gripper.wait_done()))
        if args.find_stroke:
            print("[cmd] find stroke result = {}".format(gripper.find_stroke()))
        if args.status:
            print("[status] {}".format(gripper.get_status()))
        if not any((args.selftest, args.force is not None, args.speed is not None,
                    args.position is not None, args.open, args.close,
                    args.find_stroke, args.status)):
            build_parser().print_help()
            return 0
        return 0
    except Exception as exc:
        print("[error] {}".format(exc))
        return 1
    finally:
        try:
            gripper.release()
        except Exception:
            pass
        nova.close()


if __name__ == "__main__":
    sys.exit(main())
