"""Record a ROS run (simulation or flight) to the same CSV format as
sim/closed_loop_sim, so analysis/plot_sim.py works on both.

Usage (with the launch file already running):
    python3 analysis/record_ros.py --name cf1 --duration 20 --arm-after 2 out.csv

One row per MPC command (50 Hz), with the latest estimate, reference and,
if available, ground truth (sim bridge only; NaN on hardware).
"""
import argparse
import math
import time

import rclpy
from cf_msgs.msg import AttitudeCommand
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

COLUMNS = ("t,true_px,true_py,true_pz,true_vx,true_vy,true_vz,true_roll,true_pitch,true_yaw,"
           "est_px,est_py,est_pz,est_vx,est_vy,est_vz,est_roll,est_pitch,est_yaw,"
           "ref_px,ref_py,ref_pz,ref_yaw,"
           "cmd_roll,cmd_pitch,cmd_yaw_rate,cmd_thrust_cmd,"
           "stopped,solver_ok,solve_time")
NAN = float("nan")


def euler_zyx(q):
    """[roll, pitch, yaw] from a geometry_msgs Quaternion."""
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def odom_values(msg):
    if msg is None:
        return [NAN] * 9
    p = msg.pose.pose.position
    v = msg.twist.twist.linear
    return [p.x, p.y, p.z, v.x, v.y, v.z, *euler_zyx(msg.pose.pose.orientation)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--name", default="cf1")
    parser.add_argument("--duration", type=float, default=20.0, help="s after arming")
    parser.add_argument("--arm-after", type=float, default=-1.0,
                        help="call safety/arm after this many s (< 0: don't arm)")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("recorder")
    ns = "/" + args.name
    latest = {"truth": None, "est": None, "ref": None, "active": False, "sent": None}
    rows = []
    t0 = None

    node.create_subscription(Odometry, ns + "/ground_truth", lambda m: latest.update(truth=m), 10)
    node.create_subscription(Odometry, ns + "/ekf/odom", lambda m: latest.update(est=m), 10)
    node.create_subscription(PoseStamped, ns + "/reference", lambda m: latest.update(ref=m), 10)
    node.create_subscription(Bool, ns + "/safety/active", lambda m: latest.update(active=m.data), 10)
    node.create_subscription(Twist, ns + "/cmd_vel_legacy", lambda m: latest.update(sent=m), 10)

    def on_command(cmd):
        nonlocal t0
        now = time.monotonic()
        if t0 is None:
            t0 = now
        ref = latest["ref"]
        ref_values = ([ref.pose.position.x, ref.pose.position.y, ref.pose.position.z,
                       euler_zyx(ref.pose.orientation)[2]] if ref else [NAN] * 4)
        # The command actually sent is what the supervisor forwards; while
        # stopped it is zero, otherwise the MPC command (after saturation).
        stopped = 0 if latest["active"] else 1
        sent = [cmd.roll, cmd.pitch, cmd.yaw_rate, cmd.thrust_cmd] if not stopped else [0.0] * 4
        rows.append([now - t0, *odom_values(latest["truth"]), *odom_values(latest["est"]),
                     *ref_values, *sent, stopped, int(cmd.solver_ok), cmd.solve_time])

    node.create_subscription(AttitudeCommand, ns + "/mpc/command", on_command, 10)
    arm_client = node.create_client(Trigger, ns + "/safety/arm")

    start = time.monotonic()
    armed_at = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        elapsed = time.monotonic() - start
        if armed_at is None and 0.0 <= args.arm_after <= elapsed:
            arm_client.wait_for_service(timeout_sec=5.0)
            arm_client.call_async(Trigger.Request())
            armed_at = elapsed
            node.get_logger().info("arm requested")
        if armed_at is not None and elapsed - armed_at > args.duration:
            break
        if args.arm_after < 0 and elapsed > args.duration:
            break

    with open(args.output, "w") as f:
        f.write(COLUMNS + "\n")
        for row in rows:
            f.write(",".join(repr(float(v)) for v in row) + "\n")
    node.get_logger().info(f"wrote {len(rows)} rows to {args.output}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
