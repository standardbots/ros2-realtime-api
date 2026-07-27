"""Measure stream-path timing jitter and its effect on streamed-joint tracking.

Three phases:

  1. Passive: inter-arrival statistics of the bridge's robot_joints feed
     (return-path cadence: bridge publish + DDS + client scheduling).
  2. Send-loop: wake-up lateness of this client's paced loop at --rate,
     without publishing — the scheduling noise floor every setpoint inherits.
  3. Motion (skip with --passive-only): stream a raised-cosine wave on ONE
     joint with client-supplied velocities, optionally injecting controlled
     jitter, then report tracking error and the observed joint velocity.

Why: with trustClientStreamVelocity on, the robot reconstructs the command as
FOH(q, v) through a low-pass filter with time constant fohLpfTau. A sample
arriving late by d seconds while the joint moves at v rad/s is a prediction
error of ~v*d rad, which the filter turns into a velocity transient of up to
~v*d/tau rad/s — so the smaller the tau that makes tracking crisp, the harder
real-world jitter kicks. This script puts numbers on both halves: how much
jitter the path actually has (phases 1-2), and what a chosen amount of
injected jitter does to the motion (phase 3).

Injection modes (phase 3):
  none       clean baseline
  delay      with --delay-prob, sleep uniform(0, --delay-max) ms before a
             send; later samples catch up back-to-back on the absolute
             schedule (models a stall then a burst, like real network jitter)
  drop       with --drop-prob, skip a send entirely (models packet loss; the
             robot-side FOH extrapolates through the gap)
  burst      buffer --burst-size samples and send them back-to-back (models
             batching/bufferbloat)
  stale-vel  send the velocity from --stale-ticks ago with the current
             position (kinematically inconsistent (q, v) pairs)

Tracking error is measured against the nominal schedule, so injected
delay/drops show up as error — that is the point.
"""

import time
import math
import random
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from detect_bot_id import detect_bot_id

INJECT_MODES = ['none', 'delay', 'drop', 'burst', 'stale-vel']

# Refuse waves whose commanded peak velocity exceeds this — jitter injection
# deliberately provokes transients on top of it, so keep the base motion slow.
MAX_WAVE_VELOCITY = 0.5  # rad/s


def stats_line(label, values_ms):
    d = np.asarray(values_ms, dtype=float)
    if d.size == 0:
        print(f"  {label}: no samples")
        return
    print(f"  {label}: n={d.size} mean={d.mean():.2f} p50={np.percentile(d, 50):.2f} "
          f"p95={np.percentile(d, 95):.2f} p99={np.percentile(d, 99):.2f} "
          f"max={d.max():.2f} (ms)")


def raised_cosine_wave(start, joint, amplitude, period, cycles, rate):
    """(positions, velocities) arrays for a 1-cos wave on one joint.

    q_j(t) = q0_j + A*(1 - cos(2*pi*t/period))/2 starts and ends every cycle
    at rest, sweeps [q0, q0+A], and has analytic velocity (exactly consistent
    (q, v) pairs — any inconsistency the robot sees is then injected, not
    numerical). All other joints hold their start position with v=0 (also
    consistent). Peak velocity is A*pi/period.
    """
    n = int(round(cycles * period * rate))
    t = np.arange(1, n + 1) / rate
    omega = 2 * math.pi / period

    positions = np.tile(np.asarray(start, dtype=float), (n, 1))
    positions[:, joint] += amplitude * (1 - np.cos(omega * t)) / 2
    velocities = np.zeros_like(positions)
    velocities[:, joint] = amplitude * omega * np.sin(omega * t) / 2
    return positions, velocities


class MeasureStreamJitter(Node):
    """See module docstring."""

    def __init__(self, robot_id: str | None = None):
        super().__init__("measure_stream_jitter")

        if robot_id is None:
            self.robot_id = detect_bot_id(self)
        else:
            self.robot_id = robot_id

        # The external-control bridge publishes/subscribes under /<robot_id>/external
        base = f"/{self.robot_id}/external"

        # Use BEST_EFFORT QoS to match the bridge publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(
            JointState,
            f"{base}/joints",
            10
        )

        self.start_positions = None
        self.joint_names = []
        self.arrivals = []  # (monotonic time, joint positions) while listening
        self.listening = False

        self.subscription = self.create_subscription(
            JointState,
            f"{base}/robot_joints",
            self.joint_state_callback,
            qos_profile
        )

    def joint_state_callback(self, msg):
        if self.start_positions is None and msg.position:
            self.start_positions = list(msg.position)
            self.joint_names = list(msg.name)
        if self.listening and msg.position:
            self.arrivals.append((time.monotonic(), list(msg.position[:6])))

    # ---- phase 1 ----------------------------------------------------------
    def measure_arrival_cadence(self, seconds):
        print(f"[1/3] robot_joints arrival cadence ({seconds:.0f}s idle listen)...")
        self.arrivals = []
        self.listening = True
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.listening = False

        times = np.array([t for t, _ in self.arrivals])
        if times.size < 2:
            print("  no robot_joints received - is the bridge engaged?")
            return
        gaps = np.diff(times) * 1000
        print(f"  publish rate ~{1000.0 / gaps.mean():.1f} Hz")
        stats_line("inter-arrival", gaps)

    # ---- phase 2 ----------------------------------------------------------
    def measure_loop_jitter(self, rate, seconds):
        print(f"[2/3] client send-loop wake-up jitter at {rate:.0f} Hz "
              f"({seconds:.0f}s, no publishing)...")
        dt = 1.0 / rate
        late = []
        t_next = time.monotonic()
        for _ in range(int(seconds * rate)):
            t_next += dt
            sleep_for = t_next - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            late.append(max(0.0, (time.monotonic() - t_next) * 1000))
        stats_line("wake-up lateness", late)
        print("  (scheduling noise floor every sent setpoint inherits)")

    # ---- phase 3 ----------------------------------------------------------
    def stream_wave(self, positions, velocities, rate, inject, args,
                    use_velocities, zero_velocities):
        names = self.joint_names[:6] if len(self.joint_names) >= 6 else \
            [f"joint{i}" for i in range(6)]
        dt = 1.0 / rate
        rng = random.Random(args.seed)

        sent_late_ms = []
        n_delayed = 0
        n_dropped = 0
        burst_buf = []

        self.arrivals = []
        self.listening = True
        t0 = time.monotonic()
        for k in range(len(positions)):
            nominal = t0 + k * dt

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions[k].tolist()
            if zero_velocities:
                msg.velocity = [0.0] * len(names)
            elif use_velocities:
                vk = velocities[max(0, k - args.stale_ticks)] \
                    if inject == 'stale-vel' else velocities[k]
                msg.velocity = vk.tolist()

            if inject == 'delay' and rng.random() < args.delay_prob:
                time.sleep(rng.uniform(0, args.delay_max / 1000.0))
                n_delayed += 1

            if inject == 'drop' and rng.random() < args.drop_prob:
                n_dropped += 1
            elif inject == 'burst':
                burst_buf.append(msg)
                if len(burst_buf) >= args.burst_size:
                    for m in burst_buf:
                        self.publisher.publish(m)
                    burst_buf = []
            else:
                self.publisher.publish(msg)

            sent_late_ms.append(max(0.0, (time.monotonic() - nominal) * 1000))
            rclpy.spin_once(self, timeout_sec=0)

            sleep_for = (t0 + (k + 1) * dt) - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
        for m in burst_buf:
            self.publisher.publish(m)
        self.listening = False

        return t0, sent_late_ms, n_delayed, n_dropped

    def report_wave(self, t0, positions, velocities, joint, rate,
                    sent_late_ms, n_delayed, n_dropped, csv_path):
        print("Send side:")
        stats_line("publish lateness vs schedule", sent_late_ms)
        if n_delayed:
            print(f"  injected delays: {n_delayed}")
        if n_dropped:
            print(f"  injected drops: {n_dropped}")

        if not self.arrivals:
            print("No robot_joints received while streaming - no tracking stats.")
            return

        errors = []
        for t, q_obs in self.arrivals:
            k = int(round((t - t0) * rate))
            if 0 <= k < len(positions) and len(q_obs) == 6:
                errors.append(abs(q_obs[joint] - positions[k][joint]))
        if errors:
            errors = np.array(errors)
            print(f"Tracking joint{joint} vs commanded ({errors.size} samples): "
                  f"error mean {np.degrees(errors.mean()):.2f} deg, "
                  f"max {np.degrees(errors.max()):.2f} deg")

        times = np.array([t for t, _ in self.arrivals])
        q_j = np.array([q[joint] for _, q in self.arrivals])
        dts = np.diff(times)
        valid = dts > 1e-4
        if valid.any():
            v_obs = np.abs(np.diff(q_j)[valid] / dts[valid])
            v_cmd_peak = np.abs(velocities[:, joint]).max()
            print(f"Observed joint{joint} velocity: peak {v_obs.max():.3f} rad/s, "
                  f"p99 {np.percentile(v_obs, 99):.3f} rad/s "
                  f"(commanded peak {v_cmd_peak:.3f} rad/s)")
            print("(observed velocity well above the commanded peak = the "
                  "robot-side filter amplifying jitter, roughly v*delay/tau)")

        print(f"Largest robot_joints gap while streaming: {dts.max() * 1000:.0f} ms")

        final_err = abs(q_j[-1] - positions[-1][joint])
        if final_err > 0.02:
            print(f"WARNING: ended {np.degrees(final_err):.2f} deg from the final "
                  f"setpoint - the arm may have faulted or a stop may have "
                  f"triggered mid-stream (check the move page / ACB logs).")

        if csv_path:
            with open(csv_path, 'w') as f:
                f.write("kind,t,q,v\n")
                for k in range(len(positions)):
                    f.write(f"cmd,{k / rate:.6f},{positions[k][joint]:.6f},"
                            f"{velocities[k][joint]:.6f}\n")
                for t, q in self.arrivals:
                    f.write(f"obs,{t - t0:.6f},{q[joint]:.6f},\n")
            print(f"Wrote {csv_path}")

    # ---- orchestration ----------------------------------------------------
    def start(self, args):
        print("Waiting for start joint state...")
        deadline = time.monotonic() + 10
        while self.start_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.start_positions is None:
            print("No robot_joints received - is the bridge engaged?")
            return

        if len(self.start_positions) < 6:
            print(f"Expected >= 6 joints, got {len(self.start_positions)}")
            return

        self.measure_arrival_cadence(args.observe)
        self.measure_loop_jitter(args.rate, args.observe)

        if args.passive_only:
            print("[3/3] skipped (--passive-only)")
            return

        start = np.asarray(self.start_positions[:6])
        positions, velocities = raised_cosine_wave(
            start, args.joint, args.amplitude, args.period, args.cycles, args.rate)
        peak = float(np.abs(velocities).max())
        if peak > MAX_WAVE_VELOCITY:
            raise SystemExit(
                f"Wave peaks at {peak:.2f} rad/s (limit {MAX_WAVE_VELOCITY}) - "
                f"lower --amplitude or raise --period.")

        vel_mode = 'ON' if not args.no_velocities else 'OFF (position-only)'
        if args.zero_velocities:
            vel_mode = 'ZEROED'
        detail = ''
        if args.inject == 'delay':
            detail = f" (prob {args.delay_prob}, up to {args.delay_max:.0f}ms)"
        elif args.inject == 'drop':
            detail = f" (prob {args.drop_prob})"
        elif args.inject == 'burst':
            detail = f" (size {args.burst_size})"
        elif args.inject == 'stale-vel':
            detail = f" ({args.stale_ticks} ticks)"
        print(f"[3/3] streaming joint{args.joint} wave: {args.cycles} x "
              f"{args.period:.1f}s, amplitude {args.amplitude:.2f} rad, peak "
              f"{peak:.3f} rad/s, velocities {vel_mode}, "
              f"inject {args.inject}{detail}")

        t0, sent_late_ms, n_delayed, n_dropped = self.stream_wave(
            positions, velocities, args.rate, args.inject, args,
            use_velocities=not args.no_velocities,
            zero_velocities=args.zero_velocities)

        print("Done")
        self.report_wave(t0, positions, velocities, args.joint, args.rate,
                         sent_late_ms, n_delayed, n_dropped, args.csv)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Measure external-control stream jitter, and optionally inject '
                    'controlled jitter into a single-joint wave to see how the '
                    'trusted-velocity path (trustClientStreamVelocity + fohLpfTau) '
                    'handles it',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--bot-id', dest='bot_id', type=str, help='Robot ID to stream joints to')
    parser.add_argument('--rate', dest='rate', type=float, default=100.0,
                        help='Setpoint stream rate in Hz')
    parser.add_argument('--observe', dest='observe', type=float, default=5.0,
                        help='Seconds for each passive phase')
    parser.add_argument('--passive-only', dest='passive_only', action='store_true',
                        help='Only measure cadence + loop jitter, do not move the arm')
    parser.add_argument('--joint', dest='joint', type=int, default=5,
                        choices=range(6),
                        help='Joint index to drive (wrist joints are safest)')
    parser.add_argument('--amplitude', dest='amplitude', type=float, default=0.15,
                        help='Wave amplitude in rad')
    parser.add_argument('--period', dest='period', type=float, default=6.0,
                        help='Wave period in seconds (peak velocity = amplitude*pi/period)')
    parser.add_argument('--cycles', dest='cycles', type=int, default=3,
                        help='Number of wave cycles')
    parser.add_argument('--inject', dest='inject', type=str, default='none',
                        choices=INJECT_MODES, help='Jitter to inject while streaming')
    parser.add_argument('--delay-prob', dest='delay_prob', type=float, default=0.05,
                        help='inject=delay: probability a sample is delayed')
    parser.add_argument('--delay-max', dest='delay_max', type=float, default=50.0,
                        help='inject=delay: max injected delay in ms')
    parser.add_argument('--drop-prob', dest='drop_prob', type=float, default=0.05,
                        help='inject=drop: probability a sample is dropped')
    parser.add_argument('--burst-size', dest='burst_size', type=int, default=5,
                        help='inject=burst: samples buffered then sent back-to-back')
    parser.add_argument('--stale-ticks', dest='stale_ticks', type=int, default=5,
                        help='inject=stale-vel: how many ticks old the sent velocity is')
    parser.add_argument('--seed', dest='seed', type=int, default=1,
                        help='RNG seed for the injections (reproducible runs)')
    parser.add_argument('--no-velocities', dest='no_velocities', action='store_true',
                        help='Stream positions only (A/B how the smoothing path '
                             'handles the same jitter)')
    parser.add_argument('--zero-velocities', dest='zero_velocities', action='store_true',
                        help='Full velocity vector of zeros (degenerate trusted mode)')
    parser.add_argument('--csv', dest='csv', type=str, default=None,
                        help='Write commanded + observed samples of the driven joint '
                             'to this CSV (run via ./run_shell.sh so the file lands '
                             'in the mounted repo; ./run.sh containers are removed '
                             'on exit)')
    args = parser.parse_args()

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    node = MeasureStreamJitter(robot_id=args.bot_id)

    print('Measuring stream jitter against robot: ', node.robot_id)

    try:
        if args.zero_velocities and args.no_velocities:
            raise SystemExit("--zero-velocities and --no-velocities are mutually exclusive")
        node.start(args)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
