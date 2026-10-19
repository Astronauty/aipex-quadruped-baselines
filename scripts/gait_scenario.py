#!/usr/bin/env python3
"""Scripted gait test for runs without a physical joystick.

Drives the convex MPC controller the same way the operator does:
  - /start_gait (std_msgs/Bool) releases the stance hold
    (needs wait_for_gait_trigger: true, the config default), and
  - /joy (sensor_msgs/Joy) carries stick commands. The controller reads
    axes[1] as forward, axes[0] as lateral (+ = left) and axes[3] as yaw
    (+ = counter-clockwise), each scaled by its velocity cap (0.15 m/s
    translation, 0.5 rad/s yaw).

The timeline starts when physics starts, detected as the body moving on
/sportmodestate, so the sim can sit paused at the keyframe for as long as it
takes to click Run. Default timeline:
  physics + start_delay           -> release the trot
  release + cmd_delay             -> forward command for cmd_duration
  then                            -> stick centred
The command is (--vx, --vy, --wz), any mix. With --cycles N it instead
alternates sign N times: +cmd for cmd_duration, centred for --pause, -cmd for
cmd_duration, centred for --pause, and so on, then the tail.

--damp-at T presses L2+B (damping) T seconds after the trot release.

Do not run alongside joy_node: its idle stream would overwrite these commands.

Run with the system Python after sourcing setup_local.sh:
  python3 scripts/gait_scenario.py --vx 0.05
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from unitree_go.msg import SportModeState

MAX_TRANSLATION_VEL = 0.15  # must match reference_traj_from_joy()
MAX_YAW_VEL = 0.5           # must match reference_traj_from_joy()
AXIS_X, AXIS_Y, AXIS_YAW, AXIS_L2 = 1, 0, 3, 2
BUTTON_B = 1


class GaitScenario(Node):
    def __init__(self, args):
        super().__init__('gait_scenario')
        self.args = args
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.start_pub = self.create_publisher(Bool, 'start_gait', 1)
        self.create_subscription(SportModeState, 'sportmodestate', self.on_state, 10)

        self.first_pos = None
        self.t_physics = None   # wall time physics was first seen moving
        self.t_release = None   # wall time /start_gait was sent
        self.stick = (0.0, 0.0, 0.0)  # x, y, yaw currently published
        self.damping = False
        self.done = False
        self.create_timer(1.0 / args.rate, self.tick)

        cmd = (args.vx / MAX_TRANSLATION_VEL, args.vy / MAX_TRANSLATION_VEL, args.wz / MAX_YAW_VEL)
        for name, v in zip(('--vx', '--vy', '--wz'), cmd):
            if not -1.0 <= v <= 1.0:
                raise ValueError(f'{name} exceeds its stick range')
        self.stick_cmd = cmd
        self.log(f'waiting for physics; then trot after {args.start_delay:.1f} s, '
                 f'vx {args.vx:+.3f} vy {args.vy:+.3f} m/s wz {args.wz:+.3f} rad/s '
                 f'{args.cmd_delay:.1f} s later for {args.cmd_duration:.1f} s'
                 + (f', {args.cycles} sign-alternating cycles' if args.cycles > 0 else ''))

    def command_span(self):
        if self.args.cycles <= 0:
            return self.args.cmd_duration
        return 2 * self.args.cycles * (self.args.cmd_duration + self.args.pause)

    def stick_at(self, t):
        """Stick (x, y, yaw) at t seconds after the command starts."""
        zero = (0.0, 0.0, 0.0)
        if t < 0 or t >= self.command_span():
            return zero
        if self.args.cycles <= 0:
            return self.stick_cmd
        leg = self.args.cmd_duration + self.args.pause  # one move plus its pause
        k, within = divmod(t, leg)
        if within >= self.args.cmd_duration:
            return zero
        sign = 1.0 if int(k) % 2 == 0 else -1.0
        return tuple(sign * v for v in self.stick_cmd)

    def log(self, text):
        self.get_logger().info(text)

    def on_state(self, msg):
        pos = tuple(msg.position)
        if self.first_pos is None:
            self.first_pos = pos
        elif self.t_physics is None and any(abs(a - b) > 1e-6 for a, b in zip(pos, self.first_pos)):
            self.t_physics = time.monotonic()
            self.log('physics running')

    def tick(self):
        now = time.monotonic()
        if self.t_physics is not None:
            if self.t_release is None and now - self.t_physics >= self.args.start_delay:
                self.start_pub.publish(Bool(data=True))
                self.t_release = now
                self.log('released trot (/start_gait)')
            if self.t_release is not None:
                t = now - self.t_release
                stick = self.stick_at(t - self.args.cmd_delay)
                if stick != self.stick:
                    self.log(f'stick x {stick[0]:+.2f} y {stick[1]:+.2f} yaw {stick[2]:+.2f}')
                    self.stick = stick
                damp = self.args.damp_at is not None and self.args.damp_at <= t < self.args.damp_at + 0.5
                if damp and not self.damping:
                    self.log('pressing L2+B (damping)')
                self.damping = damp
                if t >= self.args.cmd_delay + self.command_span() + self.args.tail:
                    self.done = True

        # Publish continuously, as joy_node does, so the controller always has a
        # current message with all the axes it indexes.
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 11
        joy.axes[AXIS_X], joy.axes[AXIS_Y], joy.axes[AXIS_YAW] = (float(v) for v in self.stick)
        joy.axes[AXIS_L2] = -1.0 if self.damping else 1.0  # analog trigger, rests at +1
        joy.buttons[BUTTON_B] = 1 if self.damping else 0
        self.joy_pub.publish(joy)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--start-delay', type=float, default=10.0, help='s from physics start to trot release')
    p.add_argument('--cmd-delay', type=float, default=10.0, help='s from trot release to the command')
    p.add_argument('--vx', type=float, default=0.0, help='forward command, m/s (cap 0.15)')
    p.add_argument('--vy', type=float, default=0.0, help='lateral command, m/s, + = left (cap 0.15)')
    p.add_argument('--wz', type=float, default=0.0, help='yaw-rate command, rad/s, + = CCW (cap 0.5)')
    p.add_argument('--damp-at', type=float, default=None, help='s after release to press L2+B (damping)')
    p.add_argument('--cmd-duration', type=float, default=3.0, help='s the command is held')
    p.add_argument('--tail', type=float, default=10.0, help='s of centred stick before exiting')
    p.add_argument('--cycles', type=int, default=0, help='forward/backward cycles (0: one forward command)')
    p.add_argument('--pause', type=float, default=2.0, help='s of centred stick between moves with --cycles')
    p.add_argument('--rate', type=float, default=20.0, help='/joy publish rate, Hz')
    args = p.parse_args()

    rclpy.init()
    node = GaitScenario(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.log('scenario complete; stick centred')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
