"""Turn the controller's SwingPlanArray into RViz markers.

All marker construction lives here rather than in the controller: the control
loop has a 25 ms budget and two threads, and a logging call in that path has
already destabilised the gait once. This node only reads a topic and publishes
markers, so it cannot affect control timing.

Everything is drawn in the world frame, which is the frame the swing pipeline
works in. That is deliberate for a debugging tool -- a frame error should look
wrong on screen rather than being absorbed by a transform on the way to the
display.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from convex_mpc_msgs.msg import SwingPlanArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# One hue per leg, so a trajectory can be attributed at a glance.
LEG_COLORS = {
    'FL': (0.95, 0.26, 0.21),   # red
    'FR': (0.30, 0.69, 0.31),   # green
    'RL': (0.13, 0.59, 0.95),   # blue
    'RR': (1.00, 0.76, 0.03),   # amber
}
LEG_ORDER = ['FL', 'FR', 'RL', 'RR']


def _color(rgb, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(rgb[0]), float(rgb[1]), float(rgb[2]), float(a)
    return c


class SwingPlanMarkers(Node):
    def __init__(self):
        super().__init__('swing_plan_markers')

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('show_future_swings', True)
        self.declare_parameter('show_both_foot_sources', True)
        self.frame_id = self.get_parameter('frame_id').value
        self.show_future = self.get_parameter('show_future_swings').value
        self.show_both_feet = self.get_parameter('show_both_foot_sources').value

        # Best-effort in: plans are disposable, and this subscriber must never
        # apply back-pressure that could reach the controller's publisher.
        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Reliable out: a reliable publisher matches both reliable and best-effort
        # subscribers, and Humble's RViz MarkerArray display subscribes reliable
        # whatever its config says -- a best-effort publisher here was silently
        # dropped as QoS-incompatible. Back-pressure from RViz stops at this node.
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(MarkerArray, 'swing_plan_markers', qos_out)
        self.sub = self.create_subscription(SwingPlanArray, 'swing_plan', self.on_plan, qos_in)

        self._warned_empty = False
        self.get_logger().info(
            f'Drawing swing plans in frame "{self.frame_id}" '
            f'-> /swing_plan_markers')

    # ------------------------------------------------------------------
    def on_plan(self, msg: SwingPlanArray):
        markers = MarkerArray()
        stamp = msg.stamp
        mid = 0

        def new_marker(ns, mtype, scale, color):
            nonlocal mid
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = ns
            m.id = mid
            mid += 1
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color = color
            # Markers are republished every cycle; a short lifetime means a stale
            # plan disappears instead of lingering if the controller stops.
            m.lifetime.sec = 0
            m.lifetime.nanosec = 500_000_000
            return m

        # --- Planned swing splines, one line strip per swing.
        for plan in msg.plans:
            rgb = LEG_COLORS.get(plan.leg, (0.8, 0.8, 0.8))
            if not plan.in_flight and not self.show_future:
                continue
            # In-flight swings are drawn solid and thick; not-yet-lifted ones
            # faint, so the committed trajectory stands out from the lookahead.
            alpha = 1.0 if plan.in_flight else 0.35
            width = 0.006 if plan.in_flight else 0.003
            line = new_marker(f'swing_spline_{plan.leg}', Marker.LINE_STRIP,
                              (width, 0.0, 0.0), _color(rgb, alpha))
            line.points = list(plan.sampled_points)
            markers.markers.append(line)

            # Touchdown target: where the Raibert heuristic is aiming.
            tgt = new_marker(f'touchdown_{plan.leg}', Marker.SPHERE,
                             (0.030, 0.030, 0.030), _color(rgb, alpha))
            tgt.pose.position = plan.end_position
            markers.markers.append(tgt)

            # Lift-off point, smaller, to show the swing's span.
            lift = new_marker(f'liftoff_{plan.leg}', Marker.CUBE,
                              (0.018, 0.018, 0.018), _color(rgb, alpha * 0.7))
            lift.pose.position = plan.start_position
            markers.markers.append(lift)

            if plan.in_flight:
                # THE diagnostic: reference versus measured foot position, joined
                # by a line. The length of that line is the swing tracking error
                # (what reads as swing_perr in the health log). A frame bug shows
                # as a consistently mirrored or rotated offset; a tracking lag
                # shows as a line along the direction of travel.
                err = new_marker(f'swing_error_{plan.leg}', Marker.LINE_LIST,
                                 (0.004, 0.0, 0.0), _color((1.0, 0.0, 1.0), 0.9))
                err.points = [plan.reference_position, plan.measured_position]
                markers.markers.append(err)

                ref = new_marker(f'swing_ref_{plan.leg}', Marker.SPHERE,
                                 (0.022, 0.022, 0.022), _color((1.0, 1.0, 1.0), 0.9))
                ref.pose.position = plan.reference_position
                markers.markers.append(ref)

        # --- Measured feet, from both sources, so a disagreement is visible.
        for leg_index, leg in enumerate(LEG_ORDER):
            rgb = LEG_COLORS[leg]
            in_stance = (leg_index < len(msg.stance)) and msg.stance[leg_index]

            if leg_index < len(msg.feet_from_fk):
                fk = new_marker(f'foot_fk_{leg}', Marker.SPHERE,
                                (0.026, 0.026, 0.026),
                                _color(rgb, 1.0 if in_stance else 0.45))
                fk.pose.position = msg.feet_from_fk[leg_index]
                markers.markers.append(fk)

            if self.show_both_feet and leg_index < len(msg.feet_from_sportmode):
                # Drawn as a wireframe-ish smaller cube so it is distinguishable
                # from the FK sphere when the two nearly coincide.
                sm = new_marker(f'foot_sportmode_{leg}', Marker.CUBE,
                                (0.020, 0.020, 0.020), _color((0.6, 0.6, 0.6), 0.65))
                sm.pose.position = msg.feet_from_sportmode[leg_index]
                markers.markers.append(sm)

        # --- Body: measured position, and the commanded reference beside it.
        body = new_marker('body_measured', Marker.SPHERE, (0.05, 0.05, 0.05),
                          _color((1.0, 1.0, 1.0), 0.9))
        body.pose.position = msg.body_pose.position
        markers.markers.append(body)

        ref_body = new_marker('body_reference', Marker.SPHERE, (0.05, 0.05, 0.05),
                              _color((0.0, 1.0, 1.0), 0.45))
        ref_body.pose.position = msg.reference_position
        markers.markers.append(ref_body)

        # Line between them: the body position tracking error.
        body_err = new_marker('body_error', Marker.LINE_LIST, (0.005, 0.0, 0.0),
                              _color((0.0, 1.0, 1.0), 0.8))
        body_err.points = [msg.body_pose.position, msg.reference_position]
        markers.markers.append(body_err)

        # --- Commanded velocity arrow, so the stick input is visible on screen.
        v = msg.reference_velocity
        if abs(v.x) > 1e-4 or abs(v.y) > 1e-4:
            arrow = new_marker('cmd_velocity', Marker.ARROW, (0.012, 0.025, 0.0),
                               _color((1.0, 0.5, 0.0), 0.95))
            tail = msg.body_pose.position
            head = Point()
            # Scaled up so a 0.15 m/s command is actually visible next to a
            # 0.6 m robot; this is a display gain, not a physical length.
            head.x = tail.x + v.x * 2.0
            head.y = tail.y + v.y * 2.0
            head.z = tail.z
            arrow.points = [tail, head]
            markers.markers.append(arrow)

        if not msg.plans and not self._warned_empty:
            self.get_logger().info(
                'Receiving plans with no swings -- expected while the gait is '
                'STAND or still gated behind /start_gait.')
            self._warned_empty = True

        self.pub.publish(markers)


def main():
    rclpy.init()
    node = SwingPlanMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
