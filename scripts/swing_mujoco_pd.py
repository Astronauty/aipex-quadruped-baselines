#!/usr/bin/env python3
"""Drive a swing arc with MuJoCo's own PD servos, to split two hypotheses apart.

The trot diverges with the foot pushed along axes the trajectory never commands.
Two things could be responsible:

  (a) the swing SETPOINTS are wrong -- the spline, or the frames it is expressed
      in, ask for a path the leg cannot or should not follow; or
  (b) our Cartesian PD law (tau = J^T (Kp e_p + Kd e_v), in convex_mpc.cpp) is
      wrong -- sign, frame, or gains.

This script removes (b) entirely. It converts the SAME spline into joint-space
setpoints by inverse kinematics and hands them to MuJoCo `position` actuators,
which are a textbook joint-space PD with no Jacobian and no frame conversion
anywhere in the loop. If the foot then tracks the arc, the setpoints are sound
and the fault is in our PD path. If it still does not track, the setpoints
themselves are wrong and no amount of gain tuning will fix it.

The base is welded (via make_pinned_scene.py), so body dynamics cannot
contaminate the measurement either way.

Usage:
    python3 scripts/make_pinned_scene.py      # once, to generate the scene
    python3 scripts/swing_mujoco_pd.py
    python3 scripts/swing_mujoco_pd.py --leg FL --kp 60 --kd 3
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares

GO2_DIR = Path("/opt/unitree_robotics/unitree_mujoco/unitree_robots/go2")
PINNED_SCENE = GO2_DIR / "scene_pinned.xml"

LEGS = ["FL", "FR", "RL", "RR"]
JOINTS = ["hip", "thigh", "calf"]
NOMINAL_Q = np.array([0.0, 0.9, -1.8])

# Match the controller's configured gait.
SWING_HEIGHT = 0.08
SWING_TIME = 0.5      # default: half of gait_duration_s at duty 0.5


def to_position_actuators(kp: float, kd: float) -> mujoco.MjModel:
    """The pinned scene with every <motor> swapped for a <position> servo.

    `position` actuators implement tau = kp (q_ref - q) - kd qdot internally, so
    the control law under test is MuJoCo's, not ours.
    """
    scene = ET.parse(PINNED_SCENE)
    root = scene.getroot()

    # The robot body lives in the included file; inline it so one tree holds all.
    for inc in list(root.findall("include")):
        child = ET.parse(GO2_DIR / inc.get("file")).getroot()
        root.remove(inc)
        for elem in list(child):
            root.append(elem)

    for actuator in root.iter("actuator"):
        for motor in list(actuator):
            joint = motor.get("joint")
            name = motor.get("name")
            actuator.remove(motor)
            servo = ET.SubElement(actuator, "position")
            servo.set("name", name)
            servo.set("joint", joint)
            servo.set("kp", str(kp))
            servo.set("kv", str(kd))
            # Wide enough not to clip a reasonable command; the point is to test
            # the kinematics, not actuator saturation.
            servo.set("ctrlrange", "-6.28 6.28")

    for comp in root.iter("compiler"):
        comp.set("meshdir", str(GO2_DIR / "assets"))

    # go2.xml uses explicit Euler at 2 ms. With armature 0.01 that caps a stable
    # joint-space PD at roughly kv < 2*armature/dt = 10, so a servo stiff enough to
    # track a 0.5 s arc (kv ~ 28) is numerically unstable: measured at kp=200 the
    # leg was thrown over the top of the robot to z=+0.64, which looks exactly like
    # a control fault and is not one. An implicit integrator removes the limit
    # without changing the mechanism, which is what makes this a fair test of the
    # setpoints.
    opt = root.find("option")
    if opt is None:
        opt = ET.SubElement(root, "option")
    opt.set("integrator", "implicitfast")

    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def leg_indices(model, leg):
    qpos, dof, act = [], [], []
    for j in JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_{j}_joint")
        qpos.append(model.jnt_qposadr[jid])
        dof.append(model.jnt_dofadr[jid])
        act.append(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{leg}_{j}"))
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, leg)
    return np.array(qpos), np.array(dof), np.array(act), gid


def foot_of(model, data, qpos_idx, q, gid):
    """Foot world position at joint configuration q, without disturbing `data`."""
    saved = data.qpos[qpos_idx].copy()
    data.qpos[qpos_idx] = q
    mujoco.mj_kinematics(model, data)
    p = data.geom_xpos[gid].copy()
    data.qpos[qpos_idx] = saved
    mujoco.mj_kinematics(model, data)
    return p


def ik(model, data, qpos_idx, gid, target, seed):
    """Joint angles putting the foot at `target`, from `seed`."""
    res = least_squares(
        lambda q: foot_of(model, data, qpos_idx, q, gid) - target,
        seed, xtol=1e-12, ftol=1e-12,
    )
    return res.x, np.linalg.norm(res.fun)


class CubicSwing:
    """The same clamped cubic with an apex that SwingLegTrajectory builds."""

    def __init__(self, t0, t2, p0, p2, height):
        self.t0, self.t2 = t0, t2
        t1 = 0.5 * (t0 + t2)
        p1 = 0.5 * (p0 + p2)
        p1[2] += height
        h0, h1 = t1 - t0, t2 - t1
        m1 = (6.0 * (p2 - p1) / (h1 * h1) + 6.0 * (p1 - p0) / (h0 * h0)) / (
            4.0 * (1.0 / h0 + 1.0 / h1)
        )
        self.knots = [(t0, p0), (t1, p1), (t2, p2)]
        self.m = [np.zeros(3), m1, np.zeros(3)]

    def eval(self, t):
        t = float(np.clip(t, self.t0, self.t2))
        i = 0 if t <= self.knots[1][0] else 1
        (ta, ya), (tb, yb) = self.knots[i], self.knots[i + 1]
        mi, mj = self.m[i], self.m[i + 1]
        h = tb - ta
        dt = t - ta
        c = 3.0 * (yb - ya) / (h * h) - (2.0 * mi + mj) / h
        d = 2.0 * (ya - yb) / (h**3) + (mi + mj) / (h * h)
        return ya + mi * dt + c * dt * dt + d * dt**3



# Trot phase offsets, matching GaitPlanner::set_phase_offsets() in the C++ side.
# A leg is in stance while (phase + offset) mod 1 < duty_factor.
TROT_PHASE_OFFSETS = {"FL": 0.0, "FR": 0.5, "RL": 0.5, "RR": 0.0}


def run_viewer(model, data, args, swing_template):
    """Watch a full trot gait live, with the reference foot positions drawn.

    Runs the real gait schedule rather than swinging every leg continuously: a
    leg is in stance while (phase + offset) mod 1 < duty_factor, and swings
    through one arc during the remaining fraction. With the default TROT offsets
    (FL/RR at 0.0, FR/RL at 0.5) and duty 0.5 that gives the expected pattern --
    FL+RR swing through the first half of the cycle while FR+RL stand, then the
    diagonals alternate, so each leg lifts and settles exactly ONCE per cycle.

    Stance legs hold the pose they landed in; swing legs track their arc. Real
    time, looping until the window is closed.
    """
    import time
    import mujoco.viewer

    # --swing-time is the SWING duration, so the full cycle follows from the duty
    # factor: swing occupies (1 - duty) of the period.
    period = args.swing_time / (1.0 - args.duty)
    legs = LEGS if args.legs is None else [l.strip() for l in args.legs.split(",") if l.strip()]
    for leg in legs:
        if leg not in LEGS:
            raise SystemExit(f"unknown leg {leg!r}; choose from {LEGS}")

    idx = {leg: leg_indices(model, leg) for leg in LEGS}
    seeds = {leg: NOMINAL_Q.copy() for leg in LEGS}
    stance_q = {}
    nominal_p = {}
    for leg in LEGS:
        qi, _, _, gid = idx[leg]
        stance_q[leg] = data.qpos[qi].copy()
        nominal_p[leg] = foot_of(model, data, qi, data.qpos[qi].copy(), gid)

    print(f"viewer: TROT, period {period:.2f}s (swing {args.swing_time:.2f}s, "
          f"duty {args.duty:.2f}), real time.")
    print("each leg lifts and settles once per cycle; diagonals alternate.")
    print("close the window to stop. red spheres are commanded swing-foot positions.\n")

    swings = {}
    was_swinging = {leg: False for leg in LEGS}
    peak = {leg: 0.0 for leg in LEGS}
    lifts = {leg: 0 for leg in LEGS}

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.4
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -15
        t0 = time.time()
        last_report = [0.0]

        while viewer.is_running():
            phase = (data.time / period) % 1.0
            refs = {}

            for leg in LEGS:
                qi, _, ai, gid = idx[leg]
                adj = (phase + TROT_PHASE_OFFSETS[leg]) % 1.0
                swinging = (adj >= args.duty) and (leg in legs)

                if swinging:
                    if not was_swinging[leg]:
                        # Lift-off: build this swing's arc from where the foot is.
                        p0 = nominal_p[leg].copy()
                        swings[leg] = CubicSwing(
                            0.0, args.swing_time, p0.copy(),
                            p0 + np.array([args.dx, 0.0, 0.0]), args.height)
                        lifts[leg] += 1
                        was_swinging[leg] = True
                    # Time since this leg's lift-off, within the swing window.
                    t_sw = (adj - args.duty) / (1.0 - args.duty) * args.swing_time
                    ref = swings[leg].eval(t_sw)
                    q_ref, _ = ik(model, data, qi, gid, ref, seeds[leg])
                    seeds[leg] = q_ref
                    data.ctrl[ai] = q_ref
                    refs[leg] = ref
                    peak[leg] = max(peak[leg], np.linalg.norm(data.geom_xpos[gid] - ref))
                else:
                    if was_swinging[leg]:
                        # Touchdown: hold whatever pose the foot landed in.
                        stance_q[leg] = data.qpos[qi].copy()
                        was_swinging[leg] = False
                    data.ctrl[ai] = stance_q[leg]

            mujoco.mj_step(model, data)

            if data.time - last_report[0] >= period:
                last_report[0] = data.time
                msg = "  ".join(f"{l} lifts={lifts[l]} peak={peak[l]*1000:5.2f}mm"
                                for l in legs)
                print(f"t={data.time:6.1f}s cycle={int(data.time/period)}  {msg}")

            viewer.user_scn.ngeom = 0
            for leg, ref in refs.items():
                if viewer.user_scn.ngeom < len(viewer.user_scn.geoms):
                    g = viewer.user_scn.geoms[viewer.user_scn.ngeom]
                    mujoco.mjv_initGeom(
                        g, mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([0.012, 0, 0]), ref.astype(np.float64),
                        np.eye(3).flatten(), np.array([1.0, 0.2, 0.2, 1.0], np.float32))
                    viewer.user_scn.ngeom += 1
            viewer.sync()

            ahead = data.time - (time.time() - t0)
            if ahead > 0:
                time.sleep(ahead)

    cycles = max(data.time / period, 1e-9)
    print(f"\nran {data.time:.1f}s = {cycles:.1f} cycles")
    for leg in legs:
        print(f"  {leg}: {lifts[leg]} lift-offs ({lifts[leg]/cycles:.2f} per cycle, "
              f"expect 1.00), peak error {peak[leg]*1000:.2f} mm")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--leg", default="FL", choices=LEGS)
    ap.add_argument("--kp", type=float, default=60.0, help="servo position gain")
    ap.add_argument("--kd", type=float, default=3.0, help="servo damping")
    ap.add_argument("--dx", type=float, default=0.0, help="forward step length (0 = in place)")
    ap.add_argument("--swing-time", type=float, default=SWING_TIME,
                    help="swing duration in seconds; at duty 0.5 this is half the"
                         " gait period (so --swing-time 1.0 is a 2 s gait cycle)")
    ap.add_argument("--height", type=float, default=SWING_HEIGHT,
                    help="peak foot clearance above the lift-off/touchdown chord")
    ap.add_argument("--view", action="store_true",
                    help="open the MuJoCo viewer and run the swing in real time,"
                         " looping until the window is closed")
    ap.add_argument("--legs", default=None,
                    help="restrict swinging to these legs (default: all four, on"
                         " the real trot schedule). Stance legs always hold.")
    ap.add_argument("--duty", type=float, default=0.5,
                    help="duty factor: fraction of the cycle a leg spends in"
                         " stance. With --swing-time, the period is"
                         " swing_time/(1-duty).")
    args = ap.parse_args()

    model = to_position_actuators(args.kp, args.kd)
    data = mujoco.MjData(model)
    qpos_idx, dof_idx, act_idx, gid = leg_indices(model, args.leg)

    # Start from the nominal stance, holding every leg there.
    for leg in LEGS:
        qi, _, ai, _ = leg_indices(model, leg)
        data.qpos[qi] = NOMINAL_Q
        data.ctrl[ai] = NOMINAL_Q
    mujoco.mj_forward(model, data)

    p_start = data.geom_xpos[gid].copy()
    p_end = p_start + np.array([args.dx, 0.0, 0.0])
    swing = CubicSwing(0.0, args.swing_time, p_start.copy(), p_end.copy(), args.height)

    if args.view:
        run_viewer(model, data, args, swing)
        return

    print(f"leg {args.leg}  kp={args.kp} kv={args.kd}  swing {args.swing_time}s  height {args.height}m")
    print(f"lift-off {np.round(p_start,4)} -> touchdown {np.round(p_end,4)}\n")
    print("   t      ref (x y z)              meas (x y z)             err      ik_res")

    dt = model.opt.timestep
    steps = int(args.swing_time / dt)
    seed = NOMINAL_Q.copy()
    errs, lateral = [], []

    for k in range(steps + 1):
        t = k * dt
        ref = swing.eval(t)

        q_ref, res = ik(model, data, qpos_idx, gid, ref, seed)
        seed = q_ref
        data.ctrl[act_idx] = q_ref

        mujoco.mj_step(model, data)
        meas = data.geom_xpos[gid].copy()

        e = np.linalg.norm(meas - ref)
        errs.append(e)
        lateral.append(abs(meas[1] - p_start[1]))

        if k % max(1, steps // 10) == 0:
            print(f"  {t:.3f}  [{ref[0]:+.4f} {ref[1]:+.4f} {ref[2]:+.4f}]  "
                  f"[{meas[0]:+.4f} {meas[1]:+.4f} {meas[2]:+.4f}]  {e:.5f}  {res:.1e}")

    print(f"\n  RMS tracking error : {np.sqrt(np.mean(np.square(errs)))*1000:.2f} mm")
    print(f"  max tracking error : {max(errs)*1000:.2f} mm")
    print(f"  max lateral (y) drift from lift-off: {max(lateral)*1000:.2f} mm")
    print()
    if max(errs) < 0.02 and max(lateral) < 0.02:
        print("  VERDICT: the spline setpoints are kinematically sound. A joint-space")
        print("  PD follows them with no Jacobian and no frame conversion, so the")
        print("  fault lies in our Cartesian PD path, not in the trajectory.")
    else:
        print("  VERDICT: the foot does NOT track even under a plain joint-space PD.")
        print("  The setpoints themselves are suspect -- investigate the spline and")
        print("  the frames it is built in before touching the PD gains.")


if __name__ == "__main__":
    main()
