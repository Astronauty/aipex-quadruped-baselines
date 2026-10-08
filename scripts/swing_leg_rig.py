#!/usr/bin/env python3
"""Standalone test rig for the swing-leg operational-space controller.

Pins the Go2 base in MuJoCo so the legs can be driven with no coupling to body
dynamics, then tracks the same clamped-cubic swing trajectory the C++ controller
uses and reports how well the Cartesian PD law follows it.

The point is to answer, in isolation from the falling-robot feedback loop:
  * can tau = J^T (Kp (p_ref - p) + Kd (v_ref - v)) track an 8cm/250ms arc at all?
  * what gains does it need?
  * does it stay inside the actuator limit while doing so?

Usage:
    python3 scripts/swing_leg_rig.py                 # default gains
    python3 scripts/swing_leg_rig.py --kp 500 --kd 20
    python3 scripts/swing_leg_rig.py --sweep          # gain sweep table
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

GO2_DIR = Path("/opt/unitree_robotics/unitree_mujoco/unitree_robots/go2")
GO2_XML = GO2_DIR / "go2.xml"

# Matches the C++ side: foot order FL, FR, RL, RR.
LEGS = ["FL", "FR", "RL", "RR"]
JOINTS = ["hip", "thigh", "calf"]

# Nominal stance posture (MuJoCo "home" keyframe): hip 0, thigh 0.9, calf -1.8.
NOMINAL_Q = [0.0, 0.9, -1.8]

SWING_HEIGHT = 0.08
SWING_TIME = 0.25
CTRL_DT = 0.025  # 40Hz, the controller's actual rate


def build_pinned_model():
    """Load go2.xml with the free joint removed, so the base is welded in place.

    A pinned base is what isolates the swing controller: whatever the legs do,
    the body cannot move, so tracking error is purely the leg controller's doing
    rather than the body falling out from under it.
    """
    tree = ET.parse(GO2_XML)
    root = tree.getroot()

    # Drop <freejoint/> on the base so the trunk is welded to the world.
    for body in root.iter("body"):
        for fj in list(body.findall("freejoint")):
            body.remove(fj)

    # The keyframe still describes a 19-dof (free-joint) qpos; with the base
    # welded the model is 12-dof, so it no longer applies. Drop it.
    for kf in list(root.findall("keyframe")):
        root.remove(kf)

    # Absolute meshdir so the file can be parsed from a string outside its dir.
    for comp in root.iter("compiler"):
        comp.set("meshdir", str(GO2_DIR / "assets"))

    # Gravity off: a pinned base bears all weight anyway, and this keeps the test
    # about trajectory tracking rather than gravity compensation, which on the
    # real robot is the MPC's job and not the swing controller's.
    opt = root.find("option")
    if opt is None:
        opt = ET.SubElement(root, "option")
    opt.set("gravity", "0 0 0")

    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


class CubicSwing:
    """Clamped cubic Hermite through lift-off, apex, touchdown.

    Mirrors SwingLegTrajectory in swing_trajectory.hpp: two segments, zero
    velocity at both ends, apex swing_height above the chord midpoint.
    """

    def __init__(self, t0, t2, p0, p2, height):
        self.t0, self.t2 = t0, t2
        t1 = 0.5 * (t0 + t2)
        p1 = 0.5 * (p0 + p2)
        p1 = p1.copy()
        p1[2] += height
        self.knots = [(t0, p0), (t1, p1), (t2, p2)]
        self._segs = [self._hermite(i) for i in range(2)]

    def _hermite(self, i):
        (ta, ya), (tb, yb) = self.knots[i], self.knots[i + 1]
        h0 = self.knots[1][0] - self.knots[0][0]
        h1 = self.knots[2][0] - self.knots[1][0]
        y0, y1, y2 = (k[1] for k in self.knots)
        # Interior slope from the C2 condition with clamped ends (see CubicSpline1D).
        m1 = (6.0 * (y2 - y1) / (h1 * h1) + 6.0 * (y1 - y0) / (h0 * h0)) / (
            4.0 * (1.0 / h0 + 1.0 / h1)
        )
        m = [np.zeros(3), m1, np.zeros(3)]
        mi, mj = m[i], m[i + 1]
        h = tb - ta
        a = ya
        b = mi
        c = 3.0 * (yb - ya) / (h * h) - (2.0 * mi + mj) / h
        d = 2.0 * (ya - yb) / (h**3) + (mi + mj) / (h * h)
        return (ta, a, b, c, d)

    def eval(self, t):
        t = float(np.clip(t, self.t0, self.t2))
        seg = self._segs[0] if t <= self.knots[1][0] else self._segs[1]
        ta, a, b, c, d = seg
        dt = t - ta
        p = a + b * dt + c * dt * dt + d * dt**3
        v = b + 2 * c * dt + 3 * d * dt * dt
        return p, v


def leg_indices(model, leg):
    """Joint qpos/dof indices and foot body id for one leg."""
    qpos, dof = [], []
    for j in JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_{j}_joint")
        if jid < 0:
            raise RuntimeError(f"joint {leg}_{j}_joint not found")
        qpos.append(model.jnt_qposadr[jid])
        dof.append(model.jnt_dofadr[jid])
    # The foot geom is named after the leg; use its body as the tracked frame.
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, leg)
    if gid < 0:
        raise RuntimeError(f"foot geom {leg} not found")
    return np.array(qpos), np.array(dof), model.geom_bodyid[gid], gid


def foot_pos(model, data, gid):
    """World position of the foot collision sphere's centre."""
    return data.geom_xpos[gid].copy()


def foot_jacobian(model, data, body_id, gid, dof):
    """3x3 linear Jacobian of the foot point w.r.t. this leg's three joints."""
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacp, jacr, data.geom_xpos[gid], body_id)
    return jacp[:, dof]


def run_swing(model, leg, kp, kd, torque_limit=23.7, record=False):
    """Drive one leg through a swing arc and measure tracking.

    Returns (rms_error_m, max_error_m, peak_torque_Nm, saturated_fraction).
    """
    data = mujoco.MjData(model)

    qpos, dof, body_id, gid = leg_indices(model, leg)

    # Start at the nominal stance posture and settle.
    for k in range(3):
        data.qpos[qpos[k]] = NOMINAL_Q[k]
    mujoco.mj_forward(model, data)

    p_start = foot_pos(model, data, gid)
    # Step in place: touchdown back at lift-off, so the arc is purely vertical
    # and the test is about tracking rather than footstep placement.
    traj = CubicSwing(0.0, SWING_TIME, p_start, p_start.copy(), SWING_HEIGHT)

    n_sub = max(1, int(round(CTRL_DT / model.opt.timestep)))
    n_ctrl = int(round(SWING_TIME / CTRL_DT))

    errs, taus, sat = [], [], 0
    trace = []

    for i in range(n_ctrl + 1):
        t = i * CTRL_DT
        p_ref, v_ref = traj.eval(t)

        p = foot_pos(model, data, gid)
        J = foot_jacobian(model, data, body_id, gid, dof)
        v = J @ data.qvel[dof]

        f = kp * (p_ref - p) + kd * (v_ref - v)
        tau = J.T @ f
        tau_clamped = np.clip(tau, -torque_limit, torque_limit)
        if np.any(np.abs(tau) > torque_limit * 0.99):
            sat += 1

        errs.append(np.linalg.norm(p_ref - p))
        taus.append(np.max(np.abs(tau)))
        if record:
            trace.append((t, p_ref[2], p[2], np.linalg.norm(p_ref - p), np.max(np.abs(tau))))

        # Hold the commanded torque across the control period.
        for k in range(3):
            data.ctrl[_actuator_for(model, leg, JOINTS[k])] = tau_clamped[k]
        for _ in range(n_sub):
            mujoco.mj_step(model, data)

    return (
        float(np.sqrt(np.mean(np.square(errs)))),
        float(np.max(errs)),
        float(np.max(taus)),
        sat / (n_ctrl + 1),
        trace,
    )


_ACT_CACHE = {}


def _actuator_for(model, leg, joint):
    key = (leg, joint)
    if key not in _ACT_CACHE:
        name = f"{leg}_{joint}"
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid < 0:
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{leg}_{joint}_joint")
        if aid < 0:
            raise RuntimeError(f"actuator for {leg} {joint} not found")
        _ACT_CACHE[key] = aid
    return _ACT_CACHE[key]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kp", type=float, default=200.0)
    ap.add_argument("--kd", type=float, default=10.0)
    ap.add_argument("--leg", default="FR")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--trace", action="store_true")
    args = ap.parse_args()

    model = build_pinned_model()
    limit = float(model.actuator_ctrlrange[0][1])
    print(f"pinned-base Go2, nv={model.nv}, actuator limit = +/-{limit:.1f} Nm")
    print(f"swing: {SWING_HEIGHT*100:.0f}cm arc over {SWING_TIME*1000:.0f}ms, "
          f"control at {1/CTRL_DT:.0f}Hz\n")

    if args.sweep:
        print(f"{'Kp':>7} {'Kd':>6} | {'rms err':>9} {'max err':>9} {'peak tau':>9} {'sat':>6}")
        print("-" * 56)
        for kp in (100, 200, 500, 1000, 2000, 4000):
            for kd in (5, 10, 20, 40):
                rms, mx, pk, sf, _ = run_swing(model, args.leg, kp, kd, limit)
                print(f"{kp:7.0f} {kd:6.0f} | {rms*1000:7.1f}mm {mx*1000:7.1f}mm "
                      f"{pk:7.1f}Nm {sf*100:5.0f}%")
        return

    rms, mx, pk, sf, trace = run_swing(model, args.leg, args.kp, args.kd, limit,
                                       record=args.trace)
    print(f"leg {args.leg}  Kp={args.kp:.0f}  Kd={args.kd:.0f}")
    print(f"  rms tracking error : {rms*1000:.2f} mm")
    print(f"  max tracking error : {mx*1000:.2f} mm")
    print(f"  peak |tau|         : {pk:.2f} Nm  (limit {limit:.1f})")
    print(f"  saturated samples  : {sf*100:.0f}%")

    if args.trace:
        print(f"\n{'t':>6} {'z_ref':>8} {'z_meas':>8} {'err':>8} {'tau':>8}")
        for t, zr, zm, e, tq in trace:
            print(f"{t:6.3f} {zr:8.4f} {zm:8.4f} {e*1000:7.1f}mm {tq:7.1f}")


if __name__ == "__main__":
    main()
