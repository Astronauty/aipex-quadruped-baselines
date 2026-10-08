#!/usr/bin/env python3
"""Generate a MuJoCo scene with the Go2 base welded to the world.

Why this exists
---------------
The trot falls over, and the useful diagnostic question is whether the swing
trajectories and the MPC's GRF solve are themselves correct, separately from the
body dynamics they are coupled to. With the base welded the body cannot
translate or tip, so:

  * swing legs track their splines against a body that is not sliding out from
    under them (the measured failure mode: the world-frame foot target stayed
    pinned while body_W drifted 0.4 -> 0.6 m, dragging the body-frame reference
    away from the foot);
  * the MPC solves against a constant body pose, so stance GRFs can be inspected
    without a diverging state feeding back into the next solve;
  * anything still wrong is in the controller, not in the plant coupling.

This differs from scripts/swing_leg_rig.py, which re-implements the spline and
the PD law in Python. That rig answers "can this control law track this arc at
all". This scene runs the *real* C++ GaitPlanner, SwingLegTrajectory and Gurobi
solve against the real sim and the real ROS topics -- it is the integration test,
not the unit test.

The upstream go2.xml is never modified: this reads it and writes a separate
pinned copy plus a scene that includes it, so the normal sim is untouched.

Usage
-----
    python3 scripts/make_pinned_scene.py            # write the pinned scene
    python3 scripts/make_pinned_scene.py --height Z # weld at a specific height

Then point the simulator at it:

    robot_scene: "scene_pinned.xml"     in simulate/config.yaml

and restore `scene.xml` when finished.
"""

import argparse
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

GO2_DIR = Path("/opt/unitree_robotics/unitree_mujoco/unitree_robots/go2")
SRC_XML = GO2_DIR / "go2.xml"
PINNED_XML = GO2_DIR / "go2_pinned.xml"
PINNED_SCENE = GO2_DIR / "scene_pinned.xml"

# Height the controller should *observe*, i.e. what /sportmodestate reports.
# The controller commands STANDING_HEIGHT_M = 0.312 (convex_mpc_node.hpp), so
# welding to that observed height means the MPC starts with ~zero height error
# instead of spending ~35s converging -- which is much of the point of pinning.
#
# This is NOT the same as the base_link pos: body state is read from the `imu`
# site, which sits ~0.042 m above the base origin. Welding base_link at 0.312
# would make the controller see 0.354 and chase a 4cm error it can never remove,
# since a welded body cannot respond to GRFs. The offset is measured from the
# model rather than hardcoded, so it stays right if the site moves.
# Default is the straight-leg clearance (0.426 m drop + 0.0423 m imu offset),
# rounded up, NOT the controller's 0.312 m standing height. Welding at the
# standing height puts the feet 0.156 m through the floor at qpos0; see
# KNOWN LIMITATION in build_pinned_xml().
#
# At this height the feet clear the floor at startup but cannot reach it in
# stance (they hang ~0.16 m up), so there is no ground contact. That makes this
# default suitable for testing SWING tracking in free space, and unsuitable for
# anything that needs stance reaction forces. Pass --height 0.312 to match the
# controller's commanded height and get stance contact, at the cost of the
# startup artifact described below.
DEFAULT_OBSERVED_HEIGHT = 0.47

# Matches the "home" keyframe: hip 0, thigh 0.9, calf -1.8 per leg.
NOMINAL_LEG_Q = [0.0, 0.9, -1.8]



def imu_site_offset() -> float:
    """Height of the `imu` site above the base_link origin, from the model itself.

    The controller's body height comes from this site (the sim publishes it as
    /sportmodestate position), so the weld height has to be corrected by it for
    the observed height to match what was asked for.
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(SRC_XML))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu")
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if sid < 0 or bid < 0:
        raise RuntimeError("imu site or base_link body not found")
    return float(data.site_xpos[sid][2] - data.xpos[bid][2])


def build_pinned_xml(height: float) -> str:
    """go2.xml with the base <freejoint/> removed and the trunk welded at `height`."""
    tree = ET.parse(SRC_XML)
    root = tree.getroot()

    base = None
    for body in root.iter("body"):
        if body.get("name") == "base_link":
            base = body
            break
    if base is None:
        raise RuntimeError("base_link body not found in go2.xml")

    freejoints = list(base.findall("freejoint"))
    if not freejoints:
        raise RuntimeError("base_link has no <freejoint/>; already pinned?")
    for fj in freejoints:
        base.remove(fj)

    # With no free joint the body sits wherever its pos attribute puts it.
    pos = [float(v) for v in base.get("pos", "0 0 0").split()]
    base.set("pos", f"{pos[0]} {pos[1]} {height}")

    # The home keyframe describes a 19-dof qpos (7 free-joint + 12 leg). Welding
    # drops it to 12, so the keyframe no longer matches and MuJoCo rejects it.
    # Re-emit it with only the leg joints so `ctrl` defaults stay available.
    for kf in list(root.findall("keyframe")):
        root.remove(kf)
    keyframe = ET.SubElement(root, "keyframe")
    leg_q = " ".join(str(v) for v in NOMINAL_LEG_Q * 4)

    # ctrl must be ZERO, not the joint angles.
    #
    # The Go2 model drives every leg joint with a plain <motor>, which is a pure
    # TORQUE source -- ctrl is a torque in Nm, not a position setpoint. Copying
    # leg_q into ctrl applied a constant 0.9 Nm to every thigh and -1.8 Nm to
    # every calf from the instant of reset. On the free base the feet reach the
    # floor and this is masked, but welded the legs hang and the bias drives them
    # straight into their joint stops: measured, with NO controller running, FL
    # hip +1.049 (limit 1.0472), RR/RL thigh +4.5385 (limit 4.5379) and all three
    # calves at -2.727 (limit -2.7227), while only FR stayed at the keyframe.
    # The legs were collapsed before the controller sent a single torque.
    zero_ctrl = " ".join(["0"] * 12)
    ET.SubElement(keyframe, "key", {"name": "home", "qpos": leg_q, "ctrl": zero_ctrl})

    # NOTE on the startup pose, and why the weld height matters.
    #
    # unitree_mujoco never loads a keyframe and never writes qpos: it starts from
    # MuJoCo's qpos0, which is 0 for every joint unless a joint sets `ref`. At
    # qpos0 the legs are STRAIGHT, reaching 0.426 m below the base.
    #
    # If the base is welded below that, the feet start *inside the floor* -- at a
    # 0.2697 m weld they begin 0.156 m underground. The contact solver shoves them
    # out sideways, the hips are driven onto their +-1.0472 limits before the
    # controller can fold the legs, and the MPC then recovers into a badly skewed
    # posture (measured PB_X 0.489/0.504/0.113/0.117 instead of a healthy
    # 0.267/0.267/-0.167/-0.168). Swing trajectories seeded from those foot
    # positions ask for targets outside the leg's workspace, and the swing PD then
    # pushes at max torque against a joint stop. All of that is an artifact of the
    # scene, not of the controller: the same controller on the free-base scene
    # starts from the correct geometry, because there the base spawns at 0.445 and
    # the straight legs clear the ground.
    #
    # So the weld height has to leave the straight-leg pose above the floor. That
    # is why DEFAULT_OBSERVED_HEIGHT is not the controller's 0.312 m standing
    # height: pass --height to match whatever the controller commands, and keep it
    # at or above the straight-leg clearance.
    #
    # KNOWN LIMITATION -- no weld height satisfies both constraints.
    #
    # Straight legs (qpos0) drop 0.426 m below the base; the nominal stance drops
    # 0.266 m. So a weld high enough to clear the floor at startup leaves the
    # stance feet ~0.16 m in the air, and a weld low enough for stance contact
    # starts the feet underground. There is no floor height that fixes both
    # either, since the floor would have to sit between the two.
    #
    # Approaches tried and rejected, recorded so they are not retried:
    #   * joint `ref` at the nominal stance -- moves qpos0, but `ref` redefines
    #     where the joint reads zero, so the mechanism rotates with it: qpos
    #     reports [0, 0.9, -1.8] while the leg is still straight and the foot
    #     still 0.426 m below the base. Sim geometry and controller FK would
    #     disagree, which is the class of fault this scene exists to rule out.
    #   * a joint spring biasing the legs toward stance -- the controller is
    #     supposed to stabilise from an arbitrary pose, so propping the pose up
    #     hides the behaviour under test.
    #   * tightening the calf limit so the straight pose is unreachable -- would
    #     also stop the leg extending during swing, changing the mechanism.
    #
    # The real fix is for the simulator to start from the home keyframe, but
    # unitree_mujoco never writes qpos and MuJoCo's simulate loads qpos0, so that
    # needs a change to the bridge rather than to this model.
    #
    # Practical guidance: use the default height for swing-tracking tests in free
    # space, where no ground contact is wanted anyway. For stance tests, use the
    # free-base scene.xml, which does not have this problem -- it spawns at 0.445
    # with the legs clearing the ground and settles onto them.

    header = (
        "\n  GENERATED FILE - do not edit.\n"
        "  Written by scripts/make_pinned_scene.py from go2.xml.\n\n"
        "  The base <freejoint/> is removed, so the trunk is welded to the world at\n"
        f"  z = {height:.4f}. Used to validate swing trajectories and the MPC solve\n"
        "  body dynamics in the loop. Regenerate rather than editing by hand.\n"
    )
    return ET.tostring(root, encoding="unicode").replace(
        "<mujoco", f"<!--{header}-->\n<mujoco", 1
    )


def build_pinned_scene() -> str:
    """A scene including the pinned robot, with the same lighting/ground as scene.xml.

    The obstacle boxes from scene.xml are left out: with a welded base the robot
    cannot reach them, and they only clutter the view.
    """
    return f"""<!--
  GENERATED FILE - do not edit.
  Written by scripts/make_pinned_scene.py.

  Scene for a Go2 with its base welded to the world. Select it with
  `robot_scene: "{PINNED_SCENE.name}"` in unitree_mujoco's simulate/config.yaml,
  and switch back to scene.xml for normal free-base runs.
-->
<mujoco model="go2 scene (pinned base)">
  <include file="{PINNED_XML.name}"/>

  <statistic center="0 0 0.1" extent="0.8"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height", type=float, default=DEFAULT_OBSERVED_HEIGHT,
                    help=f"body height the CONTROLLER should observe, in metres"
                         f" (default {DEFAULT_OBSERVED_HEIGHT}, its commanded standing"
                         " height). Corrected internally for the imu site offset.")
    ap.add_argument("--raw-base-height", action="store_true",
                    help="interpret --height as the base_link weld height instead,"
                         " skipping the imu site correction")
    ap.add_argument("--check", action="store_true",
                    help="load the result in MuJoCo and report the model dimensions")
    args = ap.parse_args()

    if not SRC_XML.exists():
        raise SystemExit(f"source model not found: {SRC_XML}")

    if args.raw_base_height:
        weld_z = args.height
        offset = 0.0
    else:
        offset = imu_site_offset()
        weld_z = args.height - offset

    # Straight legs at qpos0 reach this far below base_link; welding lower starts
    # the feet underground. See build_pinned_xml() for what that does.
    STRAIGHT_LEG_DROP = 0.426
    if weld_z < STRAIGHT_LEG_DROP:
        print(f"NOTE: straight legs reach {STRAIGHT_LEG_DROP:.4f} m below the base, "
              f"past this {weld_z:.4f} m weld, so a start from qpos0 would put the "
              f"feet {STRAIGHT_LEG_DROP - weld_z:.4f} m through the floor.")
        print("      This is fine as long as the simulator loads the model's home "
              "keyframe (unitree_mujoco was patched to call mj_resetDataKeyframe);")
        print("      the keyframe starts the legs in the nominal stance instead.")
        print("      If the feet ever start splayed onto the hip limits, that patch "
              "is missing.")

    PINNED_XML.write_text(build_pinned_xml(weld_z))
    PINNED_SCENE.write_text(build_pinned_scene())

    print(f"wrote {PINNED_XML}")
    print(f"wrote {PINNED_SCENE}")
    print(f"base_link welded at z = {weld_z:.4f}")
    if offset:
        print(f"imu site offset {offset:.4f} m -> controller observes z = {args.height:.4f}")

    if args.check:
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(PINNED_SCENE))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu")
        print(f"loaded OK: nq={model.nq} nv={model.nv} nu={model.nu}")
        print(f"observed body height (imu site): {data.site_xpos[sid][2]:.4f} m")
        if model.nq != 12 or model.nv != 12:
            raise SystemExit(f"expected a 12-dof model, got nq={model.nq} nv={model.nv}")
        print("12 leg DoF, no floating base -- body is frozen.")

    print()
    print("To use it, set in /opt/unitree_robotics/unitree_mujoco/simulate/config.yaml:")
    print(f'    robot_scene: "{PINNED_SCENE.name}"')
    print('and restore `robot_scene: "scene.xml"` afterwards.')


if __name__ == "__main__":
    main()
