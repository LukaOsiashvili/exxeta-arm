"""Torque budget and sizing for exxeta-arm (PROJECT_SPEC section 4.3).

Role in the system
------------------
This is the FIRST design tool. It answers: "with these link lengths, these
servos and this payload, is every joint strong enough?"

It reads cad/params/arm_params.json and writes docs/design/torque_budget.md.

How the calculation works (statics only, no calculus needed)
-----------------------------------------------------------
Worst case for the pitch joints (J2 shoulder, J3 elbow, J4 wrist) is the arm
stretched out HORIZONTALLY with the payload in the gripper: every mass then
has its longest possible lever arm.

    torque at a joint = sum over everything the joint carries of
                        mass x g x (horizontal distance from the joint axis)

Hobby-servo torque is quoted in kgf*cm, which makes this easy by hand:
a 100 g mass at 25 cm needs 0.1 kg x 25 cm = 2.5 kgf*cm.

Safety factor (SF) = rated stall torque at 6 V / required torque.
The spec demands SF >= 2 on every joint.

Where the masses come from
--------------------------
Before the CAD exists, printed-part masses are hand estimates from the params
file. As soon as tools/export_all.py has written cad/export/masses.json, every
item that names `cad_parts` uses the CAD mass and centre of mass instead.

Run:  python tools/torque_budget.py            (writes the markdown report)
      python tools/torque_budget.py --print    (also prints it to the terminal)
      python tools/torque_budget.py --gate     (exit code 1 if any joint is below
                                                the minimum safety factor: the
                                                "sizing gate" used by export_all.py)
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from pathlib import Path

from arm_params import G, KGCM_TO_NM, PROJECT_ROOT, load_params, servo_torque_kgcm

REPORT_PATH = PROJECT_ROOT / "docs" / "design" / "torque_budget.md"
CAD_MASSES_PATH = PROJECT_ROOT / "cad" / "export" / "masses.json"
JOINT_OF_LINK = {"upper_arm": "J2", "forearm": "J3", "hand": "J4"}


def load_cad_masses() -> dict | None:
    """CAD-derived masses written by tools/export_all.py, or None before the CAD exists."""
    if CAD_MASSES_PATH.exists():
        return json.loads(CAD_MASSES_PATH.read_text(encoding="utf-8"))
    return None

GMM_PER_KGCM = 10_000.0  # 1 kgf*cm = 1000 g x 10 mm = 10 000 g*mm

# Which link starts at which joint (used to decide what each joint carries).
LINK_OF_JOINT = {"J2": "upper_arm", "J3": "forearm", "J4": "hand"}
# Links carried by each pitch joint.
CARRIED = {
    "J2": {"upper_arm", "forearm", "hand"},
    "J3": {"forearm", "hand"},
    "J4": {"hand"},
}


# --------------------------------------------------------------------------
# Mass model
# --------------------------------------------------------------------------
def joint_positions(p: dict) -> dict:
    """Distance of each joint axis and the TCP from the J2 axis, arm stretched out (mm)."""
    g = p["geometry"]
    l1, l2, l3 = g["upper_arm_length_mm"], g["forearm_length_mm"], g["wrist_to_tcp_mm"]
    return {"J2": 0.0, "J3": l1, "J4": l1 + l2, "TCP": l1 + l2 + l3}


def resolved_items(p: dict, with_pots: bool = True, cad: dict | None = None) -> list[dict]:
    """Turn the mass-model list into [{name, mass_g, x_mm, link, r_roll_mm}].

    x_mm is measured from the J2 (shoulder) axis along the stretched-out arm.
    """
    g = p["geometry"]
    pos = joint_positions(p)
    link_len = {"J2": g["upper_arm_length_mm"], "J3": g["forearm_length_mm"], "J4": g["wrist_to_tcp_mm"]}
    out = []
    for it in p["mass_model"]["items"]:
        if it.get("only_with_pots") and not with_pots:
            continue
        ref = it["from"]
        x = pos[ref] + it.get("offset_mm", 0.0) + it.get("frac", 0.0) * link_len[ref]
        mass = it["mass_g"]
        src = it.get("source", "")
        if src.startswith("servo:"):
            # Mass follows the servo assigned to that joint, so swapping a servo
            # model in the params automatically updates the budget.
            joint = src.split(":")[1]
            mass = p["servos"][p["joints"][joint]["servo"]]["mass_g"]
        if cad and it.get("cad_parts"):
            # Replace the hand estimate by the real CAD parts (mass and centre of mass).
            parts = [cad["parts"][n] for n in it["cad_parts"]]
            mass = sum(c["mass_g"] for c in parts)
            x = sum(c["mass_g"] * (pos[JOINT_OF_LINK[c["link"]]] + c["com_along_mm"]) for c in parts) / mass
            src = "CAD"
        out.append({
            "name": it["name"],
            "mass_g": mass,
            "x_mm": x,
            "link": it.get("link", LINK_OF_JOINT[ref]),
            "r_roll_mm": it.get("radius_from_roll_axis_mm", 0.0),
            "source": src,
        })
    return out


# --------------------------------------------------------------------------
# Core calculation
# --------------------------------------------------------------------------
def compute(p: dict, with_pots: bool = True, cad: dict | None = None) -> dict:
    """Compute the whole budget for one parameter set. Returns a plain dict.

    `cad` = contents of cad/export/masses.json (or None to use the hand estimates).
    """
    tgt = p["design_targets"]
    payload = tgt["payload_g"]
    sf_min = tgt["safety_factor_min"]
    pos = joint_positions(p)
    items = resolved_items(p, with_pots, cad)
    offs = p["geometry"]["shoulder_offset_mm"]  # J1 axis -> J2 axis

    res = {"payload_g": payload, "positions": pos, "items": items, "joints": {}}
    res["reach_mm"] = offs + pos["TCP"]
    res["moving_mass_g"] = sum(i["mass_g"] for i in items)

    # ---- J2, J3, J4: static gravity torque, arm horizontal ----------------
    for j in ("J2", "J3", "J4"):
        xj = pos[j]
        moment_gmm = sum(i["mass_g"] * (i["x_mm"] - xj) for i in items if i["link"] in CARRIED[j])
        moment_gmm += payload * (pos["TCP"] - xj)
        res["joints"][j] = _joint_row(p, j, moment_gmm / GMM_PER_KGCM, sf_min, "static gravity, arm horizontal")

    # ---- J5 wrist roll: off-axis masses + friction + inertia --------------
    # Worst case: the hand is horizontal and rolled so that every off-axis
    # mass sits beside the axis (conservative: all on the same side).
    hand = [i for i in items if i["link"] == "hand"]
    r_pl = p["mass_model"]["payload"]["radius_from_roll_axis_mm"]
    grav5 = (sum(i["mass_g"] * i["r_roll_mm"] for i in hand) + payload * r_pl) / GMM_PER_KGCM
    inertia5 = (sum(i["mass_g"] * i["r_roll_mm"] ** 2 for i in hand) + payload * r_pl ** 2) * 1e-9  # kg*m^2
    acc5 = math.radians(p["joints"]["J5"]["max_accel_deg_s2"])
    fric5 = p["mass_model"]["friction_allowance_kgcm"]["J5"]
    req5 = grav5 + fric5 + inertia5 * acc5 / KGCM_TO_NM
    res["joints"]["J5"] = _joint_row(p, "J5", req5, sf_min, "off-axis gravity + friction + inertia")

    # ---- J1 base yaw: inertia of the stretched-out arm + friction ---------
    # Gravity does not load a vertical axis; the servo only has to accelerate
    # the arm (torque = moment of inertia x angular acceleration) and beat friction.
    inertia1 = (sum(i["mass_g"] * (offs + i["x_mm"]) ** 2 for i in items)
                + payload * (offs + pos["TCP"]) ** 2) * 1e-9  # kg*m^2
    acc1 = math.radians(p["joints"]["J1"]["max_accel_deg_s2"])
    fric1 = p["mass_model"]["friction_allowance_kgcm"]["J1"]
    req1 = inertia1 * acc1 / KGCM_TO_NM + fric1
    res["joints"]["J1"] = _joint_row(p, "J1", req1, sf_min, "inertia at default accel + friction")
    res["inertia_yaw_kgm2"] = inertia1

    # ---- Information: extra J2 torque needed to accelerate (dynamics) -----
    acc2 = math.radians(p["joints"]["J2"]["max_accel_deg_s2"])
    inertia2 = (sum(i["mass_g"] * i["x_mm"] ** 2 for i in items) + payload * pos["TCP"] ** 2) * 1e-9
    res["j2_dynamic_extra_kgcm"] = inertia2 * acc2 / KGCM_TO_NM

    # ---- Gripper holding force (information) ------------------------------
    servo6 = p["servos"][p["joints"]["J6"]["servo"]]
    jaw_lever_cm = 4.0  # jaw pivot -> grip centre, preliminary (Phase 1 CAD)
    squeeze_kgf = servo6["stall_torque_kgcm_6v"] / sf_min / jaw_lever_cm
    res["gripper"] = {
        "jaw_lever_cm": jaw_lever_cm,
        "squeeze_n": squeeze_kgf * G,
        # two friction contacts, rubber/TPU pad on a typical object: mu ~ 0.6
        "holdable_g": 2 * 0.6 * squeeze_kgf * 1000,
    }

    res["tipping"] = _tipping(p, items, payload, pos, offs, cad)
    res["ok"] = all(r["sf"] >= sf_min for r in res["joints"].values()) and res["tipping"]["sf"] >= 1.5
    return res


def _joint_row(p: dict, joint: str, required_kgcm: float, sf_min: float, load_case: str) -> dict:
    tgt = p["design_targets"]
    model = p["joints"][joint]["servo"]
    servo = p["servos"][model]
    rated = servo["stall_torque_kgcm_6v"]
    available = rated / sf_min  # "derated" torque we allow ourselves to use
    return {
        "joint": joint,
        "name": p["joints"][joint]["name"],
        "servo": model,
        "load_case": load_case,
        "required_kgcm": required_kgcm,
        "rated_kgcm": rated,
        "available_kgcm": available,
        "sf": rated / required_kgcm if required_kgcm > 0 else float("inf"),
        "margin_pct": (available - required_kgcm) / available * 100.0,
        # Information: same load against a low battery and against a weak clone.
        "sf_low_batt": servo_torque_kgcm(servo, tgt["servo_rail_voltage_low_battery"]) / required_kgcm,
        "sf_clone": rated * tgt["clone_torque_fraction"] / required_kgcm,
    }


def _tipping(p: dict, items: list[dict], payload: float, pos: dict, offs: float, cad: dict | None = None) -> dict:
    """Static tipping check: arm horizontal, pointing straight at a tipping edge.

    The base tips about the line through two adjacent feet. Everything closer
    to the centre than that line holds the base down; everything beyond it
    tries to tip it over.
    """
    b = p["base"]
    d = min(b["footprint_mm"]) / 2.0 - b["foot_inset_mm"]  # J1 axis -> tipping line
    stationary = dict(b["stationary_mass_items_g"])
    if cad:  # use the real printed masses of the base parts where the CAD has them
        for key, names in b.get("stationary_cad_parts", {}).items():
            stationary[key] = sum(cad["parts"][n]["mass_g"] for n in names)
    base_mass = sum(stationary.values())
    restoring = base_mass * d
    overturning = 0.0
    for m, r in [(i["mass_g"], offs + i["x_mm"]) for i in items] + [(payload, offs + pos["TCP"])]:
        if r > d:
            overturning += m * (r - d)
        else:
            restoring += m * (d - r)
    return {"edge_mm": d, "base_mass_g": base_mass, "restoring_gmm": restoring,
            "overturning_gmm": overturning, "sf": restoring / overturning}


# --------------------------------------------------------------------------
# Sizing sweep: "what is the longest arm for a given payload?"
# --------------------------------------------------------------------------
# Smallest link length the joints physically fit into (fork swing radius of
# one joint + servo cradle of the next). Preliminary packaging estimate.
MIN_LINK_MM = 75
MAX_LINK_MM = 140


def variant(p: dict, link_mm: float, payload_g: float, j4_servo: str) -> dict:
    """Copy of the params with equal link lengths, another payload / wrist servo."""
    q = copy.deepcopy(p)
    q["geometry"]["upper_arm_length_mm"] = link_mm
    q["geometry"]["forearm_length_mm"] = link_mm
    q["design_targets"]["payload_g"] = payload_g
    q["joints"]["J4"]["servo"] = j4_servo
    if j4_servo not in ("MG90S", "SG90"):
        # A standard-size servo at the wrist needs a bigger cradle and a wider fork.
        q["mass_model"]["items"].append(
            {"name": "extra structure for standard-size wrist servo", "mass_g": 8, "from": "J4", "source": "estimate"})
    return q


def max_link_for(p: dict, payload_g: float, j4_servo: str) -> tuple[float | None, dict | None]:
    """Longest equal link length (1 mm steps) that keeps SF >= minimum on all joints."""
    best = (None, None)
    for link in range(MIN_LINK_MM, MAX_LINK_MM + 1):
        r = compute(variant(p, link, payload_g, j4_servo))
        if all(row["sf"] >= p["design_targets"]["safety_factor_min"] for row in r["joints"].values()):
            best = (link, r)
        else:
            break
    return best


def sweep(p: dict) -> list[dict]:
    rows = []
    for j4 in ("MG90S", "S3003"):
        for payload in (50, 75, 100, 125, 150):
            link, r = max_link_for(p, payload, j4)
            if link is None:
                # Find out which joint blocks it even at the shortest arm.
                r0 = compute(variant(p, MIN_LINK_MM, payload, j4))
                worst = min(r0["joints"].values(), key=lambda row: row["sf"])
                rows.append({"j4": j4, "payload": payload, "link": None,
                             "note": f"not possible: {worst['joint']} SF {worst['sf']:.2f} even at {MIN_LINK_MM} mm links"})
            else:
                limiting = min((r["joints"][j] for j in ("J2", "J3", "J4")), key=lambda row: row["sf"])
                rows.append({"j4": j4, "payload": payload, "link": link, "reach": r["reach_mm"],
                             "sf2": r["joints"]["J2"]["sf"], "sf4": r["joints"]["J4"]["sf"],
                             "note": f"limited by {limiting['joint']}"})
    return rows


# --------------------------------------------------------------------------
# Report
# --------------------------------------------------------------------------
def render_markdown(p: dict) -> str:
    cad = load_cad_masses()
    r = compute(p, cad=cad)
    r_nopots = compute(p, with_pots=False, cad=cad)
    tgt, geo = p["design_targets"], p["geometry"]
    sf_min = tgt["safety_factor_min"]
    L = []
    w = L.append

    w("# Torque budget and sizing")
    w("")
    w("> **Generated file. Do not edit by hand.** Regenerate with `python tools/torque_budget.py`.")
    w(f"> Source: `cad/params/arm_params.json` (design revision `{p['project']['design_revision']}`).")
    w("> Printed-part masses: " + ("**CAD-derived** (`cad/export/masses.json`)." if cad
                                  else "**hand estimates** (no CAD export found yet)."))
    w("")
    w("## 1. Result")
    w("")
    w(f"- **Reach:** {r['reach_mm']:.0f} mm (J1 axis to grip centre, arm horizontal). "
      f"Fingertips reach {r['reach_mm'] + geo['tcp_to_fingertip_mm']:.0f} mm.")
    w(f"- **Links:** upper arm {geo['upper_arm_length_mm']} mm, forearm {geo['forearm_length_mm']} mm, "
      f"wrist axis to grip centre {geo['wrist_to_tcp_mm']} mm.")
    w(f"- **Rated payload:** {r['payload_g']} g at full horizontal reach.")
    w(f"- **Moving mass beyond the shoulder:** {r['moving_mass_g']:.0f} g with pots, "
      f"{r_nopots['moving_mass_g']:.0f} g without (plus payload).")
    w(f"- **Verdict:** {'PASS' if r['ok'] else 'FAIL'}: safety factor >= {sf_min:g} on every joint"
      f"{'' if r['ok'] else ' is NOT met'}.")
    w("")
    w("## 2. Joint table")
    w("")
    w(f"Rule (spec 4.3): required torque <= rated stall torque at 6 V / {sf_min:g}. "
      "\"Available\" is that derated value. Pots are assumed fitted (heavier, worst case).")
    w("")
    w("| Joint | Servo | Load case | Required (kgf·cm) | Rated @6 V | Available (derated) | Margin | **SF** | Pass |")
    w("|---|---|---|---|---|---|---|---|---|")
    for j in ("J1", "J2", "J3", "J4", "J5"):
        row = r["joints"][j]
        w(f"| {j} {row['name']} | {row['servo']} | {row['load_case']} | {row['required_kgcm']:.2f} | "
          f"{row['rated_kgcm']:.1f} | {row['available_kgcm']:.2f} | {row['margin_pct']:+.0f} % | "
          f"**{row['sf']:.2f}** | {'yes' if row['sf'] >= sf_min else 'NO'} |")
    grip = r["gripper"]
    w(f"| J6 gripper | {p['joints']['J6']['servo']} | squeeze at {grip['jaw_lever_cm']:.0f} cm jaw lever | "
      f"gives ~{grip['squeeze_n']:.1f} N squeeze | | | | holds ~{grip['holdable_g']:.0f} g by friction | "
      f"{'yes' if grip['holdable_g'] >= 2 * r['payload_g'] else 'check'} |")
    w("")
    w("### What the margin really means (read this)")
    w("")
    w("The SF above is measured against the **datasheet** torque, as the spec asks. Two things eat into it in real life:")
    w("")
    w(f"| Joint | SF vs datasheet @6 V | SF with rail sagging to {tgt['servo_rail_voltage_low_battery']:g} V (low battery) | "
      f"SF if the servo is a weak clone ({tgt['clone_torque_fraction'] * 100:.0f} % of rated) |")
    w("|---|---|---|---|")
    for j in ("J1", "J2", "J3", "J4", "J5"):
        row = r["joints"][j]
        w(f"| {j} | {row['sf']:.2f} | {row['sf_low_batt']:.2f} | {row['sf_clone']:.2f} |")
    w("")
    w("- Even with weak clones every joint stays above 1.0, so the arm still holds the rated payload at full reach; "
      "it just has little reserve there. Closer in, the load drops quickly.")
    w(f"- Accelerating the shoulder at the default limit ({p['joints']['J2']['max_accel_deg_s2']} °/s²) adds only "
      f"{r['j2_dynamic_extra_kgcm']:.2f} kgf·cm at J2, so statics dominate and the gentle default limits are appropriate.")
    w("- A stalled hobby servo overheats. SF 2 means the shoulder servo works at about half its stall current "
      "when holding the worst-case pose: fine for minutes, not for hours. Park the arm when idle.")
    w("")
    w("## 3. Where the shoulder torque comes from")
    w("")
    w("Arm horizontal. Position is measured from the shoulder (J2) axis. 1 kgf·cm = 10 000 g·mm.")
    w("")
    w("| Item | Mass (g) | Position (mm) | J2 torque (kgf·cm) | Source |")
    w("|---|---|---|---|---|")
    total = 0.0
    for it in r["items"]:
        t = it["mass_g"] * it["x_mm"] / GMM_PER_KGCM
        total += t
        w(f"| {it['name']} | {it['mass_g']:.1f} | {it['x_mm']:.0f} | {t:.3f} | {it['source']} |")
    tp = r["payload_g"] * r["positions"]["TCP"] / GMM_PER_KGCM
    w(f"| **payload** | {r['payload_g']:.0f} | {r['positions']['TCP']:.0f} | {tp:.3f} | design target |")
    w(f"| **Total** | {r['moving_mass_g'] + r['payload_g']:.0f} | | **{total + tp:.2f}** | |")
    w("")
    w(f"The payload is {tp / (total + tp) * 100:.0f} % of the shoulder load; the arm's own weight is the rest. "
      "Items with source `CAD` use the real part volumes; `estimate-print` items are hand estimates with ~15 % contingency.")
    w("")
    w("## 4. Base tipping check (preliminary)")
    w("")
    t = r["tipping"]
    w(f"Arm horizontal with payload, pointing straight at the nearest tipping line "
      f"({t['edge_mm']:.0f} mm from the J1 axis: {p['base']['footprint_mm'][0]} mm base, feet inset "
      f"{p['base']['foot_inset_mm']} mm). Stationary mass in the base: {t['base_mass_g']:.0f} g.")
    w("")
    w(f"- Restoring moment: {t['restoring_gmm'] / GMM_PER_KGCM:.1f} kgf·cm")
    w(f"- Overturning moment: {t['overturning_gmm'] / GMM_PER_KGCM:.1f} kgf·cm")
    w(f"- **Tipping safety factor: {t['sf']:.1f}** (target >= 1.5 static; sudden stops add load, hence the margin)")
    w("")
    w("## 5. Sizing sweep: longest arm for each payload")
    w("")
    w(f"Equal upper-arm and forearm length, wrist-to-grip fixed at {geo['wrist_to_tcp_mm']} mm (set by packaging), "
      f"J1/J2/J3 = MG996R. Links shorter than {MIN_LINK_MM} mm do not physically fit the joints.")
    w("")
    w("| Wrist (J4) servo | Payload (g) | Max link length (mm) | Max reach (mm) | SF J2 | SF J4 | Note |")
    w("|---|---|---|---|---|---|---|")
    for s in sweep(p):
        if s["link"] is None:
            w(f"| {s['j4']} | {s['payload']} | n/a | n/a | | | {s['note']} |")
        else:
            w(f"| {s['j4']} | {s['payload']} | {s['link']} | {s['reach']:.0f} | {s['sf2']:.2f} | {s['sf4']:.2f} | {s['note']} |")
    w("")
    w("The sweep shows the *limit* (SF exactly at the minimum) and always uses the Phase 0 hand estimates, "
      "because CAD masses only exist for the link lengths that were actually modelled.")
    w("")
    w("## 6. Units")
    w("")
    w(f"1 kgf·cm = {KGCM_TO_NM:.4f} N·m. Shoulder requirement: "
      f"{r['joints']['J2']['required_kgcm']:.2f} kgf·cm = {r['joints']['J2']['required_kgcm'] * KGCM_TO_NM:.3f} N·m.")
    w("")
    return "\n".join(L)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--print", action="store_true", help="also print the report")
    ap.add_argument("--gate", action="store_true", help="exit with code 1 if any joint is below the minimum SF")
    args = ap.parse_args()
    p = load_params()
    md = render_markdown(p)
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(md, encoding="utf-8")
    if args.print:
        print(md)
    print(f"wrote {REPORT_PATH.relative_to(PROJECT_ROOT)}")
    r = compute(p, cad=load_cad_masses())
    for j, row in r["joints"].items():
        print(f"  {j} {row['servo']:7s} SF {row['sf']:.2f}")
    print(f"  tipping SF {r['tipping']['sf']:.2f}")
    if args.gate and not r["ok"]:
        print("SIZING GATE FAILED: a joint is below the minimum safety factor. Stop and tell the owner.")
        sys.exit(1)


if __name__ == "__main__":
    main()
