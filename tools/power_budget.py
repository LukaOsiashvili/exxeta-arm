"""Power budget for exxeta-arm (PROJECT_SPEC section 6).

Role in the system
------------------
Second Phase 0 tool. It answers: "how much current do the servos draw, are the
battery, fuse, switch and converters big enough, and how long does a charge last?"

It reads cad/params/arm_params.json, reuses the joint torques from
tools/torque_budget.py and writes docs/design/power_budget.md.

Servo current model
-------------------
A DC motor's current is roughly proportional to the torque it produces:

    current = idle current + (torque / stall torque) x stall current

So a servo holding half its stall torque draws about half its stall current.

Run:  python tools/power_budget.py [--print]
"""
from __future__ import annotations

import argparse

from arm_params import PROJECT_ROOT, load_params
from torque_budget import compute, load_cad_masses

REPORT_PATH = PROJECT_ROOT / "docs" / "design" / "power_budget.md"
JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")


def servo_current(p: dict, joint: str, torque_fraction: float) -> float:
    """Current (A at the 6 V rail) of one servo producing a fraction of its stall torque."""
    s = p["servos"][p["joints"][joint]["servo"]]
    return s["idle_current_a"] + min(torque_fraction, 1.0) * s["stall_current_a_6v"]


def scenarios(p: dict) -> list[dict]:
    """Servo-rail current for a few typical situations."""
    tb = compute(p, cad=load_cad_masses())
    # Fraction of stall torque each joint needs in the worst-case pose.
    worst = {j: tb["joints"][j]["required_kgcm"] / tb["joints"][j]["rated_kgcm"] for j in ("J1", "J2", "J3", "J4", "J5")}
    worst["J6"] = 0.5  # gripper squeezing at half stall (our SF 2 rule)

    def rail(fracs: dict) -> float:
        return sum(servo_current(p, j, fracs.get(j, 0.0)) for j in JOINTS)

    return [
        {"name": "Parked / disabled (no PWM, servos idle)", "servo_a": 0.0,
         "why": "servos are not driven; only the ESP32 runs"},
        {"name": "Enabled, arm upright, nothing held", "servo_a": rail({}),
         "why": "almost no gravity torque; servos just hold"},
        {"name": "Typical work (average of moving, holding, pauses)", "servo_a": rail({j: 0.5 * f for j, f in worst.items()}),
         "why": "assumed: joints average half of their worst-case torque"},
        {"name": "Holding rated payload at full reach (worst static pose)", "servo_a": rail(worst),
         "why": "torques from the torque budget"},
        {"name": "All six servos stalled at once (fault case)", "servo_a": rail({j: 1.0 for j in JOINTS}),
         "why": "sizes the fuse, BMS, switch and converter peak"},
    ]


def render_markdown(p: dict) -> str:
    pw = p["power"]
    pack, srail, lrail = pw["pack"], pw["servo_rail"], pw["logic_rail"]
    n_series = 2
    n_parallel = int(pack["configuration"].upper().split("P")[0].split("S")[1])
    v_full, v_nom = n_series * pack["cell_v_full"], n_series * pack["cell_v_nominal"]
    v_warn, v_crit = n_series * pack["cell_v_warn"], n_series * pack["cell_v_critical"]
    v_bms = n_series * pack["cell_v_bms_cutoff"]
    cap_ah = n_parallel * pack["cell_capacity_mah"] / 1000.0
    energy_wh = cap_ah * v_nom
    usable_wh = energy_wh * pack["usable_fraction"]

    logic_w = lrail["voltage"] * (lrail["esp32_current_a"] + lrail["led_current_a"] + lrail["pots_current_a"]) / lrail["converter_efficiency"]
    standby_w = 0.3  # quiescent losses of the two converters + divider + BMS (estimate)

    L = []
    w = L.append
    w("# Power budget")
    w("")
    w("> **Generated file. Do not edit by hand.** Regenerate with `python tools/power_budget.py`.")
    w(f"> Source: `cad/params/arm_params.json` (design revision `{p['project']['design_revision']}`).")
    w("")
    w("## 1. Pack")
    w("")
    w(f"- {pack['configuration']} 18650, {pack['cell_capacity_mah']} mAh high-drain cells "
      f"(>= {pack['cell_max_continuous_a']} A continuous each): **{cap_ah:.1f} Ah, {energy_wh:.0f} Wh**, "
      f"about {usable_wh:.0f} Wh usable down to the critical threshold.")
    w(f"- Voltage: {v_full:.1f} V full, {v_nom:.1f} V nominal, **{v_warn:.1f} V warning**, "
      f"**{v_crit:.1f} V critical** (park and refuse motion), {v_bms:.1f} V BMS hard cut-off.")
    w("")
    w("## 2. Load scenarios")
    w("")
    w(f"Servo rail {srail['voltage']:g} V (converter efficiency {srail['converter_efficiency'] * 100:.0f} %). "
      f"Logic: ESP32 with Wi-Fi + LED = {logic_w:.1f} W. Battery current is given at the warning voltage "
      f"({v_warn:.1f} V), where it is highest.")
    w("")
    w("| Scenario | Servo rail current (A) | Battery power (W) | Battery current (A) | Runtime (h) | Basis |")
    w("|---|---|---|---|---|---|")
    sc = scenarios(p)
    for s in sc:
        batt_w = s["servo_a"] * srail["voltage"] / srail["converter_efficiency"] + logic_w + standby_w
        s["batt_w"], s["batt_a"] = batt_w, batt_w / v_warn
        w(f"| {s['name']} | {s['servo_a']:.2f} | {batt_w:.1f} | {s['batt_a']:.2f} | {usable_wh / batt_w:.1f} | {s['why']} |")
    typical, worst, stall = sc[2], sc[3], sc[4]
    w("")
    w(f"**Expected runtime: about {usable_wh / typical['batt_w']:.0f} h of typical use**, "
      f"about {usable_wh / worst['batt_w']:.1f} h if the arm held the worst-case pose the whole time.")
    w("")
    w("## 3. Component ratings")
    w("")
    w("| Component | Rating | Must carry | OK? | Comment |")
    w("|---|---|---|---|---|")

    def check(name, rating, need, comment):
        w(f"| {name} | {rating:g} A | {need:.1f} A | {'yes' if rating >= need else 'NO'} | {comment} |")

    check("Cells (per pack)", n_parallel * pack["cell_max_continuous_a"], stall["batt_a"], "all-stall fault current, shared by the parallel cells")
    check("BMS continuous", pw["bms_continuous_a"], stall["batt_a"], "must not trip before the fuse in a stall")
    check("Main rocker switch", pw["main_switch_rating_a"], pw["fuse_a"], "must survive anything the fuse lets through")
    check("Servo converter continuous", srail["converter_continuous_a"], worst["servo_a"], "worst static pose, continuous")
    check("Servo converter peak", srail["converter_peak_a"], stall["servo_a"], "all-stall, seconds only; bulk capacitors help with shorter spikes")
    fuse_ok = stall["batt_a"] * 1.25 <= pw["fuse_a"] <= pw["bms_continuous_a"]
    w(f"| Fuse | {pw['fuse_a']:g} A | {stall['batt_a']:.1f} A all-stall, {worst['batt_a']:.1f} A normal | {'yes' if fuse_ok else 'NO'} | "
      "clear of the all-stall peak and of capacitor inrush, so it only blows on a real short; below the BMS rating |")
    awg_a = {18: 16, 16: 22, 14: 32}[pw["main_wire_awg"]]  # conservative chassis-wiring ratings
    check(f"Main wiring ({pw['main_wire_awg']} AWG)", awg_a, pw["fuse_a"], "the fuse must protect the wire: wire rating above fuse rating")
    w("")
    w("## 4. Servo-rail dropout (2S pack vs 6.0 V rail)")
    w("")
    w("A buck converter can only step *down*, and it loses some voltage itself (its \"dropout\"). Near empty the pack "
      "voltage comes close to 6.0 V, so the rail can sag. Cheap modules rarely publish their dropout, so the table "
      "shows a good module (0.5 V) and a poor one (1.2 V). **The real value must be measured on the bench.**")
    w("")
    w("| Pack state | Pack voltage under load (V) | Rail, good module (V) | Rail, poor module (V) | MG996R torque, poor module |")
    w("|---|---|---|---|---|")
    for label, v in (("full", v_full), ("nominal", v_nom), ("warning", v_warn), ("critical: park", v_crit)):
        good = min(srail["voltage"], v - 0.5)
        poor = min(srail["voltage"], v - 1.2)
        # torque is roughly proportional to voltage (MG996R: 9.4 -> 11 kgf*cm from 4.8 -> 6 V)
        torque_pct = (9.4 + (11.0 - 9.4) * (poor - 4.8) / 1.2) / 11.0 * 100
        w(f"| {label} | {v:.1f} | {good:.1f} | {poor:.1f} | {torque_pct:.0f} % |")
    w("")
    th = pw["firmware_thresholds"]
    w(f"Decision (ADR-0003): keep the 6.0 V set-point and accept a sag to about {th['servo_rail_min_expected_v']:g} V near empty. "
      "All servos are rated 4.8-6 V, so the sag is harmless; it costs a few percent of torque exactly when the app is "
      "already telling the user to recharge.")
    w("")
    w("### Firmware thresholds (consistent with the sag)")
    w("")
    w("| Threshold | Pack voltage | Per cell | Worst-case rail | Action |")
    w("|---|---|---|---|---|")
    w(f"| Warning | {th['warn_v']:g} V | {th['warn_v'] / 2:.2f} V | {min(srail['voltage'], th['warn_v'] - 1.2):.1f} V | LED warning, banner in the app |")
    w(f"| Critical | {th['critical_v']:g} V | {th['critical_v'] / 2:.2f} V | {min(srail['voltage'], th['critical_v'] - 1.2):.1f} V | move to rest pose, then refuse motion |")
    w(f"| BMS cut-off | {v_bms:.1f} V | {v_bms / 2:.2f} V | off | hard cut by the BMS (arm goes limp: this is why we park first) |")
    w("")
    w(f"- The critical threshold is chosen so that even a poor converter still gives the servos "
      f"{min(srail['voltage'], th['critical_v'] - 1.2):.1f} V, comfortably above their 4.8 V minimum, with torque to finish parking.")
    w(f"- The voltage is low-pass filtered (time constant {th['filter_time_constant_s']:g} s) with {th['hysteresis_v']:g} V hysteresis, "
      "because servo current spikes pull the pack down briefly and must not trigger a park.")
    w(f"- Below {th['servo_power_absent_below_v']:g} V the firmware concludes that the main switch is off (ESP32 running from USB) and refuses to enable.")
    w("")
    w("## 5. Battery voltage sensing")
    w("")
    d = pw["battery_sense_divider"]
    ratio = d["r_bottom_ohm"] / (d["r_top_ohm"] + d["r_bottom_ohm"])
    w(f"Divider {d['r_top_ohm'] / 1000:g} kΩ / {d['r_bottom_ohm'] / 1000:g} kΩ with {d['filter_cap_nf']} nF to ground, "
      f"taken **after** the main switch (no drain when off).")
    w("")
    w(f"- At {v_full:.1f} V the ADC pin sees {v_full * ratio:.2f} V; at {v_crit:.1f} V it sees {v_crit * ratio:.2f} V. "
      "Both lie in the linear part of the ESP32 ADC (about 0.15-2.45 V at 11 dB attenuation).")
    w(f"- Divider current: {v_full / (d['r_top_ohm'] + d['r_bottom_ohm']) * 1e6:.0f} µA.")
    w("- Reading near 0 V while USB is connected means the main switch is off: the firmware then refuses to enable the servos.")
    w("")
    w("## 6. Charging, and charging while running")
    w("")
    c = pw["charger"]
    net = c["cc_current_a"] - typical["batt_a"]
    w(f"- Adapter: {c['adapter']}. Internal CC/CV module set to **{c['cv_voltage']:g} V / {c['cc_current_a']:g} A** "
      f"({c['cv_voltage'] / 2:.2f} V per cell).")
    w(f"- Charge time from the critical threshold: about {cap_ah * pack['usable_fraction'] / c['cc_current_a'] * 1.3:.1f} h "
      "(the 1.3 factor covers the slow constant-voltage tail).")
    w(f"- Charge current per cell: {c['cc_current_a'] / n_parallel:.2f} A (gentle; typical cells allow 1-2 A or more).")
    w("")
    w("### What happens when the arm runs while the charger is connected")
    w("")
    w("The load always runs from the pack; the charger feeds the same node. Checked behaviour:")
    w("")
    w("1. **The CC/CV module has no end-of-charge detection at all.** It is a voltage-limited, current-limited supply. "
      "With or without a load it simply holds its set voltage for as long as the adapter is plugged in. So the concern is real: "
      "the pack *will* be held at the set voltage indefinitely. A running arm does not make this worse; it only means the "
      "module's 'charging' LED (which just shows current flowing) never goes out.")
    w(f"2. **It cannot overcharge.** The pack can never exceed the set voltage ({c['cv_voltage']:g} V), "
      "and the BMS independently cuts off any single cell above about 4.25 V.")
    w(f"3. **Why {c['cv_voltage']:g} V and not 8.4 V.** What ages Li-ion cells is sitting for long periods at 4.2 V. "
      f"Holding {c['cv_voltage'] / 2:.2f} V per cell is the usual stand-by (float) level for packs that stay on a supply; "
      "it costs roughly 10 % of capacity and turns 'left it plugged in over the weekend' from slow damage into a non-event.")
    w(f"4. **Net current while working.** Typical use draws {typical['batt_a']:.1f} A and the charger supplies {c['cc_current_a']:g} A, "
      f"so during work the pack {'still fills slowly' if net > 0 else 'drains slowly'} ({net:+.1f} A net) and fills whenever the arm pauses.")
    w(f"5. **Firmware help.** When the filtered pack voltage is above {th['charger_present_above_v']:g} V the firmware reports "
      "'charger connected / full', and the app shows a reminder to unplug.")
    w("6. **Rule for the team:** unplug the adapter when the app says full, and never charge unattended (safety page).")
    w("")
    w("### Why not a power-path circuit")
    w("")
    w("A power path would feed the arm from the adapter and charge the pack separately. Rejected for v1: "
      f"(a) the servos can pull {stall['servo_a']:.0f} A peaks at 6 V, far more than a 12 V / 2 A adapter delivers, so the pack "
      "has to stay in the circuit anyway; (b) the usual diode-OR costs 0.4-0.5 V in the battery line, which is exactly the "
      "headroom the 6.0 V servo rail lacks near empty (section 4); (c) more parts to wire wrongly.")
    w("")
    w("### Two side effects to know")
    w("")
    w(f"- **Balancing.** Typical 2S balance boards only bleed a cell once it reaches 4.20 V, so at {c['cv_voltage'] / 2:.2f} V per cell they "
      "stay inactive. Use four cells of the same type, age and batch, and check both cell voltages with a multimeter every few months "
      "(they should agree within 0.05 V). The ADC pin freed by dropping the J5 pot can optionally monitor the pack mid-point.")
    w("- **Optional USB-C charger (IP2326 type).** That module *does* terminate on low current, and a running arm confuses it. "
      "If it is fitted: charge from USB-C only with the arm parked and disabled.")
    w("")
    return "\n".join(L)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--print", action="store_true", help="also print the report")
    args = ap.parse_args()
    md = render_markdown(load_params())
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(md, encoding="utf-8")
    if args.print:
        print(md)
    print(f"wrote {REPORT_PATH.relative_to(PROJECT_ROOT)}")


if __name__ == "__main__":
    main()
