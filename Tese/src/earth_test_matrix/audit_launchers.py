"""
PHASE 0 - LAUNCHER CORRECTNESS AUDIT (read-only)

Verifies that each Earth launcher in the VEHICLES registry is physically
self-consistent BEFORE any trajectory test is run. For every vehicle it computes
from the registry values:

  - liftoff thrust-to-weight (sea level)        -> must be > 1 to lift off
  - per-stage structural fraction EPSILON       -> realistic ~0.04 - 0.12
  - ideal staged delta-v via Tsiolkovsky        -> compare to LEO need (~9.4 km/s
                                                    incl. losses; v_circ ~7.8 km/s)
  - per-stage mass ratio m0/m1

and prints them next to hand-collected published figures so spec errors (wrong
magnitude, broken mass ratio, unit mismatch) are obvious.

Run with the pygmo-env interpreter:
  C:\\Users\\eduar\\miniforge3\\envs\\pygmo-env\\python.exe Tese/src/earth_test_matrix/audit_launchers.py
"""

import sys
import math
from pathlib import Path

# Tese/src is two levels up from this file (earth_test_matrix/ -> src/).
SRC = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SRC))

from Auxiliary import rocket_specs as r
from Auxiliary import constants as c

G0 = c.G_0  # 9.81, Earth-standard, used for all Isp definitions

# Earth reference for orbital-velocity context (target ~200 km LEO).
R_E = c.PLANETS["earth"]["R"]
MU_E = c.PLANETS["earth"]["MU"]
TARGET_ALT = 200e3
V_CIRC = math.sqrt(MU_E / (R_E + TARGET_ALT))

# Published / reference figures for sanity comparison (approximate, public data).
REFERENCE = {
    "falcon9": {
        "note": "Falcon 9 Block 5, 9x Merlin 1D + 1x MVac",
        "twr_liftoff": "~1.4",
        "stage1_dv_kms": "~3.9 (with payload)",
        "stage2_dv_kms": "~6+ (depends on payload)",
        "leo_payload_kg": "~17000-22000 (real)",
    },
    "electron": {
        "note": "Rocket Lab Electron, 9x Rutherford + 1x Rutherford Vac",
        "twr_liftoff": "~1.3-1.4",
        "stage1_dv_kms": "~4.0",
        "stage2_dv_kms": "~5.5-6.0",
        "leo_payload_kg": "~200-300 (real)",
    },
    "saturn_v": {
        "note": "EXCLUDED from tests - simulator has only 2 serial stages; real "
                "Saturn V needs 3 (S-IC/S-II/S-IVB). Shown as removal evidence.",
        "twr_liftoff": "~1.17",
        "stage1_dv_kms": "~3.7 (S-IC)",
        "stage2_dv_kms": "~5.0 (S-II, real insertion is S-IVB)",
        "leo_payload_kg": "~140000 (real, to LEO, 3-stage)",
    },
}

TESTED = ["falcon9", "electron"]
EVIDENCE_ONLY = ["saturn_v"]


def tsiolkovsky(isp, m0, m1):
    if m1 <= 0 or m0 <= 0 or m0 <= m1:
        return 0.0
    return isp * G0 * math.log(m0 / m1)


def audit_vehicle(name):
    r.set_vehicle(name)

    single = (r.NUM_STAGES == 1)

    # Stage 1 masses (M_TOTAL_1 already includes stage-2 + payload).
    m0_1 = r.M_TOTAL_1
    m1_1 = r.M_TOTAL_1 - r.M_PROP_1
    twr = r.F_THRUST_1_SL / (m0_1 * G0) if m0_1 else 0.0
    dv1_sl = tsiolkovsky(r.ISP_1_SL, m0_1, m1_1)
    dv1_vac = tsiolkovsky(r.ISP_1_VAC, m0_1, m1_1)

    # Stage 2 masses.
    if single:
        m0_2 = m1_2 = 0.0
        dv2 = 0.0
    else:
        m0_2 = r.M_TOTAL_2
        m1_2 = r.M_TOTAL_2 - r.M_PROP_2
        dv2 = tsiolkovsky(r.ISP_2, m0_2, m1_2)

    dv_total = dv1_vac + dv2  # use vacuum stage-1 Isp for the ideal budget

    print(f"\n{'='*72}")
    print(f"  {name}   (NUM_STAGES={r.NUM_STAGES}, BODY={r.BODY})")
    print(f"{'='*72}")
    ref = REFERENCE.get(name, {})
    print(f"  note: {ref.get('note','')}")
    print(f"  liftoff mass (GLOW)        : {m0_1:>12,.0f} kg")
    print(f"  M_PAYLOAD                  : {r.M_PAYLOAD:>12,.0f} kg")
    print(f"  liftoff T/W (sea level)    : {twr:>12.3f}   (published {ref.get('twr_liftoff','?')})")
    print(f"  Stage 1  EPSILON (struct.) : {r.EPSILON_1:>12.4f}")
    print(f"  Stage 1  mass ratio m0/m1  : {(m0_1/m1_1 if m1_1>0 else float('nan')):>12.3f}")
    print(f"  Stage 1  dv  (SL Isp)      : {dv1_sl:>12.0f} m/s")
    print(f"  Stage 1  dv  (VAC Isp)     : {dv1_vac:>12.0f} m/s   (published {ref.get('stage1_dv_kms','?')} km/s)")
    if not single:
        print(f"  Stage 2  EPSILON (struct.) : {r.EPSILON_2:>12.4f}")
        print(f"  Stage 2  mass ratio m0/m1  : {(m0_2/m1_2 if m1_2>0 else float('nan')):>12.3f}")
        print(f"  Stage 2  dv  (VAC Isp)     : {dv2:>12.0f} m/s   (published {ref.get('stage2_dv_kms','?')} km/s)")
    print(f"  {'-'*68}")
    print(f"  IDEAL TOTAL dv             : {dv_total:>12.0f} m/s  = {dv_total/1000:.2f} km/s")
    print(f"  v_circ @ {TARGET_ALT/1000:.0f} km           : {V_CIRC:>12.0f} m/s  = {V_CIRC/1000:.2f} km/s")
    print(f"  LEO need incl. losses      : {'~9400':>12} m/s  (gravity+drag ~1.5-2.0 km/s)")

    # --- flags ---
    flags = []
    if twr <= 1.0:
        flags.append("CANNOT LIFT OFF (T/W <= 1)")
    if not (0.02 <= r.EPSILON_1 <= 0.20):
        flags.append(f"EPSILON_1={r.EPSILON_1:.3f} out of realistic band")
    margin = dv_total - 9400.0
    if margin > 4000:
        flags.append(f"OVERPERFORMS by ~{margin:.0f} m/s vs LEO need "
                     "(guidance may never activate; Stage 1 alone reaches orbit)")
    elif margin < 0:
        flags.append(f"UNDERPERFORMS by ~{-margin:.0f} m/s vs LEO need "
                     "(likely suborbital / POOR results expected)")
    if r.M_PAYLOAD == 0 and name == "falcon9":
        flags.append("M_PAYLOAD=0 (unrealistic; inflates upper-stage dv)")

    print(f"  {'-'*68}")
    if flags:
        for f in flags:
            print(f"  [FLAG] {f}")
    else:
        print(f"  [OK] no physical-consistency flags")

    return {
        "name": name, "twr": twr, "epsilon_1": r.EPSILON_1,
        "dv_total": dv_total, "flags": flags,
    }


def main():
    print("\n" + "#"*72)
    print("#  PHASE 0  -  EARTH LAUNCHER CORRECTNESS AUDIT")
    print("#"*72)
    print(f"  G_0 = {G0} m/s^2   |   v_circ(200km) = {V_CIRC:.0f} m/s")

    print("\n\n>>> TESTED LAUNCHERS")
    for name in TESTED:
        audit_vehicle(name)

    print("\n\n>>> EXCLUDED (evidence only - removal candidate)")
    for name in EVIDENCE_ONLY:
        audit_vehicle(name)

    print("\n" + "#"*72)
    print("#  AUDIT COMPLETE")
    print("#"*72 + "\n")

    # restore default vehicle so importing this module leaves a clean state
    r.set_vehicle("falcon9")


if __name__ == "__main__":
    main()
