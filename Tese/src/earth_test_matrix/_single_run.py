"""
CHILD HARNESS - runs exactly ONE (vehicle, guidance, coast, rotation) combo in a
fresh interpreter and prints a single machine-readable result line.

A fresh process per combo is mandatory: the PSO solver modules snapshot stage
constants at import time, so switching vehicles in one process uses stale values;
a fresh process also isolates hard crashes (segfaults) from the rest of the sweep.

Usage (driven by run_matrix.py):
  python _single_run.py '{"vehicle":"electron","guidance":"gravity_turn",
                          "coast":"apogee_check","rotation":false,"budget":{...}}'

Output (last line on real stdout):
  RESULT_JSON {"vehicle":..., "status":"PASS|POOR|CRASH|REJECTED|ERROR", ...}

Status semantics:
  PASS     - completed and met the method's success criterion
  POOR     - completed but orbit off-target / suborbital / guidance never activated
  CRASH    - ground impact (ra.CRASH_DETECTED) or PSO 'crashed' flag
  REJECTED - a config-guard ValueError fired (expected for incompatible combos)
  ERROR    - any other unexpected exception (a real bug, e.g. brentq sign error)
"""

import sys
import os
import json
import time
import io
import contextlib
from pathlib import Path

# matplotlib backend MUST be set before anything imports pyplot.
import matplotlib
matplotlib.use("Agg")

SRC = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SRC))

# ---- classification tolerances (easy to tune) ----
APO_TOL_FRAC = 0.15      # apogee_check: apoapsis within +/-15% of target altitude
CIRC_TOL_FRAC = 0.25     # direct/pso/indirect: MEAN altitude within +/-25% of target
ECC_TOL = 0.10           # near-circular eccentricity ceiling

# Config-guard ValueError messages -> REJECTED (expected), NOT a bug.
GUARD_MARKERS = [
    "not supported",
    "Unsupported GUIDANCE_MODE",
    "is designed for body",
    "Unknown VEHICLE",
    "Unknown PLANET",
]


def main():
    # Spec via COMBO_SPEC env var (preferred, avoids shell-quoting issues) or argv[1].
    raw = os.environ.get("COMBO_SPEC") or (sys.argv[1] if len(sys.argv) > 1 else None)
    if not raw:
        sys.stderr.write("no COMBO_SPEC env var and no argv spec\n")
        sys.exit(2)
    spec = json.loads(raw)
    vehicle = spec["vehicle"]
    planet = spec.get("planet", "earth")
    guidance = spec["guidance"]
    coast = spec["coast"]
    rotation = bool(spec.get("rotation", False))
    budget = spec.get("budget", {})

    result = {
        "vehicle": vehicle, "guidance": guidance, "coast": coast,
        "rotation": rotation, "status": None, "eccentricity": None,
        "apoapsis_km": None, "periapsis_km": None, "target_km": None,
        "guidance_activated": None, "error": None, "duration_s": None,
    }

    # ---- inject config into simulation_parameters BEFORE importing solvers ----
    from Input_File import simulation_parameters as sim_params
    sim_params.VEHICLE = vehicle
    sim_params.PLANET = planet
    sim_params.GUIDANCE_MODE = guidance
    sim_params.COAST_METHOD = coast
    sim_params.ENABLE_EARTH_ROTATION = rotation
    # The repo's checked-in TARGET_ORBITAL_ALTITUDE is Moon-tuned (50 km, suborbital
    # for Earth). Force a real LEO target for Earth tests.
    sim_params.TARGET_ORBITAL_ALTITUDE = float(spec.get("target_alt_m", 200e3))
    for k, v in budget.items():
        setattr(sim_params, k, v)

    from Auxiliary import constants as c

    t0 = time.time()
    captured = io.StringIO()
    try:
        import main as sim_main
        # Skip the ~19-figure plot suite and the blocking show.
        import Plots.new_plot_runner as npr
        npr.run_new_plot_suite = lambda *a, **k: None
        import matplotlib.pyplot as plt
        plt.show = lambda *a, **k: None
        from Simulation import rocket_ascent as ra

        if os.environ.get("DEBUG_STDOUT"):
            time_arr, data, kick = sim_main.execute()
        else:
            with contextlib.redirect_stdout(captured):
                time_arr, data, kick = sim_main.execute()

        target_alt = float(sim_params.TARGET_ORBITAL_ALTITUDE)
        result["target_km"] = target_alt / 1000.0
        R_E = c.R_EARTH  # set_planet('earth') already applied inside execute()

        crashed = bool(getattr(ra, "CRASH_DETECTED", False))
        guid_start = getattr(ra, "time_guidance_start", None)
        # ra.time_guidance_start is only set on the apogee_check path (ra.run's own
        # guidance dispatch). The PSO solvers (direct/pso_coast/indirect_pmp) apply
        # guidance inside their own Stage-2 ODE and never touch this global, so the
        # activation flag is only meaningful for apogee_check.
        if coast == "apogee_check":
            result["guidance_activated"] = guid_start is not None
        else:
            result["guidance_activated"] = None

        r_final = float(data[1, -1])
        v_final = float(data[2, -1])
        gamma_final = float(data[3, -1])
        a, e, r_apo, r_peri, T = ra.get_orbital_elements(r_final, v_final, gamma_final)
        apo_alt = r_apo - R_E
        peri_alt = r_peri - R_E
        result["eccentricity"] = float(e)
        result["apoapsis_km"] = apo_alt / 1000.0
        result["periapsis_km"] = peri_alt / 1000.0

        if crashed:
            result["status"] = "CRASH"
        elif coast == "apogee_check":
            # Single-burn-to-apogee: success = apoapsis reached target (a
            # transfer orbit; low periapsis is expected by design).
            ok_apo = abs(apo_alt - target_alt) <= APO_TOL_FRAC * target_alt
            result["status"] = "PASS" if (ok_apo and peri_alt > -3e5) else "POOR"
        else:
            # direct / pso_coast / indirect_pmp: success = near-circular insertion.
            # A near-circular orbit straddles the target (apo above, peri below),
            # so judge by MEAN altitude + low eccentricity, not apo & peri each.
            mean_alt = 0.5 * (apo_alt + peri_alt)
            ok_alt = abs(mean_alt - target_alt) <= CIRC_TOL_FRAC * target_alt
            ok_circ = e <= ECC_TOL
            result["status"] = "PASS" if (ok_alt and ok_circ) else "POOR"

        # Note (not a downgrade): on the apogee_check path, guidance never
        # activating for a non-gravity-turn mode usually means the vehicle
        # overperformed and Stage 1 alone reached orbit.
        if (coast == "apogee_check" and result["guidance_activated"] is False
                and guidance not in ("gravity_turn", "indirect_pmp")):
            result["error"] = "guidance_never_activated"

    except ValueError as ex:
        msg = str(ex)
        result["status"] = "REJECTED" if any(m in msg for m in GUARD_MARKERS) else "ERROR"
        result["error"] = f"{type(ex).__name__}: {msg[:400]}"
    except BaseException as ex:  # noqa: BLE001 - catch everything for one combo
        result["status"] = "ERROR"
        result["error"] = f"{type(ex).__name__}: {str(ex)[:400]}"
    finally:
        result["duration_s"] = round(time.time() - t0, 1)

    sys.stdout.write("RESULT_JSON " + json.dumps(result) + "\n")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
