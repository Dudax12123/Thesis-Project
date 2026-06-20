"""
PARENT DRIVER - Earth-simulator choice-space sweep.

Runs the full (vehicle x guidance x coast) baseline matrix plus a rotation-ON
subset, each combo in a FRESH pygmo-env subprocess (mandatory: PSO solvers
snapshot stage constants at import; fresh processes also isolate hard crashes).
Collects one JSON result per combo into results.json and writes a readable table.

Usage (from anywhere):
  C:\\Users\\eduar\\miniforge3\\envs\\pygmo-env\\python.exe run_matrix.py [baseline|rotation|all]

Reduced PSO budgets + per-run timeouts keep the whole sweep to well under an hour;
the goal is to classify which paths RUN / PASS / POOR / CRASH / REJECT / ERROR,
not to converge each PSO to full precision.
"""

import sys
import os
import json
import time
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
PYGMO_PY = r"C:\Users\eduar\miniforge3\envs\pygmo-env\python.exe"
CHILD = str(HERE / "_single_run.py")
RESULTS_JSON = HERE / "results.json"
RESULTS_TABLE = HERE / "results_table.txt"

TARGET_ALT_M = 200e3
TIMEOUT_S = 420  # per-combo wall-clock cap; runaway PSO/brute-force -> TIMEOUT

# Reduced PSO budgets (enough to find a valid solution, far below file defaults).
# Direct (2-var) and coast (4-var) converge cheaply; indirect_pmp (7-var costate
# search) needs a much larger swarm and still tends not to circularise -- 60x100
# (~120 s) is the largest budget that fits comfortably under TIMEOUT_S.
REDUCED_BUDGET = {
    "PSO_DIRECT_N_PARTICLES": 24, "PSO_DIRECT_MAX_GENERATIONS": 40,
    "PSO_COAST_N_PARTICLES": 24,  "PSO_COAST_MAX_GENERATIONS": 40,
    "PSO_N_PARTICLES": 60,        "PSO_MAX_GENERATIONS": 100,
}

# Per-vehicle ascent overrides. The shipped TIME_TO_START_KICK=7.5 s suits
# high-T/W vehicles (falcon9) but pitches the low-T/W electron into the ground;
# electron needs a later pitch-over. (This is itself a documented finding.)
PER_VEHICLE = {
    "falcon9":  {},
    "electron": {"TIME_TO_START_KICK": 20},
}

VEHICLES = ["falcon9", "electron"]
GUIDANCE_NONINDIRECT = [
    "gravity_turn", "linear_tangent", "bilinear_tangent", "apollo",
    "cpr", "peg", "peg_new", "exp_shooting",
]
COASTS = ["apogee_check", "pso_coast", "direct"]


def make_spec(vehicle, guidance, coast, rotation=False, extra_budget=None):
    budget = dict(REDUCED_BUDGET)
    budget.update(PER_VEHICLE.get(vehicle, {}))
    if extra_budget:
        budget.update(extra_budget)
    return {
        "vehicle": vehicle, "guidance": guidance, "coast": coast,
        "rotation": rotation, "target_alt_m": TARGET_ALT_M, "budget": budget,
    }


def build_baseline():
    combos = []
    for v in VEHICLES:
        for g in GUIDANCE_NONINDIRECT:
            for cst in COASTS:
                combos.append(make_spec(v, g, cst, rotation=False))
        # indirect_pmp: own guidance mode; COAST_METHOD ignored (use 'direct').
        combos.append(make_spec(v, "indirect_pmp", "direct", rotation=False))
    return combos


def build_rotation_subset():
    """Rotation-ON probes on falcon9 to surface azimuth/ECI/pseudo-force and the
    known cpr brentq crash."""
    v = "falcon9"
    primary = {
        "gravity_turn": "apogee_check", "linear_tangent": "apogee_check",
        "bilinear_tangent": "apogee_check", "apollo": "direct",
        "cpr": "apogee_check", "peg": "apogee_check", "peg_new": "apogee_check",
        "exp_shooting": "direct",
    }
    combos = [make_spec(v, g, cst, rotation=True) for g, cst in primary.items()]
    combos.append(make_spec(v, "indirect_pmp", "direct", rotation=True))
    # extra probes
    combos.append(make_spec(v, "gravity_turn", "pso_coast", rotation=True))
    combos.append(make_spec(v, "cpr", "direct", rotation=True))
    # iterative-azimuth fallback (expected: warns and falls back to formula_compare)
    combos.append(make_spec(v, "gravity_turn", "pso_coast", rotation=True,
                            extra_budget={"AZIMUTH_INCLINATION_MODE": "iterative"}))
    return combos


def run_combo(spec, idx, total):
    label = (f"{spec['vehicle']:>8} | {spec['guidance']:<16} | "
             f"{spec['coast']:<12} | rot={'ON ' if spec['rotation'] else 'OFF'}")
    print(f"[{idx:>2}/{total}] {label} ... ", end="", flush=True)
    env = dict(os.environ)
    env["COMBO_SPEC"] = json.dumps(spec)
    env["PYTHONIOENCODING"] = "utf-8"
    t0 = time.time()
    try:
        proc = subprocess.run(
            [PYGMO_PY, CHILD], env=env, capture_output=True, text=True,
            timeout=TIMEOUT_S,
        )
        result = None
        for line in proc.stdout.splitlines():
            if line.startswith("RESULT_JSON "):
                result = json.loads(line[len("RESULT_JSON "):])
        if result is None:
            tail = (proc.stderr or proc.stdout or "").strip().splitlines()[-3:]
            result = {**_stub(spec), "status": "ERROR",
                      "error": "no RESULT_JSON; tail=" + " | ".join(tail)[:400]}
    except subprocess.TimeoutExpired:
        result = {**_stub(spec), "status": "TIMEOUT",
                  "error": f"exceeded {TIMEOUT_S}s"}
    result["duration_s"] = result.get("duration_s") or round(time.time() - t0, 1)
    print(f"{result['status']:<8} ({result['duration_s']}s)"
          + (f"  e={result['eccentricity']:.3g} "
             f"apo={result['apoapsis_km']:.0f} peri={result['periapsis_km']:.0f}km"
             if result.get("eccentricity") is not None else "")
          + (f"  [{result['error']}]" if result.get("error") else ""))
    return result


def _stub(spec):
    return {"vehicle": spec["vehicle"], "guidance": spec["guidance"],
            "coast": spec["coast"], "rotation": spec["rotation"],
            "eccentricity": None, "apoapsis_km": None, "periapsis_km": None,
            "target_km": TARGET_ALT_M / 1000, "guidance_activated": None,
            "error": None, "duration_s": None}


def write_table(results):
    lines = []
    lines.append(f"{'vehicle':>8} | {'guidance':<16} | {'coast':<12} | rot | "
                 f"{'status':<8} | {'ecc':>9} | {'apo_km':>8} | {'peri_km':>9} | "
                 f"{'dur_s':>6} | note")
    lines.append("-" * 120)
    for r in results:
        ecc = f"{r['eccentricity']:.3g}" if r.get("eccentricity") is not None else ""
        apo = f"{r['apoapsis_km']:.0f}" if r.get("apoapsis_km") is not None else ""
        per = f"{r['periapsis_km']:.0f}" if r.get("periapsis_km") is not None else ""
        lines.append(
            f"{r['vehicle']:>8} | {r['guidance']:<16} | {r['coast']:<12} | "
            f"{'ON ' if r['rotation'] else 'OFF'} | {r['status']:<8} | {ecc:>9} | "
            f"{apo:>8} | {per:>9} | {str(r.get('duration_s','')):>6} | "
            f"{r.get('error') or ''}")
    RESULTS_TABLE.write_text("\n".join(lines), encoding="utf-8")
    print("\n" + "\n".join(lines))


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    combos = []
    if which in ("baseline", "all"):
        combos += build_baseline()
    if which in ("rotation", "all"):
        combos += build_rotation_subset()

    total = len(combos)
    print(f"Earth-sim sweep: {total} combos  (mode={which}, timeout={TIMEOUT_S}s)\n")
    results = []
    for i, spec in enumerate(combos, 1):
        results.append(run_combo(spec, i, total))
        RESULTS_JSON.write_text(json.dumps(results, indent=2), encoding="utf-8")

    write_table(results)

    # summary counts
    from collections import Counter
    counts = Counter(r["status"] for r in results)
    print("\nSUMMARY:", dict(counts))
    print(f"results.json -> {RESULTS_JSON}")


if __name__ == "__main__":
    main()
