

"""
ROCKET SPECIFICATIONS - USER INPUT FILE

This file contains all the rocket specifications and parameters that define
the launch vehicle configuration. Vehicles are stored in the VEHICLES registry
below; select one from simulation_parameters.py via the VEHICLE parameter, which
main.py applies by calling set_vehicle().

The default vehicle is the SpaceX Falcon 9. All values are exposed as
module-level constants (M_PROP_1, F_THRUST_2, ...) so existing references such
as ``rocket_specs.M_PROP_2`` keep working unchanged; set_vehicle() simply
repopulates those constants for the chosen vehicle. This mirrors the
constants.set_planet() pattern.

Each vehicle declares:
- NUM_STAGES (1 or 2). Single-stage vehicles (e.g. the Apollo LM ascent stage)
  zero out the stage-2 fields and have guidance activate right after the
  pitch-over instead of waiting for a second-stage ignition.
- Payload / fairing mass
- Event timing intervals (stage separation, second engine ignition)
- Aerodynamic properties (cross-sectional area, drag/lift coefficients)
- First and (optionally) second stage engine + mass properties
The mass ratios at the bottom are derived in set_vehicle().
"""


# =======================================================
#  VEHICLE REGISTRY
# =======================================================
# Engine/mass values are public approximations — adequate for this ascent
# simulator; refine per vehicle as needed. Single-stage vehicles set the
# stage-2 fields to 0 (ISP_2 kept nonzero only to keep any Isp*g0 term finite;
# it is never used because F_THRUST_2 = 0 and the single-stage code paths
# bypass the stage-2 calculations entirely).

VEHICLES = {
    # ---- SpaceX Falcon 9 (Earth, two-stage) — original default ----
    "falcon9": {
        "NUM_STAGES": 2,
        "BODY": "earth",
        "M_PAYLOAD": 0e3,
        "M_FAIRING": 1900,
        "TIME_First_STAGE_SEPARATION": 3,
        "TIME_SECOND_ENGINE_IGNITION": 8,
        "A": 10.52, "C_D": 0.3, "C_L": 0.1,
        "ISP_1_SL": 283, "ISP_1_VAC": 311,
        "F_THRUST_1_SL": 7607e3, "F_THRUST_1_VAC": 8227e3,
        "M_STRUCTURE_1": 25.6e3, "M_PROP_1": 395.7e3,
        "ISP_2": 348, "F_THRUST_2": 934e3,
        "M_STRUCTURE_2": 3900, "M_PROP_2": 92670,
    },

    # ---- Apollo Lunar Module ascent stage (Moon, single-stage) ----
    # APS (Ascent Propulsion System): ~15.6 kN, Isp ~311 s, hypergolic.
    # Gross ascent ~4,700 kg; ascent propellant ~2,353 kg.
    "apollo_lm_ascent": {
        "NUM_STAGES": 1,
        "BODY": "moon",
        "M_PAYLOAD": 0,
        "M_FAIRING": 0,
        "TIME_First_STAGE_SEPARATION": 0,
        "TIME_SECOND_ENGINE_IGNITION": 0,
        "A": 21.0, "C_D": 0.0, "C_L": 0.0,
        "ISP_1_SL": 311, "ISP_1_VAC": 311,
        "F_THRUST_1_SL": 1.56e4, "F_THRUST_1_VAC": 1.56e4,
        "M_STRUCTURE_1": 2347, "M_PROP_1": 2353,
        "ISP_2": 311, "F_THRUST_2": 0,
        "M_STRUCTURE_2": 0, "M_PROP_2": 0,
    },

    # ---- Saturn V (Earth, approximated as two-stage-to-LEO: S-IC + S-II) ----
    # Upper stack (S-IVB + Apollo) carried as payload. Values approximate.
    "saturn_v": {
        "NUM_STAGES": 2,
        "BODY": "earth",
        "M_PAYLOAD": 140000,
        "M_FAIRING": 0,
        "TIME_First_STAGE_SEPARATION": 3,
        "TIME_SECOND_ENGINE_IGNITION": 8,
        "A": 80.1, "C_D": 0.3, "C_L": 0.1,
        "ISP_1_SL": 263, "ISP_1_VAC": 304,
        "F_THRUST_1_SL": 3.385e7, "F_THRUST_1_VAC": 3.885e7,
        "M_STRUCTURE_1": 131000, "M_PROP_1": 2149000,
        "ISP_2": 421, "F_THRUST_2": 5.17e6,
        "M_STRUCTURE_2": 36000, "M_PROP_2": 451000,
    },

    # ---- Rocket Lab Electron (Earth, two-stage small-lift) ----
    # 9x Rutherford stage 1, 1x Rutherford Vacuum stage 2. Values approximate.
    "electron": {
        "NUM_STAGES": 2,
        "BODY": "earth",
        "M_PAYLOAD": 200,
        "M_FAIRING": 50,
        "TIME_First_STAGE_SEPARATION": 3,
        "TIME_SECOND_ENGINE_IGNITION": 8,
        "A": 1.13, "C_D": 0.3, "C_L": 0.1,
        "ISP_1_SL": 303, "ISP_1_VAC": 311,
        "F_THRUST_1_SL": 1.62e5, "F_THRUST_1_VAC": 1.92e5,
        "M_STRUCTURE_1": 950, "M_PROP_1": 9250,
        "ISP_2": 343, "F_THRUST_2": 2.58e4,
        "M_STRUCTURE_2": 250, "M_PROP_2": 2050,
    },
}


def set_vehicle(name: str) -> None:
    """Override module-level vehicle constants for the selected vehicle.

    Mirrors constants.set_planet(). Repopulates every vehicle constant from the
    VEHICLES registry and recomputes the derived mass ratios.

    NOTE: must be called before any module that snapshots vehicle constants at
    import time (e.g. the PSO solvers cache F_THRUST_2 / ISP_2 in module globals).
    main.py calls this right after constants.set_planet(), before solver imports.
    """
    global NUM_STAGES, BODY
    global M_PAYLOAD, M_FAIRING, TIME_First_STAGE_SEPARATION, TIME_SECOND_ENGINE_IGNITION
    global A, C_D, C_L
    global ISP_1_SL, ISP_1_VAC, ISP_1, F_THRUST_1_SL, F_THRUST_1_VAC, F_THRUST_1
    global M_STRUCTURE_1, M_PROP_1
    global ISP_2, F_THRUST_2, M_STRUCTURE_2, M_PROP_2
    global M_TOTAL_1, LAMBDA_1, EPSILON_1, PI_1
    global M_TOTAL_2, LAMBDA_2, EPSILON_2, PI_2

    if name not in VEHICLES:
        raise ValueError(
            f"Unknown VEHICLE '{name}'. Available: {sorted(VEHICLES.keys())}")

    v = VEHICLES[name]

    NUM_STAGES = v["NUM_STAGES"]
    BODY = v["BODY"]                      # intended celestial body (must match PLANET)

    # Payload / fairing
    M_PAYLOAD = v["M_PAYLOAD"]
    M_FAIRING = v["M_FAIRING"]

    # Event intervals
    TIME_First_STAGE_SEPARATION = v["TIME_First_STAGE_SEPARATION"]
    TIME_SECOND_ENGINE_IGNITION = v["TIME_SECOND_ENGINE_IGNITION"]

    # Aerodynamics
    A = v["A"]
    C_D = v["C_D"]
    C_L = v["C_L"]

    # First stage
    ISP_1_SL = v["ISP_1_SL"]
    ISP_1_VAC = v["ISP_1_VAC"]
    ISP_1 = ISP_1_SL                      # backward-compat alias (sea-level value)
    F_THRUST_1_SL = v["F_THRUST_1_SL"]
    F_THRUST_1_VAC = v["F_THRUST_1_VAC"]
    F_THRUST_1 = F_THRUST_1_SL            # backward-compat alias (sea-level value)
    M_STRUCTURE_1 = v["M_STRUCTURE_1"]
    M_PROP_1 = v["M_PROP_1"]

    # Second stage (zeroed for single-stage vehicles)
    ISP_2 = v["ISP_2"]
    F_THRUST_2 = v["F_THRUST_2"]
    M_STRUCTURE_2 = v["M_STRUCTURE_2"]
    M_PROP_2 = v["M_PROP_2"]

    # ---- Derived mass ratios ----
    # First stage: total mass at lift-off includes structure, propellant,
    # stage 2 (zero for single-stage), and payload.
    M_TOTAL_1 = M_STRUCTURE_1 + M_PROP_1 + M_STRUCTURE_2 + M_PROP_2 + M_PAYLOAD
    LAMBDA_1 = M_PROP_1 / M_TOTAL_1 if M_TOTAL_1 else 0.0
    EPSILON_1 = M_STRUCTURE_1 / M_TOTAL_1 if M_TOTAL_1 else 0.0
    PI_1 = (M_PAYLOAD + M_STRUCTURE_2 + M_PROP_2) / M_TOTAL_1 if M_TOTAL_1 else 0.0

    # Second stage: total mass at stage-2 ignition (zero for single-stage).
    M_TOTAL_2 = M_STRUCTURE_2 + M_PROP_2 + M_PAYLOAD
    LAMBDA_2 = M_PROP_2 / M_TOTAL_2 if M_TOTAL_2 else 0.0
    EPSILON_2 = M_STRUCTURE_2 / M_TOTAL_2 if M_TOTAL_2 else 0.0
    PI_2 = M_PAYLOAD / M_TOTAL_2 if M_TOTAL_2 else 0.0


# Populate module-level constants with the default vehicle at import time so that
# `from Auxiliary import rocket_specs as r` exposes a complete, valid set of
# constants even before main.py calls set_vehicle().
set_vehicle("falcon9")
