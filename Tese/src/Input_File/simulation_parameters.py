"""
================================================================================
 SIMULATION PARAMETERS  —  ascent-trajectory simulator control panel
================================================================================

Single hand-edited control panel for the whole ascent simulator. Every constant
here is read by the consumers via
``from Input_File import simulation_parameters as sim_params`` and accessed by
name (``sim_params.NAME``); definition order is irrelevant to every consumer, so
the sections below are grouped purely for readability. Frequently-edited knobs
(mission, engine modes, guidance/coast selection) come first; fine-tuning
constants (PSO settings, penalty weights) are grouped near the end.

Table of contents
------------------
  1.  MISSION & TARGET ORBIT
  2.  LAUNCH SITE & EARTH ROTATION
  3.  AERODYNAMICS / PHYSICS TOGGLES
  4.  AZIMUTH / INCLINATION TARGETING
  5.  STAGE-1 ENGINE MODEL          (Isp / thrust modes)
  6.  ATMOSPHERE-EXIT MARKER        (guidance-start trigger)
  7.  KICK MANEUVER & ASCENT PROFILE
  8.  GUIDANCE MODE SELECTION
        8a. shared guidance parameters
        8b. Apollo / polynomial guidance
        8c. linear & bilinear tangent steering
        8d. constant pitch rate (CPR)
        8e. PEG / PEG-new
  9.  COAST METHOD SELECTION        (+ direct-insertion tolerances)
  10. OPTIMIZATION                  (apogee_check brute search)
  11. PSO OPTIMIZERS
        11a. indirect-PMP PSO       (GUIDANCE_MODE   = "indirect_pmp")
        11b. coast PSO              (COAST_METHOD    = "pso_coast")
        11c. direct PSO             (COAST_METHOD    = "direct")
  12. FAST-RUN MODE
  13. SIMULATION OUTPUT & TIMING
  14. DEBUGGING FLAGS
================================================================================
"""

import numpy as np


# ===================================================================
# 1. MISSION & TARGET ORBIT
# ===================================================================
# The orbit being targeted. Inclination pairs with the launch-site latitude
# (§2) to set the launch azimuth — see §4 Azimuth / Inclination targeting.
TARGET_ORBITAL_ALTITUDE = 500e3                 # altitude of desired orbit; [m]
TARGET_ORBIT_INCLINATION = 51.6                 # desired final orbit inclination; [deg]


# ===================================================================
# 2. LAUNCH SITE & EARTH ROTATION
# ===================================================================
ENABLE_EARTH_ROTATION = True                    # if True, include Earth rotation effects in azimuth/ECI calculations
LAUNCH_LATITUDE = 28.5                           # launch site latitude; [deg]
LAUNCH_LONGITUDE = -80.5                          # launch site longitude; [deg] (reserved for future launch window modeling)

# -------------- Rotating-frame pseudo-forces (require Earth rotation) --------------
INCLUDE_PSEUDO_FORCES = True                     # if True, include Coriolis and centrifugal accelerations in rotating-frame EOM
# Cross-heading actuator counter-force. The heading is held fixed at the launch
# azimuth — we assume the launcher's actuator cancels the lateral (cross-heading)
# pseudo-force rather than letting it turn the vehicle — so this has no effect on the
# in-plane trajectory. When True, the per-step counter-force the actuator must supply,
# m*|a_cross| [N], is computed, stored and plotted (as kN vs time). Requires
# ENABLE_EARTH_ROTATION and INCLUDE_PSEUDO_FORCES.
COMPUTE_CROSS_HEADING_COUNTER_FORCE = False


# ===================================================================
# 3. AERODYNAMICS / PHYSICS TOGGLES
# ===================================================================
INCLUDE_DRAG = True                              # if True, include aerodynamic drag force in the EOM (F_D = q * C_D * A).
                                                 # Setting this False is the master NO-ATMOSPHERE switch: lift is also
                                                 # forced off, the fairing is not carried (launched without it), and the
                                                 # atmosphere-exit marker uses the altitude method.
INCLUDE_LIFT = True                              # if True, include aerodynamic lift force in the EOM (F_L = q * C_L * A).
                                                 # Only effective while INCLUDE_DRAG is True (no lift without atmosphere).


# ===================================================================
# 4. AZIMUTH / INCLINATION TARGETING
# ===================================================================
# All three modes derive the initial launch azimuth from the spherical-geometry formula:
#   sin(beta) = cos(i_target) / cos(phi_launch)
# (i_target = TARGET_ORBIT_INCLINATION [§1], phi_launch = LAUNCH_LATITUDE [§2].)
# They differ in how they analyse the gap between that formula and the real achieved inclination.
#
#   "formula_compare":      Fly with the formula azimuth.
#                           Report the achieved inclination and its deviation from the target.
#
#   "formula_back_compare": Same as "formula_compare", but also back-derives an azimuth from
#                           the achieved inclination via the same formula and reports the
#                           difference between the formula azimuth and that back-derived azimuth.
#
#   "iterative":            Sweeps the launch azimuth over
#                           [beta_formula - RANGE, beta_formula + RANGE] in steps of
#                           AZIMUTH_ITER_STEP_DEG to find the azimuth that best achieves
#                           the target inclination.  The kick angle is fixed from the
#                           initial optimisation run (re-optimising per azimuth is too costly).
AZIMUTH_INCLINATION_MODE = "formula_compare"  # Options: "formula_compare", "formula_back_compare", "iterative"
AZIMUTH_ITER_STEP_DEG  = 0.1                  # [deg] azimuth step size for iterative sweep (only used when mode = "iterative")
AZIMUTH_ITER_RANGE_DEG = 10.0                 # [deg] sweep half-width around formula azimuth (only used when mode = "iterative")
AZIMUTH_ITER_TOL_DEG   = 0.05                 # [deg] inclination tolerance — warns and falls back if no solution found within this bound


# ===================================================================
# 5. STAGE-1 ENGINE MODEL  (Isp / thrust modes)
# ===================================================================
# Isp and thrust feed the shared Stage-1 EOM (rocket_ascent._get_stage1_isp /
# _get_stage1_thrust), so these knobs are honored identically by every backend.

# -------------- Stage 1 Specific Impulse Mode --------------
# Select which Isp value to use for the first stage engine:
#   "sea_level":  Use sea-level Isp (ISP_1_SL) throughout stage 1 — most conservative
#   "vacuum":     Use vacuum Isp (ISP_1_VAC) throughout stage 1 — best-case efficiency
#   "average":    Use the mean of sea-level and vacuum Isp — simple middle ground
#   "linear":     Linearly ramp from ISP_1_SL at ignition to ISP_1_VAC at stage-1 burnout,
#                 updating every ISP_1_LINEAR_UPDATE_RATE seconds (discrete steps)
ISP_1_MODE = "sea_level"                        # Options: "sea_level", "vacuum", "average", "linear"
ISP_1_LINEAR_UPDATE_RATE = 5.0                  # [s] step interval for linear ramp (only used when ISP_1_MODE = "linear")

# -------------- Stage 1 Thrust Mode --------------
# Select which thrust value to use for the first stage engine:
#   "sea_level":  Use sea-level thrust (F_THRUST_1_SL) throughout stage 1 — most conservative
#   "vacuum":     Use vacuum thrust (F_THRUST_1_VAC) throughout stage 1 — best-case performance
#   "average":    Use the mean of sea-level and vacuum thrust — simple middle ground
#   "linear":     Linearly ramp from F_THRUST_1_SL at ignition to F_THRUST_1_VAC at stage-1 burnout,
#                 updating every THRUST_1_LINEAR_UPDATE_RATE seconds (discrete steps)
# CAVEAT: "vacuum" over-performs Stage 1 so much that COAST_METHOD="apogee_check"
# can no longer land the apogee on target for any kick — the kick search raises a
# ValueError (use COAST_METHOD="pso_coast"/"direct", or "sea_level"/"average" here).
THRUST_1_MODE = "sea_level"                     # Options: "sea_level", "vacuum", "average", "linear"
THRUST_1_LINEAR_UPDATE_RATE = 5.0               # [s] step interval for linear ramp (only used when THRUST_1_MODE = "linear")


# ===================================================================
# 6. ATMOSPHERE-EXIT MARKER  (guidance-start trigger)
# ===================================================================
# Choose how to detect when the rocket exits the atmosphere and guidance should start:
#   "altitude":         Use altitude threshold (traditional method)
#   "dynamic_pressure": Use dynamic pressure threshold (more physically meaningful)
#   "aerothermal_flux": Use aerothermal flux threshold (Phi = 0.5*rho*v^3)
ATMOSPHERE_EXIT_METHOD = "dynamic_pressure"     # Options: "altitude", "dynamic_pressure", "aerothermal_flux"
ALT_NO_ATMOSPHERE = 65e3                        # altitude threshold for atmosphere exit; [m]
                                                #   (only used if ATMOSPHERE_EXIT_METHOD = "altitude")
DYNAMIC_PRESSURE_THRESHOLD = 1000.0             # dynamic pressure threshold [Pa]
                                                #   (only used if ATMOSPHERE_EXIT_METHOD = "dynamic_pressure")
                                                #   Typical value: 1000 Pa (fairly low, indicating thin atmosphere)
AEROTHERMAL_FLUX_THRESHOLD = 1135.0             # aerothermal flux threshold [W/m^2]
                                                #   (only used if ATMOSPHERE_EXIT_METHOD = "aerothermal_flux")
                                                #   Phi = 0.5*rho*v^3; negligible heating below this value


# ===================================================================
# 7. KICK MANEUVER & ASCENT PROFILE
# ===================================================================
TIME_TO_START_KICK = 7.5                        # time at which the kick maneuver begins; [s]
DURATION_INITIAL_KICK = 45.                     # duration of the triangular alpha kick profile [s]
                                                # (only used when KICK_PROFILE_MODE == "triangular")

# Kick-maneuver PROFILE for run()'s Stage-1A:
#   "triangular"    : ramped alpha-kick via pitch_program_linear over
#                      DURATION_INITIAL_KICK seconds. Kick angle is searched
#                      directly over [ALPHA_LOWEST, ALPHA_HIGHEST] rad.
#   "instantaneous" : discontinuous gamma jump via _run_stage1a_with_kick
#                      (same mechanism as pso_coast/indirect_pmp). Kick angle
#                      convention becomes gamma_p in [1.54, 1.57] rad, with
#                      kick_angle = gamma_p - pi/2 computed internally.
KICK_PROFILE_MODE = "triangular"   # Options: "triangular", "instantaneous"

# Initial kick angle used for single (non-optimised) runs. The apogee_check
# brute search over the kick angle uses ALPHA_LOWEST/ALPHA_HIGHEST — see §10.
INITIAL_KICK_ANGLE = - np.deg2rad(3.0)          # Initial kick angle [rad]


# ===================================================================
# 8. GUIDANCE MODE SELECTION
# ===================================================================
# Choose the guidance strategy for the trajectory:
#   "gravity_turn": Pure gravity turn all the way (traditional method)
#                   - Initial kick maneuver, then zero angle of attack throughout
#                   - No active guidance after kick
#   "linear_tangent": Linear tangent steering law (classical guidance)
#                   - Initial kick until atmosphere exit
#                   - tan(α + γ) varies linearly with time-to-go
#                   - Classic ascent guidance method
#   "bilinear_tangent": Bilinear tangent steering law (advanced guidance)
#                   - Initial kick until atmosphere exit
#                   - tan(α + γ) = ratio of two linear functions of time-to-go
#                   - More flexible than linear tangent, controls value and derivative
#   "apollo":       Apollo polynomial guidance (classical explicit guidance)
#                   - Initial kick until atmosphere exit
#                   - Polynomial acceleration profiles in x and y directions
#                   - Enforces position and velocity terminal constraints
#                   - Used in Apollo missions; enforces full terminal constraints
#                   - The k3/k4 (vertical) coefficients target vy=0, y=y_target
#                     AT t_go — i.e. a full orbit-insertion endpoint. Use with
#                     COAST_METHOD="direct" (which checks for that same
#                     endpoint). COAST_METHOD="apogee_check" cuts the burn on a
#                     totally different mid-flight condition (osculating apogee
#                     reaching y_target while vy is still large) and is NOT a
#                     workable pairing with apollo — use "peg_new" for that.
#                     (main.py raises ValueError for apollo + apogee_check.)
#   "cpr":          Constant Pitch Rate guidance
#                   - No kick maneuver — flies vertical, then CPR takes over immediately
#                   - Linearly ramps pitch angle θ from 90° (vertical) to 0° (horizontal)
#                   - θ_dot = (90° − 0°) / t_go; α = θ_cmd − γ at each step
#                   - t_go estimated with Apollo propellant-based formula at guidance start
#                   - No kick angle optimisation required
#   "peg":          Powered Explicit Guidance (PEG)
#                   - Same kick + gravity-turn Stage 1 as other modes
#                   - Activates in Stage 2 after atmosphere exit only
#                   - Solves for linear pitch program sin(pitch) = A + B*t each major cycle
#                   - Explicitly targets r_T, ṙ_T = 0, v_θ_T = √(μ/r_T) for circular orbit
#   "peg_new":      Analytical Predictor-Corrector PEG (from first principles)
#                   - Derived from Pontryagin's minimum principle (Jaggers 1977, McHenry et al. 1979)
#                   - Primary variable: v_go (2D velocity-to-be-gained vector)
#                   - t_go from rocket equation: t_go = τ·(1−exp(−‖v_go‖/c))
#                   - Position costate λ'_r from analytical formula (paper eq 71)
#                   - Steering: û = v_go/‖v_go‖ + λ'_r·(t−t_λ)·r̂  (normalised, eq 72)
#                   - Gravity handled naturally through v_go iteration (no ad-hoc C correction)
#                   - Stage 2 only, after atmosphere exit
#   "exp_shooting": Exponential pitch-law guidance with single-shot shooting optimization
#                   - Pitch angle: θ(t_rel) = a·exp(b·t_rel), α = θ − γ
#                   - (a, b) solved once at guidance start via scipy.optimize.fsolve
#                   - Terminal constraints: r(T_burnout) = r_T, γ(T_burnout) = 0
#                   - Fixed coefficients for the entire burn (open-loop after initialization)
#   "indirect_pmp": Indirect method via Pontryagin's Minimum Principle (PMP)
#                   - PSO (PyGMO or scipy.differential_evolution) optimises initial
#                     costate values plus coast/burn timing and kick angle jointly
#                   - Stage 1: standard gravity turn (PSO-chosen kick angle)
#                   - Stage 2: costates [λ_r, λ_v, λ_γ] propagated alongside the
#                     state with drag-free EOM (free-flight phase after fairing jettison)
#                   - Control α = atan2(−λ_γ/V, −λ_V) at every timestep (Eq. 34)
#                   - Coast phase timing fully controlled by PSO (apogee trigger NOT used)
#                   - Objective: burn time + terminal constraint penalties (Eq. 39)
#                   - See indirect_pso_solver.py and indirect_pmp_guidance.py
#                   - PSO tuning for this mode lives in §11a (PSO_* constants).
GUIDANCE_MODE = "gravity_turn"  # Options: "gravity_turn", "linear_tangent", "bilinear_tangent", "apollo", "cpr", "peg", "peg_new", "exp_shooting", "indirect_pmp"

# -------------- 8a. Shared guidance parameters --------------
# (GUIDANCE_UPDATE_RATE is used by apollo and linear_tangent/bilinear_tangent;
#  see §8b/§8c. TGO_ESTIMATOR / GUIDANCE_TGO_USE_PSO_PLAN feed the scalar-t_go modes.)
GUIDANCE_UPDATE_RATE = 2                       # How often to recompute guidance coefficients [s]

# -------------- Time-to-go estimator (apollo / linear_tangent / bilinear_tangent / cpr-"tgo") --------------
# Selects how t_go is estimated for the modes that consume it as a scalar:
#   "rocket_equation": gravity-blind  T_BUP·(1−exp(−VG/Ve))   (current default)
#   "peg_new":         peg_new's gravity-aware estimate  τ·(1−exp(−‖v_go‖/Ve))
#                      with the radial gravity loss folded in (predictor-corrector).
# Affects apollo / linear_tangent / bilinear_tangent and cpr in "tgo" mode (cpr
# under pso_coast uses the PSO-optimised θ_dot, so it is unaffected there). peg has
# its own internal T solver and is NOT affected; peg_new is the source.
TGO_ESTIMATOR = "rocket_equation"   # Options: "rocket_equation", "peg_new"

# -------------- PSO-Planned t_go Override (PSO solvers only) --------------
GUIDANCE_TGO_USE_PSO_PLAN = False              # If True, t_go for apollo/linear_tangent/bilinear_tangent/
                                              # cpr/peg is the PSO-planned burn-time countdown
                                              # (planned burn-arc end time - t) instead of the
                                              # rocket-equation estimate (_compute_tgo_stage2).
                                              # peg_new is unaffected. Only has an effect inside
                                              # pso_coast_solver / direct_pso_solver (no effect on
                                              # rocket_ascent.run()).

# ===================================================================
# 8a-bis. SEGMENTED (multi-law, altitude-triggered) GUIDANCE
# ===================================================================
# When MULTI_GUIDANCE_ENABLED is True the single GUIDANCE_MODE law is ignored and
# the rocket flies the ordered GUIDANCE_SEGMENTS schedule instead: a passive
# gravity turn from launch until the FIRST entry's altitude, then each chosen law
# in turn. Each non-final segment aims at the indirect-PMP optimal (alt, v, γ)
# waypoint at the NEXT entry's altitude; the LAST entry aims at orbit insertion.
# t_go is the planned-deadline countdown (deadline − t), NOT the rocket-equation
# estimate, so it never collapses across the stage boundary.
#
# NOTE: When MULTI_GUIDANCE_ENABLED is False (default) NONE of this has any effect
# — every existing mode/path behaves exactly as before.
MULTI_GUIDANCE_ENABLED = False

# Ordered list of (guidance_law, activation_altitude_m). Altitudes MUST be
# strictly increasing. Supported laws this iteration:
#   "apollo", "peg_new", "linear_tangent", "bilinear_tangent".
# (linear_tangent / bilinear_tangent are angle-only: they match the waypoint's
#  flight-path angle but not its altitude/velocity, so their tracking is weaker.)
GUIDANCE_SEGMENTS = [
    ("apollo",  40_000.0),     # Apollo takes over at 40 km, aims at PMP(120 km)
    ("peg_new", 120_000.0),    # peg_new takes over at 120 km, aims at orbit insertion
]

# Per-segment coefficient-freeze time-to-go [s] for the intermediate (non-final)
# segments. Smaller than APOLLO_FREEZE_THRESHOLD so short shaping segments are not
# frozen the instant they start. The final segment uses APOLLO_FREEZE_THRESHOLD.
SEGMENT_INTERMEDIATE_FREEZE_THRESHOLD = 2.0

# --- Indirect-PMP reference trajectory (supplies the segment waypoints) ---
SEGMENT_TARGET_SOURCE     = "pmp"   # "pmp" (interpolate the PMP reference) — only option for now
PMP_REFERENCE_CACHE       = "Tese/src/Output/pmp_reference.npz"  # cache file path
PMP_REFERENCE_USE_CACHE   = True    # load the cache if present and inputs unchanged
PMP_REFERENCE_FORCE_RERUN = False   # recompute the PMP reference even if a valid cache exists

# -------------- 8b. Apollo / polynomial guidance --------------
# (Only used if GUIDANCE_MODE is "apollo". APOLLO_FREEZE_THRESHOLD is also the
#  freeze threshold for peg/peg_new — see §8e.)
APOLLO_FREEZE_THRESHOLD = 10.0                  # Time-to-go threshold to freeze Apollo coefficients [s]
                                                 # (prevents numerical instability as tgo->0)
APOLLO_THRUST_MAGNITUDE_CONTROL = False          # Enable thrust magnitude control for Apollo guidance
                                                 # If True: Apollo commands both thrust angle AND magnitude
                                                 # If False: Apollo only commands angle (fixed thrust)

# -------------- 8c. Linear / bilinear tangent steering --------------
# (Only used if GUIDANCE_MODE is "linear_tangent" or "bilinear_tangent")
GUIDANCE_COEFFICIENTS_FIXED = True           # If True, coefficients are computed once at guidance
                                              # start and held constant; only t_go varies each step
                                              # (t_go is always recomputed each step).
                                              # If False (default), recomputed every GUIDANCE_UPDATE_RATE s.

# -------------- 8d. Constant Pitch Rate (CPR) guidance --------------
# (Only used by COAST_METHOD="apogee_check". Under "pso_coast" the constant
#  pitch rate is a PSO decision variable — see PSO_COAST_CPR_THETA_DOT_* in §11b —
#  so these two are ignored there.)
CPR_THETA_DOT_MODE = "manual"       # How to determine the constant pitch rate:
                                  #   "tgo":    θ_dot = (90°) / t_go  where t_go is from the
                                  #             Apollo propellant-based rocket-equation estimate
                                  #   "manual": use CPR_THETA_DOT directly (duration derived)
CPR_THETA_DOT = 0.4              # Between {0.1, 0.5}[deg/s] manual pitch rate (only used when CPR_THETA_DOT_MODE = "manual")
                                  # Guidance duration = 90° / CPR_THETA_DOT

# -------------- 8e. PEG / PEG-new guidance --------------
# (Only used if GUIDANCE_MODE is "peg")
PEG_MAJOR_LOOP_RATE = 2.0           # Major-loop update period [s] — how often A, B, T are recomputed

PEG_CONVERGENCE_MODE = "damped"     # Guide+Estimate convergence method:
                                     #   "damped":     damped fixed-point iteration (recommended)
                                     #                 T_next = PEG_CONVERGENCE_DAMPING * T_est
                                     #                        + (1 - PEG_CONVERGENCE_DAMPING) * T_current
                                     #                 stops when |ΔT| < PEG_CONVERGENCE_TOL
                                     #   "fixed_iter": N undamped iterations — may oscillate at Stage-2 start
PEG_CONVERGENCE_DAMPING = 0.5       # Damping factor ∈ (0, 1] (only used when mode = "damped")
                                     # Lower = more damping, slower but safer. 0.5 recommended.
PEG_CONVERGENCE_TOL = 0.5           # Convergence tolerance [s] (only used when mode = "damped")
PEG_CONVERGENCE_MAX_ITER = 30       # Max iterations for both modes
                                     # "fixed_iter": runs exactly this many iterations
                                     # "damped":     upper bound (usually converges in < 10)


# ===================================================================
# 9. COAST METHOD SELECTION
# (applies to all guidance modes except "indirect_pmp")
# ===================================================================
# Choose how the coast start is determined during Stage 2:
#   "apogee_check" : current method (unchanged).
#                    interrupt_single_burn_traj fires when the rocket's
#                    instantaneous apogee equals the target altitude.
#                    The rocket then coasts ballistically to apogee and an
#                    impulsive circularisation burn is applied.
#   "pso_coast"    : PSO simultaneously optimises kick angle (gamma_p),
#                    total Stage-2 burn fraction (delta_tr_pct), coast-start
#                    fraction (coast_start_pct), and coast duration (delta_tc).
#                    The trajectory is Thrust → Coast → Thrust with direct
#                    orbit insertion — no separate circularisation burn.
#                    The selected guidance law (apollo, peg, …) steers the
#                    rocket during both thrust arcs.
#                    Has no effect when GUIDANCE_MODE = "indirect_pmp" (that
#                    mode always uses its own PSO with costates).
#                    CAVEAT: "exp_shooting" is a weak fit with "pso_coast" — its
#                    BVP assumes one continuous burn to propellant depletion,
#                    which the thrust-coast-thrust split forbids. Prefer a
#                    feedback law (linear_tangent, apollo, peg, peg_new) here.
#                    PSO tuning for this method lives in §11b (PSO_COAST_*).
#   "direct"       : Continuous single Stage-2 burn to DIRECT orbit insertion —
#                    no coast, no circularisation burn. Always PSO: a 2-variable
#                    PSO (direct_pso_solver) optimises gamma_p (kick angle) AND the
#                    Stage-2 burn duration so the insertion lands in the
#                    DIRECT_INSERTION_* box — inertial velocity = circular
#                    √(μ/r_target), flight-path angle ≈ 0, altitude ≈ target.
#                    The achieved orbit (eccentricity, apo/peri) and whether the
#                    insertion was "clean" (all three within tolerance) are
#                    reported. Requires PyGMO. Has no effect when
#                    GUIDANCE_MODE = "indirect_pmp" (that mode runs its own PSO).
#                    Intended for the direct-insertion laws (peg, peg_new, apollo).
#                    CAVEAT: a single continuous burn (no coast) is delta-v-marginal,
#                    so ONLY {apollo, peg, peg_new} reach the target circular orbit.
#                    gravity_turn/linear_tangent/bilinear_tangent/cpr/exp_shooting
#                    converge (budget-independently) to a SUBORBITAL insertion here —
#                    use "pso_coast"/"apogee_check" (which have a coast) for those.
#                    direct_pso_solver prints a warning for those pairings.
#                    PSO tuning for this method lives in §11c (PSO_DIRECT_*).
COAST_METHOD = "pso_coast"   # Options: "apogee_check", "pso_coast", "direct"

# -------------- Direct-insertion REPORTING tolerances --------------
# Diagnostic only (COAST_METHOD == "direct"): these do NOT affect the PSO solve, the
# optimization objective, or the engine cutoff. MECO fires at circular velocity
# (interrupt_velocity_exceeded); the PSO drives insertion accuracy through the
# PSO_DIRECT_W_* weights in §11c, not through these thresholds.
# After the solve, the achieved insertion is graded "clean" only if the velocity,
# flight-path-angle AND altitude errors are ALL within the tolerances below; this sets
# the printed "Clean insertion (within tol)" verdict (see interrupt_direct_insertion
# in rocket_ascent.py and the report in main.py). Otherwise the achieved orbit is
# reported as-is.
DIRECT_INSERTION_VELOCITY_TOL_MS  = 10.0   # |v_inertial − √(μ/r_target)| [m/s]
DIRECT_INSERTION_FPA_TOL_DEG      = 0.5    # |flight-path angle|          [deg]
DIRECT_INSERTION_ALTITUDE_TOL_KM  = 5.0    # |altitude − target|         [km]


# ===================================================================
# 10. OPTIMIZATION  (apogee_check brute search)
# ===================================================================
# ALPHA_LOWEST/ALPHA_HIGHEST are the kick-angle brute-search bounds used by the
# apogee_check solver (solver.find_initial_kick_angle_coast_single_burn). The
# single-run kick angle is INITIAL_KICK_ANGLE (§7).
ALPHA_LOWEST = -np.deg2rad(5.5)                  # lowest possible kick angle to be tested; [rad]
ALPHA_HIGHEST = -np.deg2rad(2.5)                 # highest possible kick angle to be tested; [rad]
MAX_ACCEPTED_BURN_TIME = 100.                    # maximum accepted burn time of delta-v; [s]
# apogee_check: a kick angle is accepted only if its achieved apogee is within this
# fraction of the target radius. Tight (0.0002 ≈ 1.4 km) now that the apogee
# interrupt and the SECO conversion use the same (launch) latitude.
APOGEE_MATCH_TOL_FRAC = 0.0002                   # apogee match tolerance (fraction of r_target)


# ===================================================================
# 11. PSO OPTIMIZERS
# ===================================================================

# -------------------------------------------------------------------
# 11a. Indirect-PMP PSO   (only used when GUIDANCE_MODE = "indirect_pmp")
# -------------------------------------------------------------------

# -------------- PSO algorithm settings (from paper Sect. 4.2.2) --------------
PSO_N_PARTICLES     = 250      # swarm size
PSO_MAX_GENERATIONS = 500      # maximum number of generations
PSO_C1              = 2.05      # cognitive parameter (paper default)
PSO_C2              = 2.05      # social parameter   (paper default)
PSO_OMEGA           = 0.7298    # inertia weight      (paper default)
PSO_VMAX            = 0.5       # maximum particle velocity (normalised)
PSO_SEED            = 42        # RNG seed for reproducible PSO runs

# -------------- Decision-variable bounds (Table 6 of paper) ------------------
# x = [lambda0_r, lambda0_v, lambda0_g, delta_tc, delta_tr_pct, coast_start_pct, gamma_p]
PSO_LB = [-1.0,  -1.0,  -1.0,   0.0,   0.0,   0.0,  1.54]   # lower bounds
PSO_UB = [ 1.0,   1.0,   1.0, 2000.0, 100.0, 100.0,  1.57]   # upper bounds
# lambda0_{r,v,g}   : initial costate values for Stage 2     [−1, 1]
# delta_tc          : coast phase duration                    [0, 2000] s
# delta_tr_pct      : Stage-2 burn as % of max propellant time [0, 100] %
# coast_start_pct   : coast start as % of Stage-2 burn time   [0, 100] %
# gamma_p           : pitch maneuver angle                    [1.54, 1.57] rad

# -------------- Penalty weight factors for augmented objective (Eq. 39) ------
# All terms are now NON-DIMENSIONAL (see indirect_pso_solver._objective_terms),
# so these weights are unitless and directly comparable. Tuned so that near
# feasibility (<1% terminal errors) the penalties are O(1) — comparable to the
# normalised burn-time term J_nd ∈ [0, 1] — while a few-percent miss makes the
# constraints dominate, forcing PSO to hit the target orbit before shaving burn
# time. Lower the constraint weights later if feasibility becomes reliable.
PENALTY_W_J         = 1.0       # burn-time term (J normalised by T_MAX_2)
PENALTY_W_ALTITUDE  = 100.0     # s1: relative altitude error (1% error -> 1.0)
PENALTY_W_VELOCITY  = 100.0     # s2: relative velocity error (1% error -> 1.0)
PENALTY_W_FPA       = 10.0      # s3: FPA error in deg        (1 deg  -> 10.0)
PENALTY_W_TRANSVERS = 10.0       # s4: transversality (meaningful after ‖λ₀‖=1)
GAMMA_REF_DEG       = 1.0       # FPA non-dimensionalisation reference [deg]

# -------------------------------------------------------------------
# 11b. Coast PSO   (only used when COAST_METHOD == "pso_coast")
# -------------------------------------------------------------------

# -------------- PSO COAST algorithm settings --------------
PSO_COAST_N_PARTICLES     = 100      # swarm size
PSO_COAST_MAX_GENERATIONS = 250      # maximum number of generations
PSO_COAST_C1              = 2.05    # cognitive parameter
PSO_COAST_C2              = 2.05    # social parameter
PSO_COAST_OMEGA           = 0.7298  # inertia weight
PSO_COAST_VMAX            = 0.5     # maximum particle velocity (normalised)
PSO_COAST_SEED            = 42      # RNG seed for reproducible runs

# Decision-variable bounds for the 4-variable coast PSO:
# x = [delta_tc, delta_tr_pct, coast_start_pct, gamma_p]
# gamma_p is bounded to [1.54, 1.57] rad (~88.2°–89.9°), a narrow near-vertical
# pitch-over band — standalone constants (not derived from ALPHA_LOWEST/HIGHEST).
PSO_COAST_LB = [  0.0,   50,   0.0,  1.54]
PSO_COAST_UB = [1000.0, 100.0, 100.0,  1.57]
# delta_tc          : coast phase duration                    [0, 2000] s
# delta_tr_pct      : Stage-2 burn as % of max propellant time [0, 100] %
# coast_start_pct   : coast start as % of Stage-2 burn time   [0, 100] %
# gamma_p           : pitch maneuver angle                    [1.54, 1.57] rad

# -------------- Penalty weights for coast PSO objective --------------
# No transversality term (no costates in this solver).
PSO_COAST_W_J         = 1.0     # burn-time term (J normalised by T_MAX_2)
PSO_COAST_W_ALTITUDE  = 100.0   # relative altitude error  (1% error → 1.0)
PSO_COAST_W_VELOCITY  = 100.0   # relative velocity error  (1% error → 1.0)
PSO_COAST_W_FPA       = 10.0    # FPA error in deg         (1 deg  → 10.0)
PSO_COAST_GAMMA_REF_DEG = 1.0   # FPA non-dimensionalisation reference [deg]

# -------------- Per-guidance EXTRA pso_coast decision variables --------------
# Under COAST_METHOD="pso_coast", cpr and exp_shooting expose extra decision
# variables appended after the 4 base vars [delta_tc, delta_tr_pct,
# coast_start_pct, gamma_p]:
#   cpr          -> + theta_dot  (constant pitch rate, rad/s, optimised by PSO;
#                                  initial pitch is set by gamma_p via the kick)
#   exp_shooting -> + a, b        (open-loop pitch law θ = a·exp(b·t_rel),
#                                  α = θ − γ; coefficients optimised by the PSO
#                                  instead of the per-arc fsolve shooting solve)
PSO_COAST_CPR_THETA_DOT_LB_DEG = 0.02   # [deg/s] lower bound for cpr pitch rate
PSO_COAST_CPR_THETA_DOT_UB_DEG = 1.0    # [deg/s] upper bound for cpr pitch rate
PSO_COAST_EXP_A_LB = 0.0     # exp_shooting a (initial commanded pitch ≈ [rad])
PSO_COAST_EXP_A_UB = 1.6
PSO_COAST_EXP_B_LB = -0.05   # exp_shooting b (pitch decay rate [1/s])
PSO_COAST_EXP_B_UB = 0.005

# -------------------------------------------------------------------
# 11c. Direct PSO   (only used when COAST_METHOD == "direct")
# -------------------------------------------------------------------

# -------------- PSO DIRECT algorithm settings --------------
PSO_DIRECT_N_PARTICLES     = 50
PSO_DIRECT_MAX_GENERATIONS = 100
PSO_DIRECT_C1              = 2.05
PSO_DIRECT_C2              = 2.05
PSO_DIRECT_OMEGA           = 0.7298
PSO_DIRECT_VMAX            = 0.5
PSO_DIRECT_SEED            = 42

# x = [gamma_p (rad), t_burn_pct (% of T_MAX_2)]
PSO_DIRECT_LB = [1.54,  50.0]
PSO_DIRECT_UB = [1.57, 100.0]

# -------------- Penalty weights for direct PSO objective --------------
# Same 4-term structure as PSO_COAST_W_* (no transversality term, no costates).
PSO_DIRECT_W_J           = 1.0     # burn-time term (J normalised by T_MAX_2)
PSO_DIRECT_W_ALTITUDE    = 100.0   # relative altitude error  (1% error -> 1.0)
PSO_DIRECT_W_VELOCITY    = 100.0   # relative velocity error  (1% error -> 1.0)
PSO_DIRECT_W_FPA         = 10.0    # FPA error in deg         (1 deg  -> 10.0)
PSO_DIRECT_GAMMA_REF_DEG = 1.0     # FPA non-dimensionalisation reference [deg]


# ===================================================================
# 12. FAST-RUN MODE
# ===================================================================
# If True, skips optimization and uses pre-determined optimal kick angles
RUN_FAST = False

# Optimal kick angles for each guidance mode (in radians)
# These values should be updated after running optimization for each mode
OPTIMAL_KICK_ANGLES = {
    "gravity_turn": -np.deg2rad(3.0),           # Update after optimization
    "linear_tangent": -np.deg2rad(3.0),         # Update after optimization
    "bilinear_tangent": -np.deg2rad(3.0),       # Update after optimization
    "apollo": -np.deg2rad(4.5),                  # Update after optimization
    "peg": -np.deg2rad(3.0),                     # Update after optimization
    "peg_new": -np.deg2rad(3.0),                 # Update after optimization
    "exp_shooting": -np.deg2rad(3.0),            # Update after optimization
}


# ===================================================================
# 13. SIMULATION OUTPUT & TIMING
# ===================================================================
TIME_STEP = 0.01                              # output sampling interval for t_eval; [s]
                                              # (integration itself is adaptive, max_step=1)
DURATION_AFTER_SIMULATION = 1000.               # duration of simulation after reaching desired orbit; [s]


# ===================================================================
# 14. DEBUGGING FLAGS
# ===================================================================
INTERRUPTS_PRINT = False
EVENTS_PRINT = True
