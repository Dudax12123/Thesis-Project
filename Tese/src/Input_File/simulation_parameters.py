import numpy as np

# ===================================================
# General parameters
# ===================================================

# -------------- Gravity Turn --------------
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

# -------------- Aerodynamics --------------
INCLUDE_LIFT = False                             # if True, include aerodynamic lift force in the EOM (F_L = q * C_L * A)

# -------------- Desired Orbit --------------
TARGET_ORBITAL_ALTITUDE = 500e3                             # altitude of desired orbit; [m]

# -------------- Earth Rotation (Optional) --------------
ENABLE_EARTH_ROTATION = True                # if True, include Earth rotation effects in azimuth/ECI calculations
LAUNCH_LATITUDE = 28.5                        # launch site latitude; [deg]
LAUNCH_LONGITUDE = -80.5                      # launch site longitude; [deg] (reserved for future launch window modeling)
TARGET_ORBIT_INCLINATION = 51.6               # desired final orbit inclination; [deg]
INCLUDE_PSEUDO_FORCES = False                # if True, include Coriolis and centrifugal accelerations in rotating-frame EOM
INCLUDE_CROSS_HEADING_PSEUDO_FORCE = False    # if True, include cross-heading Coriolis/centrifugal component in heading rate (requires INCLUDE_PSEUDO_FORCES and TRACK_HEADING_STATE)
COMPUTE_CROSS_HEADING_COUNTER_FORCE = False  # if True, compute & store the lateral force [N] needed to cancel the cross-heading drift (requires INCLUDE_PSEUDO_FORCES); plotted as kN vs time
TRACK_HEADING_STATE = False                    # if True, propagate heading as an additional state when Earth rotation is enabled

# -------------- Azimuth / Inclination Mode --------------
# All three modes derive the initial launch azimuth from the spherical-geometry formula:
#   sin(beta) = cos(i_target) / cos(phi_launch)
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

# -------------- Guidance Mode Selection --------------
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
GUIDANCE_MODE = "apollo"  # Options: "gravity_turn", "linear_tangent", "bilinear_tangent", "apollo", "cpr", "peg", "peg_new", "exp_shooting", "indirect_pmp"

# -------------- Polynomial Guidance Parameters --------------
# (GUIDANCE_UPDATE_RATE is also used by linear_tangent/bilinear_tangent;
#  APOLLO_FREEZE_THRESHOLD is also the freeze threshold for peg/peg_new.)
GUIDANCE_UPDATE_RATE = 2                      # How often to recompute guidance coefficients [s]
APOLLO_FREEZE_THRESHOLD = 10.0                  # Time-to-go threshold to freeze Apollo coefficients [s]
                                                 # (prevents numerical instability as tgo->0)
APOLLO_THRUST_MAGNITUDE_CONTROL = False          # Enable thrust magnitude control for Apollo guidance
                                                 # If True: Apollo commands both thrust angle AND magnitude
                                                 # If False: Apollo only commands angle (fixed thrust)
# -------------- Linear / Bilinear Tangent Steering Parameters --------------
# (Only used if GUIDANCE_MODE is "linear_tangent" or "bilinear_tangent")
GUIDANCE_COEFFICIENTS_FIXED = True           # If True, coefficients are computed once at guidance
                                              # start and held constant; only t_go varies each step.
                                              # If False (default), recomputed every GUIDANCE_UPDATE_RATE s.
GUIDANCE_TGO_FIXED = False                    # If True, t_go is computed once at guidance start and
                                              # held constant throughout guidance.
                                              # If False (default), recomputed every ODE step.

# -------------- Constant Pitch Rate (CPR) Guidance Parameters --------------
# (Only used if GUIDANCE_MODE is "cpr")
CPR_THETA_DOT_MODE = "manual"       # How to determine the constant pitch rate:
                                  #   "tgo":    θ_dot = (90°) / t_go  where t_go is from the
                                  #             Apollo propellant-based rocket-equation estimate
                                  #   "manual": use CPR_THETA_DOT directly (duration derived)
CPR_THETA_DOT = 0.4              # Between {0.1, 0.5}[deg/s] manual pitch rate (only used when CPR_THETA_DOT_MODE = "manual")
                                  # Guidance duration = 90° / CPR_THETA_DOT

# -------------- PEG Guidance Parameters --------------
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
THRUST_1_MODE = "sea_level"                     # Options: "sea_level", "vacuum", "average", "linear"
THRUST_1_LINEAR_UPDATE_RATE = 5.0               # [s] step interval for linear ramp (only used when THRUST_1_MODE = "linear")

# -------------- Atmosphere Exit / Guidance Start Marker --------------
# Choose how to detect when the rocket exits the atmosphere and guidance should start:
#   "altitude":         Use altitude threshold (traditional method)
#   "dynamic_pressure": Use dynamic pressure threshold (more physically meaningful)
#   "aerothermal_flux": Use aerothermal flux threshold (Phi = 0.5*rho*v^3)
ATMOSPHERE_EXIT_METHOD = "dynamic_pressure"             # Options: "altitude", "dynamic_pressure", "aerothermal_flux"
ALT_NO_ATMOSPHERE = 65e3                        # altitude threshold for atmosphere exit; [m]
                                                 # (only used if ATMOSPHERE_EXIT_METHOD = "altitude")
DYNAMIC_PRESSURE_THRESHOLD = 1000.0             # dynamic pressure threshold [Pa]
                                                 # (only used if ATMOSPHERE_EXIT_METHOD = "dynamic_pressure")
                                                 # Typical value: 1000 Pa (fairly low, indicating thin atmosphere)
AEROTHERMAL_FLUX_THRESHOLD = 1135.0             # aerothermal flux threshold [W/m^2]
                                                 # (only used if ATMOSPHERE_EXIT_METHOD = "aerothermal_flux")
                                                 # Phi = 0.5*rho*v^3; negligible heating below this value

# -------------- Optimization --------------
ALPHA_LOWEST = -np.deg2rad(5.5)                  # lowest possible kick angle to be tested; [rad]
ALPHA_HIGHEST = -np.deg2rad(2.5)                # highest possible kick angle to be tested; [rad]~
MAX_ACCEPTED_BURN_TIME = 100.                    # maximum accepted burn time of delta-v; [s]
# apogee_check: a kick angle is accepted only if its achieved apogee is within this
# fraction of the target radius. Tight (0.0002 ≈ 1.4 km) now that the apogee
# interrupt and the SECO conversion use the same (launch) latitude.
APOGEE_MATCH_TOL_FRAC = 0.0002                   # apogee match tolerance (fraction of r_target)

# -------------- Fast Run Mode --------------
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

# ===================================================
# Single Run specific parameters
# ===================================================
INITIAL_KICK_ANGLE = - np.deg2rad(3.0)          # Initial kick angle [rad]


# ===================================================
# FOR SIMULATION
# ===================================================
TIME_STEP = 0.01                              # output sampling interval for t_eval; [s]
                                              # (integration itself is adaptive, max_step=1)
DURATION_AFTER_SIMULATION = 1000.               # duration of simulation after reaching desired orbit; [s]


# ===================================================
# FOR DEBUGGING
# ===================================================
INTERRUPTS_PRINT = False
EVENTS_PRINT = True


# ===================================================
# INDIRECT PMP / PSO PARAMETERS
# (only used when GUIDANCE_MODE = "indirect_pmp")
# ===================================================

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


# ===================================================
# COAST METHOD SELECTION
# (applies to all guidance modes except "indirect_pmp")
# ===================================================

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
#   "direct"       : Continuous single Stage-2 burn to DIRECT orbit insertion —
#                    no coast, no circularisation burn. The selected guidance law
#                    steers the burn, which is cut (MECO) the instant the inertial
#                    velocity reaches circular √(μ/r_target) — the standard orbit-
#                    insertion trigger, so the engine never over-burns past
#                    insertion. Whether the flight-path angle and altitude ALSO
#                    landed within tolerance (a "clean" insertion) is then reported
#                    together with the achieved orbit (eccentricity, apo/peri). If
#                    Stage-2 propellant depletes before circular velocity is
#                    reached, the achieved under-speed orbit is reported instead.
#                    Intended for the direct-insertion laws (peg, peg_new, apollo).
COAST_METHOD = "direct"   # Options: "apogee_check", "pso_coast", "direct"

# -------------- Direct-insertion tolerances --------------
# (only used when COAST_METHOD == "direct") The Stage-2 burn is cut (MECO) when the
# inertial velocity reaches circular √(μ/r_target). At that cutoff the insertion is
# graded "clean" only if the velocity, flight-path-angle AND altitude errors are all
# within the tolerances below; otherwise the achieved orbit is reported as-is and the
# kick optimiser converges to the smallest combined (tolerance-normalised) box error.
DIRECT_INSERTION_VELOCITY_TOL_MS  = 10.0   # |v_inertial − √(μ/r_target)| [m/s]
DIRECT_INSERTION_FPA_TOL_DEG      = 0.5    # |flight-path angle|          [deg]
DIRECT_INSERTION_ALTITUDE_TOL_KM  = 5.0    # |altitude − target|         [km]

# -------------- Direct-insertion optimisation mode --------------
#   "brute_force" : existing 1-variable kick-angle sweep; burn runs until
#                    interrupt_direct_meco fires or propellant depletes.
#   "pso"         : 2-variable PSO (direct_pso_solver) jointly optimises
#                    gamma_p (kick angle) AND Stage-2 burn duration, targeting
#                    the DIRECT_INSERTION_* box directly.
DIRECT_OPTIMIZATION_MODE = "pso"   # Options: "brute_force", "pso"

# -------------- PSO DIRECT algorithm settings --------------
# (only used when COAST_METHOD == "direct" and DIRECT_OPTIMIZATION_MODE == "pso")
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

# -------------- PSO COAST algorithm settings --------------
# (only used when COAST_METHOD == "pso_coast")
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
