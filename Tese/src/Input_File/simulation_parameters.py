import numpy as np

# ===================================================
# General parameters
# ===================================================

# -------------- Gravity Turn --------------
TIME_TO_START_KICK = 7.5                        # time at which the instantaneous pitch-over is applied; [s]
DURATION_INITIAL_KICK = 45.                     # DEPRECATED — kept only so legacy plot scripts that reference
                                                # this symbol still import cleanly. The kick is now an
                                                # instantaneous gamma jump at TIME_TO_START_KICK and has no duration.

# -------------- Aerodynamics --------------
INCLUDE_LIFT = True                             # if True, include aerodynamic lift force in the EOM (F_L = q * C_L * A)

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
#   "simple_poly":  Simplified polynomial guidance (linear gamma transition)
#                   - Initial kick until atmosphere exit
#                   - Linear transition from current flight path angle to horizontal
#                   - Simple, stable, but not optimal
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
#                   - Used in Apollo missions, more accurate than simple_poly
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
#                   - Works with any GUIDANCE_START_MODE (after_kick or after_atmosphere_exit)
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
GUIDANCE_MODE = "indirect_pmp"  # Options: "gravity_turn", "simple_poly", "linear_tangent", "bilinear_tangent", "apollo", "cpr", "peg", "peg_new", "exp_shooting", "indirect_pmp"

# -------------- Guidance Start Timing --------------
# When should the guidance law activate after the kick maneuver?
#   "after_atmosphere_exit": Start guidance when the atmosphere exit condition is met (current default)
#   "after_kick": Start guidance immediately after the kick maneuver ends (earlier start)
GUIDANCE_START_MODE = "after_kick"   # Options: "after_atmosphere_exit", "after_kick"

# -------------- Polynomial Guidance Parameters --------------
# (Only used if GUIDANCE_MODE is "simple_poly" or "apollo")
GUIDANCE_UPDATE_RATE = 2                      # How often to recompute guidance coefficients [s]
APOLLO_FREEZE_THRESHOLD = 10.0                  # Time-to-go threshold to freeze Apollo coefficients [s]
                                                 # (prevents numerical instability as tgo->0)
APOLLO_THRUST_MAGNITUDE_CONTROL = False          # Enable thrust magnitude control for Apollo guidance
                                                 # If True: Apollo commands both thrust angle AND magnitude
                                                 # If False: Apollo only commands angle (fixed thrust)
APOLLO_TGO_METHOD = "propellant"                # Time-to-go estimation method for Apollo guidance:
                                                 #   "propellant": truncated rocket-equation t_go = T_BUP*(VG/Ve)*(1-0.5*VG/Ve)
                                                 #                  (physically accurate, accounts for remaining propellant)
                                                 #   "altitude":   simple t_go = altitude_remaining / v_radial
                                                 #                  (legacy, unreliable when gamma is small)

# -------------- Linear / Bilinear Tangent Steering Parameters --------------
# (Only used if GUIDANCE_MODE is "linear_tangent" or "bilinear_tangent")
GUIDANCE_COEFFICIENTS_FIXED = True           # If True, coefficients are computed once at guidance
                                              # start and held constant; only t_go varies each step.
                                              # If False (default), recomputed every GUIDANCE_UPDATE_RATE s.
GUIDANCE_TGO_FIXED = False                    # If True, t_go is computed once at guidance start and
                                              # held constant throughout guidance.
                                              # If False (default), recomputed every ODE step.
LTS_TGO_METHOD = "propellant"                  # t_go estimation method for linear/bilinear tangent laws:
                                              #   "altitude":   t_go = (target_alt - current_alt) / v_radial
                                              #                 (simple, default)
                                              #   "propellant": Apollo rocket-equation t_go
                                              #                 (stage 1: T_BUP1 + coast + stage-2 burn;
                                              #                  stage 2: T_BUP2*(1-exp(-VG/Ve)))

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
ALPHA_STEP = np.deg2rad(0.05)                 # step size for kick angle sweep; [rad]
MAX_ACCEPTED_BURN_TIME = 100.                    # maximum accepted burn time of delta-v; [s]

# -------------- Fast Run Mode --------------
# If True, skips optimization and uses pre-determined optimal kick angles
RUN_FAST = False

# Optimal kick angles for each guidance mode (in radians)
# These values should be updated after running optimization for each mode
OPTIMAL_KICK_ANGLES = {
    "gravity_turn": -np.deg2rad(3.0),           # Update after optimization
    "simple_poly": -np.deg2rad(3.0),            # Update after optimization
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
SS_THROTTLE = 1.0                               # Second Stage throttle 
INITIAL_KICK_ANGLE = - np.deg2rad(3.0)          # Initial kick angle [rad]


# ===================================================
# FOR SIMULATION
# ===================================================
TIME_STEP = 0.01                              # step size for integration; [s]
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
PSO_N_PARTICLES     = 350       # swarm size
PSO_MAX_GENERATIONS = 1250      # maximum number of generations
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
PENALTY_W_VELOCITY  = 200.0     # s2: relative velocity error (1% error -> 1.0)
PENALTY_W_FPA       = 10.0      # s3: FPA error in deg        (1 deg  -> 10.0)
PENALTY_W_TRANSVERS = 10.0       # s4: transversality (meaningful after ‖λ₀‖=1)
GAMMA_REF_DEG       = 1.0       # FPA non-dimensionalisation reference [deg]
