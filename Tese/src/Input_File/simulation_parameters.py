import numpy as np

# ===================================================
# General parameters
# ===================================================

# -------------- Gravity Turn --------------
TIME_TO_START_KICK = 7.5                        # time to start gravity turn; [s]
DURATION_INITIAL_KICK = 45.                     # duration of gravity turn; [s]

# -------------- Aerodynamics --------------
INCLUDE_LIFT = True                             # if True, include aerodynamic lift force in the EOM (F_L = q * C_L * A)

# -------------- Desired Orbit --------------
TARGET_ORBITAL_ALTITUDE = 500e3                             # altitude of desired orbit; [m]

# -------------- Earth Rotation (Optional) --------------
ENABLE_EARTH_ROTATION = True                # if True, include Earth rotation effects in azimuth/ECI calculations
LAUNCH_LATITUDE = 28.5                        # launch site latitude; [deg]
LAUNCH_LONGITUDE = -80.5                      # launch site longitude; [deg] (reserved for future launch window modeling)
TARGET_ORBIT_INCLINATION = 51.6               # desired final orbit inclination; [deg]
INCLUDE_PSEUDO_FORCES = True                # if True, include Coriolis and centrifugal accelerations in rotating-frame EOM
INCLUDE_CROSS_HEADING_PSEUDO_FORCE = True    # if True, include cross-heading Coriolis/centrifugal component in heading rate (requires INCLUDE_PSEUDO_FORCES and TRACK_HEADING_STATE)
COMPUTE_CROSS_HEADING_COUNTER_FORCE = True  # if True, compute & store the lateral force [N] needed to cancel the cross-heading drift (requires INCLUDE_PSEUDO_FORCES); plotted as kN vs time
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
#   "pso_paper":    Indirect optimal-control trajectory (Morgado, Marta, Gil 2022)
#                   - Particle Swarm Optimization (pyswarms) replaces the brute-force kick-angle search
#                   - Integrates 3 adjoint (costate) variables (lam_h, lam_V, lam_gamma) alongside the
#                     state during the free-flight phase (post-atmosphere-exit)
#                   - Steering from Pontryagin's Minimum Principle, paper eq. (34):
#                       alpha = atan2(-lam_gamma/V, -lam_V)
#                   - PSO design vars: 3 initial costates, pitch-over angle gamma_p,
#                     Stage-2 last-burn fraction, coast start fraction, coast duration
#                   - Coast happens MID-BURN inside Stage 2 (engine cut, then short terminal impulse)
#                   - Disables Coriolis/centrifugal pseudo-forces during the PSO run to match the
#                     paper's frame assumption (paper sec 3.2.2)
GUIDANCE_MODE = "pso_paper"  # Options: "gravity_turn", "simple_poly", "linear_tangent", "bilinear_tangent", "apollo", "cpr", "peg", "peg_new", "exp_shooting", "pso_paper"

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
    "pso_paper": -np.deg2rad(3.0),               # Filled in by PSO from gamma_p (initial pitch)
}

# ===================================================
# PSO Paper Mode Parameters
# ===================================================
# (Only used if GUIDANCE_MODE is "pso_paper")
# Reference: Morgado, Marta, Gil — "Multistage rocket preliminary design and trajectory
# optimization using a multidisciplinary approach", Structural and Multidisciplinary
# Optimization 65:192 (2022).  https://doi.org/10.1007/s00158-022-03285-y

# PSO swarm settings (paper sec 5.2 used 250 particles / 1000 iter for validation,
# sec 6 used 100 particles / 250 iter for design demonstration).
PSO_PAPER_POPULATION   = 300        # particles per swarm
PSO_PAPER_ITERATIONS   = 2000        # max iterations
PSO_PAPER_C1           = 2.05       # cognitive coefficient (paper sec 5.2)
PSO_PAPER_C2           = 2.05       # social coefficient — reduced from 2.05 to weaken pull toward global best and reduce premature convergence
PSO_PAPER_W            = 0.7298       # inertia weight — raised from 0.7298 to slow velocity decay and maintain exploration
PSO_PAPER_VMAX_NORM    = 0.5        # normalized max velocity

# Design-variable bounds (paper Tables 6, 9).
# Layout of the 7-element design vector x:
#   x[0] = lam_h0   (initial costate)
#   x[1] = lam_V0   (initial costate)
#   x[2] = lam_g0   (initial costate)
#   x[3] = gamma_p  (initial pitch after pitch-over, rad)
#   x[4] = Δt_c     (coast duration during Stage 2, s)
#   x[5] = coast_start_pct (fraction of total Stage-2 thrust time spent BEFORE coast)
#   x[6] = last_burn_pct   (fraction of m_prop_S2 / mdot to burn in Stage 2)
# Paper §4.1: vertical liftoff until t = PSO_PAPER_T_PITCHOVER, then an
# instantaneous pitch maneuver sets the flight path angle to gamma_p (= x[3]).
# The simulator's standard 45s triangular kick is bypassed in paper mode.
# Set to match TIME_TO_START_KICK so γ stays at 90° via the existing
# `time_kick_start is None` guard until pitch-over — no separate freeze logic
# needed, and by 7.5s the rocket is at ~40 m/s where the gravity-turn rate
# dγ/dt ∝ 1/V is well-behaved (it diverges if pitch-over happens at t=3s, v≈15 m/s).
PSO_PAPER_T_PITCHOVER           = 7.5                    # [s]

PSO_PAPER_LAMBDA_BOUNDS         = (-1.0, 1.0)            # initial costates [-]
PSO_PAPER_GAMMA_P_BOUNDS        = (1.3, 1.57)            # initial pitch [rad] (74.5°–89.9°) — widened from (1.54, 1.57) for Falcon 9 class
PSO_PAPER_COAST_DURATION_BOUNDS = (0.0, 1500.0)          # Δt_c [s] — tightened for 500 km targets (paper sec 6 used (500, 3000) for Electron going much higher)
PSO_PAPER_COAST_START_PCT       = (0.0, 1.0)             # fraction of Stage-2 thrust time before coast
PSO_PAPER_LAST_BURN_PCT         = (0.70, 0.95)           # fraction of Stage-2 max-burn time (paper Table 11; 5% reserve case)

# Penalty weights (paper eq. 39; the paper does not publish s_c values).
PSO_PAPER_PENALTY_ALT   = 1.0e3
PSO_PAPER_PENALTY_VEL   = 1.0e3
PSO_PAPER_PENALTY_GAMMA = 1.0e3     # gamma is now normalised to [0,1] — same scale as alt/vel
PSO_PAPER_PENALTY_HAM   = 1.0e3     # transversality residual (H_f_last + H_f_coast − H_0_last)
PSO_PAPER_PENALTY_HARD  = 1.0e6     # ground impact / NaN — softened from 1e20: still ~100× worse than typical orbit cost, but gives PSO usable gradients near the crash boundary instead of an infinite wall

# Early-stop convergence (paper sec 6.1, paragraph after Table 13:
#   "Early PSO convergence is assumed if the PSO algorithm finds the optimal trajectory
#    within 0.1% deviation for the insertion altitude and velocity, and 0.0175 rad
#    deviation for the final flight path angle.")
PSO_PAPER_EARLY_STOP_ALT_TOL   = 1.0e-3       # fractional altitude error
PSO_PAPER_EARLY_STOP_VEL_TOL   = 1.0e-3       # fractional velocity error
PSO_PAPER_EARLY_STOP_GAMMA_TOL = 0.0175       # absolute gamma error [rad] ≈ 1°

# Pseudo-forces are forced OFF during the PSO run (paper sec 3.2.2 neglects Coriolis
# and centrifugal accelerations during trajectory simulation, citing Reilly 1979).
# The solver saves/restores INCLUDE_PSEUDO_FORCES around the swarm. Earth rotation
# can stay enabled — its initial velocity boost (paper eq. 22) is preserved.
PSO_PAPER_FORCE_DISABLE_PSEUDO = True

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
