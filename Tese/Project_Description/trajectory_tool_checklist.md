# Trajectory Simulation & Optimization Checklist

Columns: **Var** (suggested config key), **Units**, **Type** (S = sweep, O = optimize), **Bounds/Options**, **Notes**.

## Vehicle & Propulsion
| Parameter | Var | Units | Type | Bounds/Options | Notes |
|---|---|---:|:--:|---|---|
| Stage dry mass (per stage) | `m_dry[i]` | kg | S/O | mission-specific | Major payload driver |
| Propellant mass (per stage) | `m_prop[i]` | kg | S/O | mission-specific | Use mass fraction if preferred |
| Vacuum Isp | `Isp_vac[i]` | s | S | ±2–5% | Include dispersion for robustness |
| Sea-level Isp | `Isp_sl[i]` | s | S | ±2–5% | Impacts liftoff T/W |
| Max thrust | `T_max[i]` | kN | S/O | mission-specific | With throttle limits |
| Throttle profile | `throttle_sched[i]` | — | O | piecewise const. | Decision vector over time |
| Gimbal limit | `gimbal_max[i]` | deg | S | 3–7 | Controls α authority |
| Gimbal rate limit | `gimbal_rate[i]` | deg/s | S | 5–20 | Avoids saturation |
| Restarts (upper stage) | `restarts[i]` | — | S | 0,1,2 | Discrete choice |
| Fairing mass | `m_fair` | kg | S | mission-specific | Trade with size/drag |
| Fairing jettison rule | `fairing_rule` | — | S/O | `q<thresh` or `h>Hmin` | Pick rule + threshold |

## Aerodynamics & Loads
| Parameter | Var | Units | Type | Bounds/Options | Notes |
|---|---|---:|:--:|---|---|
| Cd multiplier | `Cd_bias` | — | S | 0.9–1.1 | Sensitivity/uncertainty |
| Ref. area | `S_ref` | m² | S | mission-specific | Affects drag |
| Max dynamic pressure | `q_max` | kPa | S/O | 25–60 | Hard constraint |
| Max angle of attack | `alpha_max` | deg | S/O | 3–8 | In dense air |
| q·α cap | `qa_max` | kPa·deg | S | project-specific | Optional aero limit |
| Heat rate proxy cap | `qdot_max` | (arb.) | S | project-specific | If modeled |

## Guidance / Control Law — Common
| Parameter | Var | Units | Type | Bounds/Options | Notes |
|---|---|---:|:--:|---|---|
| Guidance cycle time | `guidance_dt` | s | S | 0.05–0.2 | 5–20 Hz |
| Pitch/azimuth filter gain | `ctrl_gain` | — | S | 0.5–1.5 | Stability vs lag |
| AoA limiter on | `alpha_limit_on` | — | S | true/false | Recommended true |

### Gravity-Turn (GT)
| Parameter | Var | Units | Type | Bounds/Options | Notes |
|---|---|---:|:--:|---|---|
| Pitch-kick angle | `gt_kick_deg` | deg | O | 2–10 | Key GT knob |
| Kick onset (alt or V) | `gt_kick_onset` | m or m/s | O | 0.5–3 km or 80–150 m/s | Choose one |
| Kick duration | `gt_kick_dur` | s | O | 2–15 | Smooth turn-in |
| GT start velocity | `gt_start_V` | m/s | S | 60–150 | Switch from vertical rise |

### Polynomial Law
| Parameter | Var | Units | Type | Bounds/Options | Notes |
|---|---|---:|:--:|---|---|
| Order | `poly_order` | — | S | 3–5 | Discrete |
| Coefficients | `poly_coeff[k]` | — | O | bounded | Or set via BCs |
| Segment breakpoints | `poly_t[k]` | s | O | monotonic | If piecewise |
| Slope/curvature caps | `poly_slope_max`,`poly_curv_max` | — | S | project-specific | Control α & q |

### Tangent / Bi-Tangent
| Parameter | Var | Units | Type | Bounds/Options | Notes |
|---|---|---:|:--:|---|---|
| Tangency angle(s) | `tan_ang[k]` | deg | O | −10–+10 | W.r.t. target velocity |
| Switch criteria | `tan_switch` | m/s or s | O | bounded | V/alt/time |
| Curvature cap | `tan_kappa_max` | 1/m | S | project-specific | Turn-rate limit |
| Terminal weights | `term_w_pos`, `term_w_vel` | — | S | tune | For tracking terminal state |

## Mission Geometry & Initial Conditions
| Parameter | Var | Units | Type | Bounds/Options | Notes |
|---|---|---:|:--:|---|---|
| Launch latitude | `lat_deg` | deg | S | site-fixed | Driver of azimuth |
| Launch longitude | `lon_deg` | deg | S | site-fixed | For rotation geometry |
| Launch azimuth | `az0_deg` | deg | O | 60–120 (typ.) | Optimize to target inc |
| Time of day | `t_launch` | UTC | S/O | window | For RAAN targeting |
| Pad elevation | `h_pad` | m | S | measured | Minor effect |
| Wind profile | `wind_prof` | — | S | samples/Monte Carlo | Robustness testing |

## Staging & Events
| Parameter | Var | Units | Type | Bounds/Options | Notes |
|---|---|---:|:--:|---|---|
| Stage sep condition | `sep_cond[i]` | — | S | burnout/time | Logic flag |
| Upper-stage ignition delay | `ign_delay` | s | S | 0–300 | For coast arcs |
| Coast duration | `coast_dur` | s | O | 0–1200 | Before final burn |
| Fairing jettison threshold | `fair_q_thr` or `fair_h_min` | kPa / km | S/O | q: 5–10; h: 50–120 | Pick rule above |

## Target Orbit & Terminal Constraints
| Parameter | Var | Units | Type | Bounds/Options | Notes |
|---|---|---:|:--:|---|---|
| Target altitude | `h_circ` | km | S | 200–800 | Or ap/peri pair |
| Target inclination | `i_tgt` | deg | S | site-dependent | Inclination band |
| RAAN (if needed) | `RAAN_tgt` | deg | S | window | Requires time-of-day |
| Perigee/apogee limits | `hp_min`,`ha_max` | km | S | mission-specific | Acceptance window |
| Insertion γ bound | `gamma_max` | deg | S | ≤0.5 | At MECO/SECO |
| Node window | `LAN_window` | deg | S | ±Δ | For phasing missions |

## Environment & Models
| Parameter | Var | Units | Type | Bounds/Options | Notes |
|---|---|---:|:--:|---|---|
| Atmosphere model | `atm_model` | — | S | Std/Seasonal | Toggle |
| Density bias | `rho_bias` | — | S | 0.9–1.1 | Sensitivity |
| Gravity model | `grav_model` | — | S | point/J2 | Upper-stage targeting |
| Drag fidelity | `drag_mode` | — | S | const-Cd / table | Swap easily |
| Monte Carlo seeds | `mc_seed` | — | S | integer | For repeatability |

## Discrete Design Choices
| Parameter | Var | Units | Type | Bounds/Options | Notes |
|---|---|---:|:--:|---|---|
| Poly segments | `poly_nseg` | — | S | 1–3 | Mixed-integer |
| Bi-tangent segments | `tan_nseg` | — | S | 1–2 | Mixed-integer |
| Fairing option | `fair_opt` | — | S | A/B | Mass/drag variants |
| Upper-stage engine | `us_engine_opt` | — | S | options list | Isp/thrust trade |
| Launch site | `site_opt` | — | S | sites list | Latitude change |

## Objectives & Metrics (record every run)
| Metric | Var | Units | Type | Target |
|---|---|---:|:--:|---|
| Payload to orbit | `payload_kg` | kg | — | maximize |
| Ideal Δv | `dV_ideal` | m/s | — | report |
| Losses (gravity/drag/steering) | `dV_grav`,`dV_drag`,`dV_str` | m/s | — | minimize |
| Max-Q / Max-α / Max-g | `q_peak`,`alpha_peak`,`g_peak` | kPa/deg/g | — | ≤ limits |
| Time to orbit | `t_orbit` | s | — | report |
| Insertion errors | `apo_err`,`peri_err`,`inc_err` | km/deg | — | ≤ tolerances |
| Control activity | `gimbal_sat_time` | s | — | minimize |
| Compute cost | `iter_count`,`wall_time` | —/s | — | compare methods |
