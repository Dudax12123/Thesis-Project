# Thesis flags: backlog

Started 2026-10-01. Items found while fixing the thesis, parked here so they do not interrupt the user's own edit list. Review them together once that list is done. New flags are appended. Thesis line numbers are as of Overleaf `46062bb`.

## Ch. 6 walkthrough v5, 2026-10-07 (final results; session 67a7e9e8)

Page: https://claude.ai/artifact/2xqwupr6w8A68RqJ2oJq7e, version 5. Numbers, figures and tables come from `Output/results_matrix_chapter6_20261007/`, read by the session's scratchpad `facts_v5.py` and rendered into its `render_v5/`. Source in `ch6walk/content_v5.py`. The user's 31 stored answers are reviewed: 30 kept, and S4-T4 cleared, because it applied to the reference-tracking case.

- **Plot library, uncommitted** (`Tese/src/Plots/results_figures/`; full suite 294 passed, 5 skipped):
  - `REPORTED_CASES` drops show_ref_track(_apollo), leaving 17 cases.
  - `peg_waypoint` (figure and table) uses the segmented cases as the waypoint rows.
  - `showcase_laws` gains the gravity-turn and Apollo rows.
  - The showcase α range is taken from the steered samples only. Before, the linear tangent's +13° first burn was clipped off the panel.
  - The `loss_accumulation` legend is moved. This closes the round-2 C13 (the MECO label under the legend). Note that two flags carry the id C13: the other is the `tab:architecture_cost` tail-improvement one, under Code repository.
  - `apollo_waypoint` and `segmented_results` are built on request only.
- **T36. Removing the reference-tracking cases reaches Ch. 5** (page item X11; proposed, not approved).
  - "nineteen" appears at l.410, 479, 535 and 542.
  - `tab:case_matrix` carries the "Coast-start waypoint" group and the `show_ref_track_apollo` row.
  - The "coast-start waypoint" paragraph (l.602–613) should shrink to the waypoint's definition.
  - l.594 and l.626–628 refer to the waypoint cases.
  - Ch. 6: delete §6.4 and the Apollo-waypoint subsection with their floats, and move the segmented section into §6.4's place.
  - **Done in Ch. 6, uncommitted (2026-10-07):** the intro, the X1 paragraph and §6.1 are written in. X11 is applied: §6.4 deleted, the segmented section in its place, the Apollo-waypoint floats out. fig:reference_card (now `results_reference_profiles`), tab:reference_results, fig:peg_waypoint, tab:peg_waypoint and fig:segmented_handoff are re-rendered and recaptioned (part of T33).
  - **Ch. 5: done, exactly X11's list (user, 2026-10-07).**
    - The four counts; at l.479 the same sentence now reads "one group of which changes".
    - tab:case_matrix rows; l.594; the waypoint paragraph reduced to its definition.
    - In the segmented paragraph, the "Unlike the waypoint cases above" clause and the last sentence.
    - Sweep clean. The user chose to keep the unused files.
  - **Fixed afterwards (user, same day; thesis uncommitted):** the three passages below now leave the removed cases out. The code layout reads "two further solvers". One mention is left: the §6.6.2 `\discuss` note ("against the waypoint cases, which search nothing"), which goes when S7-D1 is written in.
  - **Were still naming the removed cases, outside X11's list:**
    - Ch. 5 l.172–179: "three further solvers … reference\_track\_solver.py flies the waypoint cases".
    - Ch. 3 l.365: "The waypoint cases … search nothing".
    - App. A l.25, the tab:full_results caption: "the waypoint cases search nothing".
- **T26, T29, T30: moot.** The final references coast 1 823–1 987 s inside the shared 2 000 s bound. None stops at it, and Ch. 3's [0, 2 000] s is right again.
- **T37. `gt_apogee`'s budget is still cut at SECO** (T27, C16). The page recommends a caption note on its `tab:loss_budget` row for now. The drafts keep its gravity loss (1 180 m/s) out of the group ranges.
- **K13. Check in Overleaf:** `tab:showcase_laws` with six rows; `tab:peg_waypoint` with four rows and 11 columns (K11).
- **§6.2–6.4 written in, 2026-10-07, thesis uncommitted.** S2-D1/D2, S3-D1/D2 and S4-D1/D2 replace their `\discuss` notes. Two new captions (approved in the chat): `fig:gt_baseline_card` and `fig:gt_axes` (2 × 2). `fig:peg_atmosphere` is out of the text; its PNG is kept. Re-rendered from the results set: `results_gt_baseline_card.png`, `results_gt_axes.png`, `gt_results`, `peg_results`, `loss_budget` and `caption_values` (the last two because §6.2–6.3 quote `tab:loss_budget`). T32 is done: `jaggers1974peg` is in the bibliography. The plot library change, uncommitted: `run_card` (b) drops the rotated "Coast" label, which sat on the speed curve. The shading still marks the coast.
  - Still stale in the thesis, for §6.5–6.6 and App. A: `showcase_laws`, `full_results`, `architecture_cost`, and the six §6.6/App. figures.
  - Left out: S2-D1's optional out-of-plane sentence (S2-T6). The approved S2-D2 already says the same thing.
- **T38. §6.2.4 credits part of the apogee check's margin to its impulsive insertion. That is the user's wording, chosen 2026-10-07 against the data.**
  - At apogee γ = 0 by construction, so the 93 m/s impulse corrects speed only. A finite burn of the same Δv would last 2.7 s (C20).
  - The swarm gravity turn's own 18.2 s final burn loses 0.1 m/s of gravity and none of steering (α = 0), so there is little for the impulse to save. The 1.31 t comes from the lower staging and cut-off (gravity loss to SECO 1 180 against 1 768 m/s; T37).
  - To make the sentence consistent, §6.2.1 now gives only the facts of the apogee check. S2-D1's "an idealization, but a small one … the margin comes from the trajectory instead" is out.
  - An examiner could ask for the number. Offer the bounded wording again at the final read.
  - **Update 2026-10-07, after the S7-V1 extension:** `tab:loss_budget` now shows the apogee check's gravity loss to apogee, 1 576 m/s, against 1 768 for the swarm flight. That 192 m/s is worth 1.2–1.5 t at 6–8 kg per m/s, i.e. the whole margin. §6.6.1 ("these two losses account for almost every shortfall") and S7-D2 ("keeps 1.31 t more … by coasting for 2 721 s") now sit beside the §6.2.4 sentence.
- **§6.5–6.6 and App. A written in, 2026-10-07, thesis uncommitted.** S5-D1/D2 and S7-D1/D2/D3 replace their `\discuss` notes. No `\discuss` is left in Ch. 6.
  - Captions: the S5-F2 caption on `tab:showcase_laws`, and a new `tab:loss_budget` caption (approved in the chat).
  - S7-D1 changes, approved in the chat:
    - The bands sentence is corrected: the apogee check is inside the gravity band and below the drag band.
    - The gravity-loss ranges include the apogee check.
    - `(Figure~\ref{fig:loss_budget}b)` becomes `fig:loss_budget`; that figure has one panel.
    - A pointer to `fig:loss_accumulation` is added.
    - The accuracy sentence points to the kept App. figure.
  - User decisions, 2026-10-07:
    - S7-F5: `fig:accuracy_vs_propellant` is kept, against X4/X6.
    - S7-F7: `fig:solve_cost` is out (its PNG is kept).
    - S7-V1: extend the window now.
  - Re-rendered: every remaining stale float. All Ch. 6 floats now come from the results set.
- **T27, C16, T37: closed by S7-V1 (code uncommitted).**
  - `run_record.apogee_impulse_index` runs `gt_apogee`'s budget window through the coast to the apoapsis and adds the impulse (ideal Δv; steering 2|Δv| if retro).
  - `rocket_ascent._finish_single_burn_rotating` closes the thrust log to zero at SECO. The archived record held 934 kN through the whole coast. Output-only: trajectories were never affected. Found while doing this.
  - Two regression tests in `test_apogee_check_coast.py` fail on the old code. Full suite: 296 passed, 5 skipped.
  - `gt_apogee` re-flown into the results set, trajectory bit-identical:
    - gravity loss 1 179.6 → 1 575.8 m/s, ideal ΔV 8 754.1 → 8 847.0 m/s, residual +17.4 → −4.8 m/s;
    - the batch wall clock (127.3 s) is kept, with a `repairs` entry; the README records it;
    - the backup is `Output/results_matrix_chapter6_20261007_pre_budget_window/`;
    - the manifest says `8f8bdd5-dirty`. Re-fly once the code is committed, for a clean hash (about a minute).

## Launch-site batch, 2026-10-06 (session 8d81b701; launched from worktree `f350460`)

User decisions (2026-10-06): the PMP references are re-swarmed at 250×1000 from seed 3 only, then refined. The four reference-dependent cases wait for the new pmp_baseline extremal. The 750×1500 runs stay stored.

- **T34. The thesis gives the references as 750×1500 swarms from five seeds** (Overleaf `8198fa1`). The following change on adoption; this supersedes T23's premise. **ON HOLD: no thesis changes for now.**
  - Ch. 3 `tab:pso_settings`: the Indirect column (750/1500) and the seed note at l.568–569.
  - Ch. 3 l.712–713: "refined from the swarms of several seeds, and the best extremal kept".
  - Ch. 5 l.590–593: "750 particles over 1500 generations from five random seeds … seed 3 is kept".
  - The search cost: `PMP_*_SEARCH` in `run_results_matrix.py`, and any quoted swarm cost.

User decision (2026-10-07): the references are the three 750×1500 seed-3 swarms refined with the coast up to 2 000 s, first pass only. The four dependent cases fly against `Output/pmp_reference_launchsite_750x1500_c2000_20261007.npz` with `--set PSO_UB=[…, 2000.0, …]`. Earlier flights are kept under `results_matrix_launchsite_20261006/_set_aside_*`.

- **T35. The grid + Brent search is described only for the law-terminated burn** (found 2026-10-07; ON HOLD with the report).
  - Ch. 3 l.645–649 says only that the 500-point grid "is refined by Brent's method". The one Brent's method the thesis describes, at l.321, is the root finder: bisection plus inverse quadratic interpolation. The search uses Brent's minimiser instead: golden section plus parabolic interpolation, `scipy.optimize.minimize_scalar`, bounded. Same citation, `brent1973algorithms`.
  - If `gt_direct` moves to `"grid_brent"` (measured 2026-10-07: same point as the swarm, J′ 42.0733 against 42.0730, 26 067 flights instead of 250 000), the nested form also needs a sentence: a law with no cutoff of its own searches the burn time per γ_p by its own 21-point grid + Brent. Ch. 5 l.596–600 would then cover two cases.

- **K12. The reference-tracking flights do not hold the reference's coast.**
  - On the 2 000 s pmp_baseline reference (22 649.0 kg; coast 1 823 s, from 135.4 km):
    - `show_ref_track` (peg_new) reaches 508 × 493 km with 17 035.5 kg, 5.6 t below the reference.
    - `show_ref_track_apollo` ends at 498 × 33 km, so it does not reach orbit. Its 22 802.3 kg is not comparable.
  - On the 250×1000 2 000 s reference (coast 1 934 s) the same flights gave 16 517.0 kg and a periapsis of 85.9 km.
  - Cause (measured 2026-10-07): the tracking flights coast for the reference's fixed Δt_c, and arc 1 misses γ by +0.06–0.07°.
    - From a low coast start (135 km, e ≈ 0.035), that miss moves the apoapsis by roughly 6 km.
    - peg_new reaches its final ignition at 505.8 km, still climbing (γ +0.13°). It must descend to 500 km while circularising: 24.3 s of burn against the reference's 4.0 s.
    - On the 3 500 s reference the same miss left it 2 km low (497.8 km), which costs little: 7.9 s, 21 661.5 kg.
    - apollo ends its final burn on its own t_go estimate (C-flag above, l.385): 3.4 s, 133 m/s short.
  - The 2026-09-26 flights (21 859 / 22 215 kg) had final burns of 1.4 s and 0.2 s. Their target was the apoapsis of an ellipse, so arriving near apoapsis was enough.
  - Candidate fixes: end the coast at the flown apoapsis rather than after Δt_c; end apollo's final burn on the orbit conditions.
  - Tried 2026-10-07 on peg_new, against the 750×1500 2 000 s reference. Uncommitted: `REFERENCE_TRACK_COAST_MODE`, default `"duration"`, tests pass.
    - `"apoapsis"` (coast to γ = 0): the apoapsis is 506.2 km, so arc 3 still burns 24.6 s. Result 16 956.1 kg, no gain.
    - Coast ending when the flight first climbs through 500 km (scratchpad `ref_track_target_alt.py`, swaps the event): arc 3 burns 4.6 s. Result 22 425.7 kg, 504.1 × 495.7 km, 223 kg below the reference.
  - **Adopted for `show_ref_track` (user, 2026-10-07; uncommitted).** `REFERENCE_TRACK_COAST_MODE = "target_altitude"`, set as a case override; the `"apoapsis"` trial mode was removed. Re-flown into the launch-site batch: 22 425.7 kg. Full suite: 294 passed, 5 skipped.
    - Thesis impact, ON HOLD with the report: Ch. 5's reference-tracking description says the case flies the reference's coast length. §6.7 must state the coast rule.
    - `show_ref_track_apollo` flies it too (user, 2026-10-07): 22 803.2 kg, 497.8 × 33.2 km, still not in orbit.
      - Its apoapsis (497.8 km) is below the target, so its coast ends there.
      - Its final burn gets t_go = 3.37 s from the rocket equation. That is below the 10 s freeze, so the coefficients are frozen at ignition.
      - Steering the vertical channel first, it needs ~1 200 m/s² to climb 2.3 km and level off in 3.4 s, against ~34 m/s² of thrust. It flies α ≈ +90° then −90°, leaving no horizontal thrust, and ends 133 m/s short.
      - With `TGO_ESTIMATOR = "peg_new"` (diagnostic, `Output/ref_track_apollo_tgo_peg_new_trial_20261007/`): t_go 8.6 s, still frozen; 498.5 × 175.3 km, 21 377.1 kg. A longer t_go alone does not fix it.
      - Open: apollo's final burn cannot both climb 2.2 km and add 135 m/s along-track in a short burn (see also C8).
  - Decide whether §6.7 reports these flights as they are, or whether the tracking flights get one of these fixes.

## `gt_baseline` refinement trial, 2026-10-06 (session 8d81b701; nothing committed)

Scripts in that session's scratchpad: `refine_law_case.py`, `scan_gt_kick.py`, `scan_gt_coast.py`, `archive_gt_c3500.py`. Archives in `Tese/src/Output/law_refine_trial_20261006/`; figures in `Tese/src/Output_Plots/comparisons/gt_baseline_refine_trial/`.

- **C18. The law swarms stop far from their own optimum.** **ON HOLD (user, 2026-10-06): no further law refinement for now.**
  - The archived `gt_baseline` point (250×1000 swarm, 20 687.5 kg), refined within the swarm's own box (coast ≤ 2 000 s), reaches 22 069.8 kg: +1 382 kg in 53 s.
    - The coast lengthens from 423 s to the 2 000 s bound and starts at 204 km instead of 328 km. The kick goes from 1.523° to 1.722°, and the last burn is 2.9 s.
    - The orbit closes exactly (J′ 0.761846), and the dense re-run scores the same J′.
  - The refinement closes the orbit for each kick (least squares on the three insertion conditions) and searches the kick alone. On the J′ penalty, Nelder–Mead stalls after +173 kg. That was the method behind the 2026-10-06 law predictions, so they predict what a re-swarm finds, not the laws' optima.
  - On the archived row's commit (`c9a7ea3`) the same refinement gains +844 kg (coast 1 134 s). The shortfall is the swarm's convergence.
  - Consequence for Ch. 6: the references are refined and the laws are not. The predicted gap between `pmp_baseline` and `gt_baseline` is ≈ 1.8 t; refined within 2 000 s, `gt_baseline` is 551 kg below the reference.
  - Decide: refine every swarm case from its archived point (the recipe the references follow) instead of re-swarming, or report the law rows as swarm-limited. Untested beyond the gravity turn: the closed-loop laws and the laws with extra variables need the same orbit-closing step with more free variables.
  - **The references' own recipe works unchanged** (user preference 2026-10-06: one refinement algorithm in the thesis). Script: `law_polish_ref_recipe.py` (scratchpad).
    - Same settings as `pmp_swarm_polish.py`: Levenberg–Marquardt at fixed kick, continuation in γ_p (0.0005 rad, then a half step).
    - What changes: no costates. The three durations are solved from the three orbit conditions, where the PMP has five unknowns and five conditions.
    - At a coast bound the PMP drops H_coast_end; a law has no condition to drop, so the kick is solved in the coast's place and the sweep ends there.
    - `gt_baseline`: 22 069.8 kg at 2 000 s (20 s, 603 flights), 22 663.3 kg at 3 500 s. Identical to the trial; the half step changes nothing.
    - It covers the 4-variable cases: the gravity-turn rows, `peg_*` and `show_apollo`. Untested on a closed-loop law.
    - `show_cpr`, `show_exp_shooting` and the two tangent laws also have law constants, which no condition fixes. Proposed: hold them at the swarm's values.
  - **`peg_baseline`: the recipe fails at the first solve, at both coast limits** (`diag_peg.py`, `diag_peg_kick.py`).
    - At the archived point it misses by 48.5 m, −5.7 m/s and −0.122° (J′ 2.13).
    - peg_new steers to the orbit itself, so its altitude miss stays at 41–66 m whatever the durations or the kick. There is no root near the point.
    - Every 3×3 system drawn from D1, Dc, D3 and the kick is near-singular: condition ratio 277–1 161, with the altitude row weakest.
    - The miss also jumps with D1: γ swings 0.3° within 0.4 s, from the 2 s guidance cycle and the freeze.
    - So the recipe covers laws whose terminal state is set by the durations (the gravity turn; to check: tangent, cpr, exponential). The closed-loop laws (`peg_*`, `show_apollo`) need a swarm or a derivative-free method.
  - **`peg_baseline` by Nelder–Mead on J′** (`nm_refine_case.py`; 6 061 flights, 10 min; archive `law_refine_trial_20261006/peg_baseline_refined_nm`).
    - Result: J′ 0.836590 and 16 028.8 kg (−67.5 kg against the row). It misses by 36 m and −0.16 m/s, with γ = 0. Coast 0.04 s (a direct insertion), kick 1.057° (was 1.148°).
    - Not cleanly converged: each restart still found more (J′ 0.8523 → 0.8429 → 0.8404 → 0.8403 → 0.8388 → 0.8366), so it is the best point found, not a verified optimum.
    - J′ pays 1 m/s of miss like ≈ 1.3 t of propellant. The 600-flight estimate kept 16 115 kg with a 1.4 m/s miss; the longer run spent 86 kg more to close it.
    - So the closed-loop law sits close to what its swarm found. Unlike the gravity turn, it has no long-coast family to reach.
- **T32. If the laws are refined (C18), the thesis needs no new algorithm, only a wider scope** (Overleaf `8198fa1`; wording proposed in session 8d81b701, not applied). **ON HOLD (user, 2026-10-06): no thesis changes for now.**
  - Ch. 3 `ssec:pmp_polish` ("Local Refinement of the Indirect Extremal"):
    - the title;
    - the rationale "This matters for the indirect trajectory alone", which becomes untrue;
    - one sentence on a law's 3×3 system;
    - the coast-bound rule for a law;
    - "extremal" → "solution" in the continuation paragraph;
    - optionally, the held constants of cpr, exponential and the tangent laws.
  - Ch. 5 l.590: the "optimal reference" paragraph gets a sentence saying every swarm case is refined the same way.
  - Ch. 6:
    - l.44–45 ("reported first and apart from the flyable laws");
    - `Tables/architecture_cost.tex`, whose refinement cost is on the indirect row only (`tables.py`);
    - every PMP-vs-law gap quoted.
- **C19. Stage 2 is aerodynamically vacuum, so the coast bound stands in for the atmosphere.**
  - `pso_coast_solver._stage2_ode_guidance` sets F_L = F_D = 0, and so do the other PSO architectures and the PMP's Stage-2 kernel.
  - Along the `gt_baseline` family that closes the orbit, propellant rises as the coast lengthens, while the coast's lowest point falls: 109 km at 3 250 s, 65 km at 3 400 s, 23 km at 3 500 s. Past ≈ 3 550 s no orbit closes.
  - With the references' 3 500 s bound, the gravity turn therefore keeps 22 663.3 kg, above `pmp_baseline` (22 620.6 kg), on a flight whose post-flight budget puts its drag at 107 km/s.
  - Within 2 000 s nothing is affected (lowest coast point 204 km). `pmp_baseline`'s coast starts at 138 km, climbing.
  - Related: C17 (the bounds differ by architecture), T30 (the drag-free reference skims), the open flag on the PSO paths dropping drag at separation.
  - Decide with C18: one coast bound for every architecture, checked against a minimum coast altitude (≈ 100–120 km), or that minimum as an explicit condition.

## Launch-site rotation model, 2026-10-05 (code uncommitted; nothing re-flown)

User decision: the latitude is held at launch, and the rotation is credited as the launch-site speed ω·r·cos φ₀ everywhere. That covers the target, the conversion and the budget gain, and also the pseudo-forces (`earth_rotation.planar_pseudoforce_rates`, the ENU terms at heading π/2). The azimuth enters no equation of motion. Label: `EARTH_ROTATION_MODEL = "launch_site"`.

- **T28. Thesis passages to revise** (each discussed first). User rule (2026-10-06): write the convention as the model, with no "earlier/new model" contrast anywhere. The reference table keeps its swarm-point values.
  - **Ch. 2**
    - l.226–249: Eq. `sim_ecef_eci` is now the model, everywhere, not a shortcut taken in the conversion only. The "optimistic, bounded by ω r (1 − sin ψ) cos φ" sentence becomes the statement of the convention.
    - l.289–306: ψ appears in the in-plane terms only as π/2. The cross-heading load is that of a due-east flight.
    - l.345–347: the drift sentence. The inclination is no longer an output of the dynamics.
    - l.478–479: "latitude follows in closed form from the downrange" becomes "held at its launch value".
    - Eq. `dv_gain` l.533–539: Gain = ω r cos φ₀ at the insertion radius, 440.8 m/s, not V_eq cos φ.
    - l.529–531: the residual is the centrifugal work.
  - **Ch. 5:** the rotation-convention text. T18 changes nature: the insertion is now circular in the model, so the old disclosure ("129 m/s short of circular, h_a/h_p read 500 km") no longer applies. What remains to state is that the credit is a due-east one, optimistic against a 44.98° azimuth.
  - **Ch. 6:** every credit disclosure and `\discuss` note, the residual column, and the `fig:gt_axes` caption (the latitude inset is removed from `sec62_gravity_turn.py`).
- **C15. RESOLVED by the change.** No heading or latitude enters the pseudo-forces any more.
- **C11. Changed.** The inclination diagnostic (legacy `run()` only) now resolves the state on `LAUNCH_AZIMUTH` at the launch latitude. The dynamics never do this, so it reads as the drift of a vehicle that had flown A_I as its ground heading (≈ 49.8°). Decide whether to keep it.
- **C16. `gt_apogee`'s budget residual is +17.4 m/s, not a few m/s** (verification flight, scratch).
  - The gain is evaluated at the target radius (440.8 m/s), but `gt_apogee`'s budget window ends at SECO, at 174 km (ω r cos φ₀ = 419.9 m/s). Of the residual, +20.9 m/s is that offset and −3.5 m/s is the centrifugal work.
  - Fix: evaluate the gain at the radius where the window ends, or extend the window to the apogee (T27).
  - `gt_baseline`'s residual is −5.8 m/s (was −125.2).
- **C17. Under the new model `gt_apogee` does a Hohmann-like transfer.**
  - Measured on the verification flight: SECO at 174 km, 2 721 s coast to apoapsis, 93.0 m/s circularisation (was 1.1). Propellant left: 22 315.9 kg (archived 21 534.2).
  - The coast exceeds the 2000 s bound the swarms are held to.
  - Report it or bound it for Ch. 6 fairness.
- **Review pass (same day).**
  - The budget identity closes on both verification flights. The residual is −(in-plane centrifugal work) + (gain-radius offset) to within 0.3 m/s.
  - `pmp_norot` re-flies bit-identical, and `gt_norot`'s archived x re-scores to the identical J′, so the two rotation-off rows stay valid.
  - Added `_data.check_one_rotation_model`, called by `make_all` and `tables`, so a partly re-flown matrix cannot be drawn mixed.
  - `force_model_note` no longer flags an inertial Stage 2 flown under the new model.
  - Left stale, as they were already: `Tese/Project_Description/EARTH_ROTATION_CHANGES.md` and `simulator_eom_dynamics_kinematics.tex` (March 2026), and the legacy suite's latitude plot (now a flat line).
- **Tests.** 274 pass and 16 skip until the re-fly (`tests/_refly.py`); the skipped ones re-enable themselves when `pmp_baseline` and the matrix rows carry the label. New: `tests/test_rotation_model.py`, plus rewritten level-flight and frame-counterpart tests. Re-pinned to the current model at the same x: `test_direct_grid` (2 J values) and `test_guidance_refresh_mode` (3 cases). The replayed `pmp_baseline` extremal now scores J′ 121.8, against 0.76: it no longer reaches the target.
- **Not changed, against the plan:** `dev-notes/pmp_swarm_polish.py` `ENVIRONMENT_KEYS` does not get the label. Refusing pre-change archives as starts would block refining the new references from the current extremals, which is the cheapest path to the re-fly.

## Ch. 6 walkthrough, 2026-10-05 (open; decisions on the "Chapter 6 Walkthrough" artifact)

- Page: https://claude.ai/artifact/2xqwupr6w8A68RqJ2oJq7e (db collections `reviews`, `sections`). 87 items, X1–X7 and S1–S7. Source: that session's scratchpad `ch6walk/` (`content.py`, `build.py`).
- **Version 3, 2026-10-06** (89 items; source in session 67a7e9e8's scratchpad `ch6walk/`, where `content_v3.py` patches `content.py`):
  - The user's §6.1 answers are applied: S1-T1 (a table), S1-T3, S1-T4, S1-T5 (no refinement detail) and the S1-F1 figure comment.
  - S1-T2 answered an outdated recommendation, so its stored answer was cleared; the rewritten item quotes it.
  - X2 and X7 are rewritten; X8 (writing before the re-run) and S2-V2 (C17) are new.
  - Every claim at risk carries a `% [re-run]` line and an amber note, with predictions from session 8d81b701 (`predict/`, `predict_pmp*` logs).
- **Version 4, 2026-10-06 evening** (90 items; `content_v4.py` patches v3).
  - The user's answers on 6.1–6.4 are implemented in `Plots/results_figures/`, uncommitted (page item X9):
    - `sec63.reference_profiles`, the new `fig:reference_card` file;
    - `tables.reference_results`: h_MECO and h_coast replace h_a and h_p;
    - `run_card.draw` clips at insertion;
    - `sec62.secondary_axes` is 2×2;
    - `_panels.waypoint_figure` gets MECO/SECO lines;
    - `peg_atmosphere` is out of `FIGURES`;
    - `tables._waypoint_table` splits the steering loss.
  - Tests: 289 pass, 5 skip.
  - The 6.1–6.3 drafts are rewritten from the comments, with the refined references' numbers.
  - Previews: `render_v4/` and `preview_root/` in that session's scratchpad. The law figures are drawn on the pre-re-run archives with their own reference, so no figure mixes models.
- **T29. Ch. 3 `tab:pso_settings` gives the indirect coast bound as [0, 2000] s.**
  - It is 3 500 s for the references since 2026-10-06 (2 500 s for `pmp_vacuum`); the laws keep 2 000 s.
  - Ch. 3 l.666 measures the omitted costate terms "over a 2000 s coast". The refined references coast 1 741–2 500 s, so re-check that bound.
- **T30. CLOSED 2026-10-06 (`fbca83e`).** `pmp_vacuum` carries its own 2 500 s coast bound, which keeps its lowest point at 57 km. Whether §6.1 says so is an optional sentence in S1-D1 (the user's S1-T5 answer: no refinement detail).
- **T31. CLOSED 2026-10-06 by the user (S1-T3).** The model change is to be treated as if it were the model all along, so the swarm-point comparison stays. The drafts quote the gain again: +1.92 to +2.84 t (S1-D2, X3).
- **T32. Add `jaggers1974peg` to the bibliography at the final edits** (S3-T2). It is cited in the S3-D1 draft.
  - Jaggers, R. F., *Asymmetrical booster ascent guidance and control system design study, Volume 5: Space Shuttle powered explicit guidance*. Boeing Aerospace Co., Houston. Report 5-2581-HOU-154, NASA CR-140191, contract NAS9-13568, 28 June 1974. Metadata checked on NTRS 19740024190; the PDF is `References/19740024190 - PEG.pdf`.
  - The supporting line is the footnote to its problem statement: "restricted to a single burn maneuver, i.e., no long coast arcs between stages".
- **T33. Captions to rewrite at the final edits.**
  - `fig:reference_card` now draws three time-history panels, and its tangent-fit panel is gone (0.33° RMS goes in the text).
  - `fig:gt_axes` is 2×2. The caption must say the non-rotating run's pseudo-forces are identically zero and not drawn.
  - `fig:peg_waypoint` and `fig:apollo_waypoint` carry MECO/SECO lines.
  - `tab:reference_results` loses h_a/h_p, so the caption states the 500 km circular insertion.
- **K11. Check in Overleaf:** `tab:peg_waypoint` and `tab:apollo_waypoint` are now 11 columns (steering split), and `fig:reference_card` is ≈ 0.9 pp at 6.3 × 8.2 in.
- **C20. The apogee check's impulsive circularization saves almost nothing** (S2-T2 check).
  - At apogee γ = 0 by construction, and 93 m/s as a finite burn at 934 kN lasts 2.7 s (0.003 rad of arc).
  - Its margin over the swarm's gravity turn is the trajectory (low SECO, long coast) and the swarm's convergence (C18), not the idealization. The S2-D1 draft says so; the user had suggested the impulse explains it.
- **C17, follow-up: the coast bounds differ by architecture.** References 3 500 s, swarm laws 2 000 s, apogee check unbounded (2 721 s predicted). No swarm law is predicted above 818 s. Page item S2-V2 proposes reporting this as is.
- **T27 and C16 together.** The predicted `gt_apogee` has SECO at 174 km, a 2 721 s coast and a 93 m/s circularization. In its SECO-cut window it reads a gravity loss of 1 180 m/s and a residual of +17.4 m/s. S7-V1's window extension fixes both; `gt_apogee` re-flies in about a minute.
- **T27. `gt_apogee`'s ΔV budget is cut at SECO,** so its 1 132 s coast to apogee lies outside the window and its gravity loss (1 378 m/s) is not comparable with the others'. In every other case the coast is inside the window; the reference spends 402 m/s of gravity loss in its coast. Proposed in S7-V1: extend the window to the apogee and add the 1.1 m/s impulse.
- **C14. `run_card.draw` and `sec62.secondary_axes` draw past insertion.** This affects `fig:gt_baseline_card` (a)/(b) and `fig:gt_axes` (c), (d) and (f). Proposed in S2-F1/S2-F3: clip with `_panels.to_insertion`.
- **C15. The pseudo-forces hold the heading at the launch azimuth while the latitude follows the great circle.** This is the dynamics counterpart of C11, found 2026-10-05 while working out the rotation-credit projection (T18).
  - Where: `pso_coast_solver` l.731, `indirect_pso_solver` l.277, `rocket_ascent` l.1331/1761 (legacy) and l.823 (diagnostic). Every architecture does the same, so comparisons between them stay internally consistent.
  - Effect: the speed at which the model holds level flight at 500 km depends on where a case inserts.
    - With the heading held: 7 300.5 m/s for `pmp_baseline` (30.3°), 7 320.2 m/s for `peg_baseline` (35.9°), 7 373.5 m/s for `gt_baseline` (48.4°), 7 388.8 m/s for `show_linear_tangent` (51.5°).
    - With the heading propagated by Clairaut (sin ψ = cos i / cos φ): 7 295–7 301 m/s at every latitude.
  - Any projected target (ω r cos i, 7 301.1 m/s) is level flight for every case only if this is fixed with it. Script: `projection_check.py` in that session's scratchpad.

## `pmp_norot` adopted 2026-10-05 (code `b99acce` + archive `724707a`; thesis `4e30b7e`, not pushed)

- **Result:** seed 3's half-step extremal (user's choice, to match `pmp_baseline`) leaves 19 384.5 kg, on a true 500 km circular orbit.
  - Coast 1 942.8 s; final burn 3.2 s; residual −1.6 m/s, against −112.5 m/s for `pmp_baseline`.
  - `gt_norot`'s shortfall is now 1 788 kg, against 1 574 kg for `gt_baseline`.
  - The rotation is worth 2 877 kg to the reference and 3 091 kg to the gravity turn.
  - The harness re-flies it from the stored vector in 0.5 s, to the same numbers.
- **Code (`b99acce`, 277 tests pass):**
  - `PMP_NOROT_EXTREMAL`/`_SEARCH` in `build_matrix`.
  - `pmp_norot` added to `REPORTED_CASES`, `SWARM_POINT_KG`, `reference_results` and `CASE_LABELS`.
  - `_reference_for` matches non-rotating cases to `pmp_norot`.
  - The ranking figure gets a third reference line; its legend is on one row, and its dagger note now names only the sea-level nozzle.
  - CLAUDE.md and worktree.md updated.
- **Archive (`724707a`):** re-flown at `b99acce`, clean, into `Output/results_matrix_norot_20261005/`, then copied in. The CSV was rebuilt: one row added, the rest unchanged. The `pmp_reference.npz` sha256 is unchanged.
- **Thesis (`4e30b7e`), rendered byte-identical to the approved preview:**
  - Four tables gain a row: `reference_results`, `loss_budget`, `full_results`, and `architecture_cost`'s indirect cell.
  - `gt_results` gains the `gt_norot` shortfall.
  - Four figures gain a bar or point: `law_ranking`, `arc_structure`, `accuracy_vs_propellant`, `solve_cost`.
  - Estimated +0.1 pp.
  - Wording E1–E4, approved: the `tab:gt_results` and `tab:architecture_cost` captions, two §6.1 notes, and one §6.2.4 note.
  - Sweep clean.
- **Closes T22, and T17 for `gt_norot`.** `gt_sea_level_engine` keeps its local treatment.
- **T25. The recipe sentence is 5.6 kg off for `pmp_norot`.** Ch. 3 l.711–712 ("the best extremal kept") and Ch. 5 l.592–593 ("the extremal leaving the most propellant was kept") do not hold: seed 2's first pass left 5.6 kg more than the seed-3 extremal kept.
  - Settle this with T23 at the `pmp_vacuum` gate, which also picks that reference's seed.
- **T26. Disclose the coast bound in §6.1's prose.** Every `pmp_norot` continuation was stopped by the 2000 s coast bound with the propellant still rising (≈ 60 kg per 0.00025 rad). The extremal kept is the last converged one, 57 s short of the bound.
  - The pinned-coast solve at 2000 s failed. Its estimate, 19 421.6 kg, puts at most ≈ 40 kg beyond the reference.

## Outline review, round 4 applied 2026-10-04 (thesis pushed as `3b42c1f`)

- **Applied:** all 10 round-4 items, W5-01 … W7-01, approved on the Thesis Outline and Cuts artifact with no comments. They carry out the round-1 decisions on Ch. 5–7:
  - 12 approved; N6-07, N6-10 and N6-11 rejected; N6-01 is information only.
  - Chat answers: N5-04 goes to §5.2.1, with a pointer in §6.2.4; N6-05 moves to the appendix; N7-01 drops the §6.7.3 clause; the N6-02 card carries the waypoint and the tangent fit.
  - Files: Ch. 5, Ch. 6, Ch. 7, `Thesis.tex` (re-enables `Thesis_Appendix_A.tex`), and `Thesis_Appendix_A.tex` ("Supplementary Results", Appendix B: `fig:accuracy_vs_propellant`, `tab:full_results`, `fig:solve_cost`).
  - Figures: `results_reference_card`, `results_peg_waypoint` and the new `results_gt_axes` rendered into `Figures/`. Five orphaned PNGs removed with `git rm` (staged): `results_reference_trajectory`, `results_peg_vs_reference`, `results_gt_{architecture,rotation,engine}`.
  - Script: `apply_round4.py` in that session's scratchpad.
  - Sweep clean.
- **Estimate:** Ch. 5 9.3 → 9.1 pp, Ch. 6 17.8 → 14.2 pp, Ch. 1–7 78.5 → 74.7 pp. Appendix B adds 2.5 pp, so the appendices hold 6.7 pp. With ≈ 8 pp of Ch. 6 prose to come, ≈ 2.7 pp remain to cut.
- **Plot library, uncommitted (code repo):**
  - `run_card.draw(waypoint=, pitch_fits=)` and `_panels.waypoint_figure(extra=)`.
  - `sec62.secondary_axes` → `results_gt_axes.png`, using `_style.TALL_6`.
  - `sec63.reference_trajectory` and `sec63.peg_vs_reference` are removed.
  - 276 tests pass.
- **T24. The 100-page total is tight.** The appendices hold 6.7 pp (A 4.2, B 2.5), so the total is ≈ 80 + 11–13 (bibliography) + 6.7 ≈ 98–100 pp. Any further float sent to an appendix has to come out of the 80, not be added on top. Calibrate against the Overleaf PDF before moving more.
  - The reference card's faint drag-free line is today's 250×1000 `pmp_vacuum`. Redraw it after the D3 gate (T23).
- **K10.** Check when compiling in Overleaf:
  - Appendix B: its heading, TOC entry, and the numbering of its floats (B.1, B.2);
  - the sideways `tab:full_results` inside the appendix;
  - `fig:gt_axes` (6.3 × 7.8 in at text width) with its caption, on one page;
  - the reference card's panel (b) legend, which now names the waypoint.

## Outline review, round 3 applied 2026-10-04 (thesis pushed as `3cf2451`)

- **Applied:** all 18 round-3 items, W3-01 … W4-10, approved on the Thesis Outline and Cuts artifact with no comments. They carry out the round-1 decisions on Ch. 3–4 and four answers given in the chat:
  - the appendix also takes classical PEG and the exponential shooting fallback;
  - Teofilatto's two-segment programme and CFPAR move there as they are;
  - `fig:kick_profiles` moves whole;
  - N3-13 is written now, with `pmp_norot` added.
  - Files: Ch. 3, Ch. 4, Ch. 5, Ch. 6 (two consequential edits), `Thesis.tex`, and the new `Thesis_Appendix_Guidance.tex` ("Additional Guidance Options", the only appendix built).
  - Script: `apply_round3.py` in that session's scratchpad.
  - One edit beyond the drafted wording: CPR is now defined at its first use (Ch. 4 §4.3.3), because the CFPAR sentence that defined it moved to the appendix.
  - Sweep clean.
- **Estimate:** Ch. 3 23.0 → 20.4 pp, Ch. 4 15.1 → 9.9 pp, Ch. 1–7 86.6 → 78.5 pp. The appendix adds 4.2 pp outside the 80-page limit.
  - With ≈ 8 pp of Ch. 6 prose to come, ≈ 6.5 pp remain to cut.
  - The 100-page total now holds ≈ 80 + 11–13 (bibliography) + 4.2 (appendix), leaving ≈ 3–5 pp for any Ch. 6 floats moved to an appendix.
- **No edit, by decision:** N3-01 (the promoted sections were intended), N3-02, N3-09, N3-14. N3-10 and N3-11 were verified: the flown forms are already described.
- **Still to decide:** round 1 for Chapters 5–7 (N5-01 … N7-01). Decided 2026-10-04; see round 4 above.
- **T22. CLOSED 2026-10-05 (thesis `4e30b7e`).** `pmp_norot` in Ch. 6. Ch. 5 now lists it (`tab:case_matrix`, nineteen cases) and names the non-rotating reference in §5.2.4, but Ch. 6 has no row, figure or prose for it.
  - Once the 750×1500 runs are adopted, add it to `Plots/results_figures` (tables and the reference figures).
  - Give `gt_norot` its shortfall against it in `tab:gt_results`.
  - Then revisit T17.
- **T23. Recheck the N3-13 wording when the runs finish.** Ch. 3 (`tab:pso_settings`: Indirect 750/1500, "seeds 1–4 and 42"; §3.6 and §3.7) and Ch. 5 §5.2.3 now say that every reference comes from 750×1500 swarms flown from five seeds, with the best refined extremal kept.
  - If the D3 gate keeps today's 250×1000 `pmp_vacuum`, these lines need an exception.
  - Also confirm the seed list for all three references.
- **K9.** Check when compiling in Overleaf:
  - Appendix A: its heading and TOC entry, and the numbering of `fig:kick_profiles` and `eq:kick_profile` (A.1);
  - the new `eq:optimal_ascent_problem`: its long `\text{}` row inside `aligned` may overrun the line;
  - `tab:events` (`l l l`, no wrapping);
  - `tab:guidance_laws` without its code-name column;
  - §3.3, which now carries two labels.

## Outline review, round 2 applied 2026-10-04 (thesis pushed as `9495a8b`; plot-library change `bed33f5`)

- **Applied:** W0-01 … W2-13 from the "Round 2" section of the Thesis Outline and Cuts artifact.
  - Files: `Thesis.tex`, Ch. 1, the rebuilt Ch. 2 (script `rebuild_ch2.py` in that session's scratchpad), Ch. 3, Ch. 4, Ch. 5, Ch. 6 and the nomenclature.
  - Re-rendered from code: `tables.py` and `sec65_losses.py` now read ΔV; the outputs are `Tables/loss_budget.tex`, `results_loss_budget.png` and `results_loss_accumulation.png`.
  - Sweep clean. Estimate: Ch. 2 25.3 → 14.4 pp; Ch. 1–7 97.3 → 86.6 pp, plus about 3 blank pages removed. About 15 pp still to cut once Ch. 6 prose (+8 pp) is written.
- **Still to decide:** round 1 for Chapters 3–7 (N3-01 … N7-01). Ch. 3–4 were decided and applied in round 3 (above).
- **T19. `tab:env_params` has no source for ω_E** (7.2921159e-5 rad/s, from `constants.OMEGA_EARTH`). Name one.
- **T20. Reporting Δi and the lateral load in Ch. 6** (user, N2-09). Not yet defined how or where. Before reporting:
  - fix C11;
  - compute both offline from the 18 archives;
  - choose the held or the great-circle heading for F⊥.
  - Ch. 2 l.306 keeps "reported over the ascent in Chapter 6" until the Ch. 6 location is set. Ch. 7's two Future Work items and the §6.2.4 note then go back from "computed" to "reported".
- **T21. CLOSED 2026-10-04.** Ch. 2's pressure-loss equation changed form (W2-13): ∫ p_a A_e/m dt, against vacuum thrust. `Auxiliary/losses.py` l.14 and l.85 integrate exactly this, so the thesis now matches the archived numbers.
- **K8.** Check when compiling in Overleaf:
  - the rebuilt Ch. 2: the Ascent Phases `[H]` figure now opens the chapter;
  - `tab:env_params` in Ch. 5;
  - chapters now open on any page.
- **C13.** `results_loss_accumulation.png`: the vertical "MECO" label overlaps the legend (pre-existing, visible after the re-render).

## Outline and page-cut review of 2026-10-04 (open)

- On the "Thesis Outline and Cuts" artifact (https://claude.ai/artifact/WhBU2mbE9WyBqeaGydw9dB): an outline of every heading with page estimates, and 69 items N0-01 … N7-01. Decisions go to db collection `reviews` and heading tags and notes to `outline`.
- Estimate, ±15 %: 97.3 pp of content in Ch. 1–7, plus ≈ 3 blank pages, plus ≈ 8 pp of Ch. 6 prose still to come. That puts the cut needed for 80 pp at ≈ 28 pp.
- Offered: Tier 1 12.0 pp, Tier 2 9.4 pp, Tier 3 7.7 pp. The script is `page_estimate.py` in that session's scratchpad.
- New findings filed only there:
  - Ch. 2 l.315 still says the lateral load is reported in Ch. 6 (stale since XR1).
  - Ch. 2 l.349 and l.397–404 claim A_G and Δi are reported.
  - `tab:force_params` labels 6378 km the mean Earth radius; it is the equatorial radius.
  - Ch. 3 §3.5–§3.8 were promoted to `\section` in Overleaf `bb9a883`.
  - Ch. 5 l.108 still says "demonstration case".
  - Ch. 7 l.95 cites a vehicle limitation that §6.7.3 does not print.

## Decisions of 2026-10-04 (Chapter 6 Review artifact, 31 items)

- **Applied, uncommitted:** FX1, FX3, MV1, MV2, MV4, DP1–DP7, CA1–CA4, DN1–DN6, OR1, OR2, OR3, CM1.
  - Ch. 5 gains three statements: why the sea-level engine is the alternative (§5.1.1), what `peg_direct` changes besides the architecture (§5.2.3), and the two checks on the delta-v budget (§5.2.4).
  - Ch. 6 §6.7.1 keeps one printed sentence; its verification paragraphs moved to Ch. 5.
  - OR1/OR2: `show_apollo` left `tab:showcase_laws` and `fig:showcase_laws` (now a 2×2 grid), and `peg_baseline` left `tab:segmented_results`. Done in `tables.py` and `sec67_capabilities.py`, re-rendered into the thesis.
  - OR3: `Thesis_Appendix_A.tex` is reduced to its heading and a comment.
- **Rejected, no edit:** RC1 and RC2 (T18 stays parked), FX2 (the seed sentence of §6.7.3 stays; see T9), MV3 (post-separation drag stays in the `tab:loss_budget` caption only; see C3).
- **XR1: option (b), applied.** The inclination diagnostic is unreliable (C11), so Ch. 6 reports neither number. The "Optional" bullet of §6.2.2 is gone; §6.2.4 and Ch. 7's two Future Work items now say the out-of-plane forcing is computed (Section `ssec:pseudo_forces`) rather than reported.

## Decisions of 2026-10-03 ("still open" page of the Chapter 5 Review artifact)

- **Done, uncommitted:**
  - T7: `tables.caption_values` writes `Tables/caption_values.tex`; the preamble inputs it and the two Ch. 6 captions use `\circDvApogee` and `\dragAfterSeparation`.
  - C1, C2, C9, C10: stale comments fixed.
  - C5: `APOGEE_CHECK_COAST_FRAME` now defaults to `"rotating"`; CLAUDE.md and worktree.md updated; 273 tests pass.
- **Round 4 applied, uncommitted:**
  - T1 + T2 (FIG2): arc-structure figure redrawn into `Figures/`, with its caption and drawing note.
  - T12 (T12b): the Hamiltonian jumps at the switches.
  - T15 (T15a–d): Ch. 1 is a short introduction, Ch. 2 the phases and arcs, Ch. 4 the guidance split; B9 says the kick is a step in γ.
  - T16: WIKI (Teren 1966 replaces the Orbiter wiki, whose entry is deleted), BEN (bib title and TM number), MAH (cited as the preprint), R12 and R4 reworded. R11 needed no change.
- **Still open from T16:** R8 (Betts 1998 not in `References/`: kept until the user checks it) and R10 (rejected: both Apollo t_go citations stay, though Bennett was checked and does not give the truncation, and Chandler & Smith 1967 is not in `References/`).
- **T3 (FIG3): approved and applied.** Segmented-schedule figure redrawn for the law-terminated schedule flown, with a new caption, its introducing sentence and its drawing note. The §3.4 sentence now says only the segmented mode steers before ignition.
- **No edit:** T9 (Ch. 5 keeps the seed-3 sentence; closed), T17 and T18 (parked), T8 (to be written with the Ch. 6 results).
- **On hold by the user:** T13 (spelling), C8 (time-to-go constant).
- **Closed with no change:** C3, C4, C6.
- **C7:** approved, but the user deletes the folder; the staging copies are byte-identical to the matrix archives.

## Thesis: open

- **T1. The direct architecture is "cut at circular velocity", but no such event exists.**
  - Where: `fig:arc_structures` (the Direct row, "event: circular velocity reached"). §3.4.3.2 is fixed (§3.4 review, 2026-10-01); the `tab:events` row is deleted (§3.1–3.3 review, S10, 2026-10-02).
  - Code: `direct_pso_solver` registers only crash events. The burn ends at t_burn% (a decision variable) or, when law-terminated, at peg_new's own t_go.
  - §3.4.3 contradicts itself: it describes an event cut-off and then lists t_burn% as a decision variable.
- **T2. `fig:arc_structures` has stale labels.** It is now the only arc-structure figure.
  - The brute-force row says "only variable: kick angle α", but the search runs over γ_p (§3.4.2).
  - The segmented row partitions Stage 2 only, but a law can start in Stage 1 (§3.4.5).
  - Its dimensions do not match the law-terminated forms flown: direct has 1 variable, segmented 2–3.
  - Source: `dev-notes/figures/fig_architecture_arc_structures.py`.
- **T3. One other schematic is stale:** `fig:segmented_schedule` (`segmented_schedule.png`). Script in `dev-notes/figures/`.
  - `fig:dependency_tree` is no longer built: the user commented out Ch. 5 §5.2 Software Implementation on 2026-10-03.
- **T4. CLOSED 2026-10-01 (R5).** §3.4.4 and the `tab:pso_settings` caption now name the law-terminated segmented form flown.
- **T5. CLOSED 2026-10-02 (S10).** The burnout, radius-check, horizontal-flight and apogee-match rows exist on the brute-force path only (`rocket_ascent.py` l.1903); the swarm paths carry only `_event_crash`. They are now marked † in `tab:events`.
- **T6. CLOSED 2026-10-02 (S10).** The "timeline emerges from the physics" sentence is deleted.
- **T7. Numbers are hand-typed in captions:**
  - 1.1 m/s in `tab:full_results` and `tab:loss_budget`;
  - 1.8 m/s in `tab:loss_budget`.

  They do not update when a case is re-flown.
- **T8. The PEG coast rows collapsed to direct insertion** (old "flag 2"). peg_new under `pso_coast` coasts 0.5 s. The Ch. 6 treatment is deferred.
- **T9. CLOSED 2026-10-04 (outline round 3, W3-07).** `tab:pso_settings` and Ch. 5 §5.2.3 now give the references as 750×1500 from five seeds; the laws keep 250×1000 at seed 42. Follow-up in T23. History:
- **T9 (as filed). The PMP swarms are presented as 250×1000, seed 42** (user decision 2026-10-01, R3: omit the exception).
  - In fact the PMP extremals come from seed-3 swarms, the atmospheric one at 750×1500 (`run_results_matrix.py` l.467, l.476).
  - Ch. 5's optimal-reference paragraph (§5.2.3) still states this, against `tab:pso_settings` ("seed 42"). Since `12f255e`, Ch. 6 §6.7 also points to `tab:pso_settings` for "the fixed seed".
  - Decide whether Ch. 5 keeps the sentence.
- **T10. CLOSED 2026-10-02 (Ch. 3 de-duplication, A4).** Ch. 4 §4.4.1 now gives the open-loop tangent form flown under `pso_coast` (`eq:tangent_openloop`) with its bounds, and l.374–375 names the tangent and exponential laws together.
- **T11. CLOSED 2026-10-02 (Ch. 4 review, A1/R3).** §4.4.4.2 now cites `mahajan2025peg`, added to the bib.
- **T12. CLOSED 2026-10-04 (outline round 3, W3-06).** The Multi-arc integration subsubsection is gone; §3.6.3 is now three paragraphs that no longer restate the corner conditions. History:
- **T12 (as filed). §3.4.3.3 (Multi-arc integration) may overstate the corner conditions.**
  - It says the costates "and the Hamiltonian remain continuous by the Weierstrass–Erdmann corner conditions, so no interior jump conditions are imposed".
  - The costates are continuous by construction. H continuity at a thrust/coast switch is an optimality condition, which the penalty enforces only through Eq. `duration_stationarity`; integration does not impose it.
  - Verify against `dev-notes/pmp_duration_conditions.py` before rewording.
- **T13. British and American spellings are mixed across the thesis.**
  - Ch. 3 prose is now American throughout, matching its title and headings (S22, 2026-10-02).
  - The other chapters are still mixed. Census of -ise/-ize, -our/-or, manoeuvre/maneuver and -lled/-led forms (UK/US): Ch. 1 3/23, Ch. 2 31/30, Ch. 4 37/21, Ch. 5 26/10, Ch. 6 11/31.
  - Decide on one convention. The converter is `dev-notes/us_spelling.py` (dry run by default, `--apply` to write): it changes prose only and leaves comments, labels, refs, cites and `\texttt` untouched.
- **T14. CLOSED 2026-10-02 (Ch. 4 review, A5).** `tab:guidance_laws` now has a "Flown as" column instead of the $t_{go}$ column, and §4.2.1 says only Apollo uses the estimate in Ch. 6.
- **T15. Ch. 1 l.65 and Ch. 2 l.850 both describe the three ascent phases** and the endo/exo split. Ch. 3 is not involved; this was left out of the Ch. 3 de-duplication pass.
  - The Ch. 4 review (C1) found that Ch. 4 l.36 says the same: open-loop on the endo arc, closed-loop on the exo arc.
  - Recommendation: Ch. 4 owns it, and Ch. 1 and Ch. 2 reduce to pointers.
  - Not applied: the user kept Ch. 1 and Ch. 2 out of that pass. B9 (Ch. 2 l.120 calls the kick "the limiting case" of an instantaneous α change) waits for the same reason.
- **T16. Ch. 4 citations that need the source in hand** (Ch. 4 review, 2026-10-02; not changed):
  - **R4:** the exponential pitch law (§4.4.5) has no source. Cite one, or say the law is introduced here.
  - **R8:** `BettsSurvey1998` at Ch. 4 l.43 (onboard simplifications, correction terms). `suresh2015integrated` or `tewari2011advanced` may fit better.
  - **R10:** `ChandlerSmith1967IterativeGuidanceMode` for "the classical Apollo expression" of t_go (l.160). It sits inside the A4 paragraph, which was left alone.
  - **R11:** `Ulrich` on the implemented CPR ramp (Eq. `cpr`). Check that it gives the ramp.
  - **R12:** l.211, "Steering laws define the time history of the vehicle flight-path angle~\cite{Ulrich}". The laws command attitude.
  - **Bennett 1970:** the new Apollo paragraph cites `bennett1970lunar` for the vertical-first thrust priority. The attribution comes from the `apollo_guidance` docstring (Luminary P12). Verify it against the paper.
  - **`mahajan2025peg`:** its `booktitle` is "AAS/AIAA Astrodynamics Specialist Conference", inferred from the PDF name (ASC25). The PDF says only "(Preprint) AAS 25-844".
  - **`orbiterwiki_peg`:** a wiki page, last edited 26 Sep 2021, now the source of the classical scalar variant. The page was checked on 2026-10-02 and gives sin(pitch) = A + B·t + C, C = (μ/r² − ω²r)/a₀. Decide whether a wiki source is acceptable; the variant is not flown.
- **T17. Closed for `gt_norot` 2026-10-05 (thesis `4e30b7e`); `gt_sea_level_engine` stays as decided.** Two cases have no reference of their own environment (Ch. 5 review X6, 2026-10-03).
  - `gt_norot` and `gt_sea_level_engine`. Ch. 5 §5.2.4 quotes the shortfall against "the reference of the same environment" and has no clause for them; Ch. 6 `tab:gt_results` handles them locally.
  - User decision: address it later by adding a reference for those cases, not by rewording.
  - 2026-10-04: plan in `dev-notes/pmp-references-750x1500-plan-2026-10-04.md`. A new `pmp_norot` reference (750×1500, five seeds, refined) covers `gt_norot`. The user chose not to build one for `gt_sea_level_engine`, so `tab:gt_results` keeps its local treatment of that case.
  - 2026-10-04 (round 3): Ch. 5 §5.2.4 now names the non-rotating reference, and `tab:case_matrix` lists `pmp_norot`. Ch. 6 waits for the runs (T22).
- **T18. The rotation-credit disclosure is now stated nowhere** (Ch. 5 review O7, user decision 2026-10-03: "drop it").
  - Ch. 5's "Two conventions" paragraph is commented out, and Ch. 3's and Ch. 6's pointers to it are removed. Ch. 6's `\discuss` notes still name the convention.
  - The single-seed limitation survives in Ch. 6 §6.7.
  - Raised again by the Ch. 6 review (2026-10-04, RC1/RC2); both rejected, so this stays parked. The definition exists (Ch. 2 l.258, Ch. 3 l.449); its consequence does not: insertion 129.2 m/s short of circular while the h_a/h_p columns read 500 km. Also, `tab:loss_budget` Gain = ω r_target cos φ sin ψ = 311.5 m/s, not Eq. `dv_gain` (409 m/s), and the residual (−112 to −138) is mostly that 129.2 m/s credit gap, not the pseudo-force work Ch. 2 l.677 names.

## Thesis: check when compiling in Overleaf

- **K1. Table widths are estimated, not compiled.**
  - `tab:full_results` is sideways, with `tabcolsep` 4pt, at about 94 % of the text height.
  - `tab:segmented_results` and `tab:loss_budget` are at about 94 % of the text width.
- **K2.** `tab:events` with the new second-stage ignition row.
- **K3.** The shortened §2.8 page: two `[H]` figures and the event list.
- **K4.** The redrawn `frame_strategy.png` (switch now labelled at insertion) and the new `eq:tangent_openloop` in Ch. 4.
- **K5.** Ch. 4 after its review:
  - `tab:guidance_laws` with its new "Flown as" column;
  - the new `eq:apollo_priority`;
  - the shortened §4.4.4 (PEG);
  - the two new bib entries: `@misc` with `url`/`urldate`, as the SpaceX entries use.
- **K6.** The Ch. 5 review, round 1 (2026-10-03):
  - `tab:fixed_conditions` now has a wrapping `p{0.5\textwidth}` value column and a factor block;
  - `tab:stage_params` has three `\multicolumn{2}{c}` rows (C_D, C_L, reference area);
  - the new `tab:numerics` in Ch. 3 §3.3.1;
  - Ch. 5's new `\begin{comment}` block around the reporting conventions.
- **K7.** The Ch. 6 review (2026-10-04): `fig:showcase_laws` as a 2×2 grid; the longer explanatory paragraph of Ch. 5 §5.2.4; §6.7.1 now one sentence ahead of its notes.

## Code repository

- **C1.** The `shed_fairing_if_due` docstring says MECO is at "71–135 km". Six reported cases cut off at 60.6–66.3 km.
- **C2.** The `simulation_parameters.py` §6 header still says "guidance-start trigger". `ATMOSPHERE_EXIT_METHOD` is read only by the fairing rule under `"atmosphere_exit"` and by the reference-cache key.
- **C3. Post-separation drag.** The swarm paths are drag-free after separation; the brute-force path keeps drag.
  - The thesis discloses it (decided 2026-10-01).
  - Optional fix: apply drag in `_stage2_ode_guidance`, or stop the budget's drag integral at separation.
- **C4.** `tables.SWARM_POINT_KG` is hard-coded, because the swarm archives are untracked.
- **C5.** The `APOGEE_CHECK_COAST_FRAME` config default is still `"inertial"`; the matrix flies `"rotating"`.
- **C6.** The message of `71da0d5` describes only the figures. That commit also holds the α fix, the repair and the worktree changes.
- **C7.** Delete `Tese/src/Output/_clean_rerun_20261001/`, the staging copy of the clean re-runs.
- **C8. The rocket-equation time-to-go uses the wrong burn-time constant** (Ch. 4 review, 2026-10-02).
  - Where: `rocket_ascent.py` l.1139, l.1175, l.1200; `pso_coast_solver._compute_tgo_stage2` l.343–352.
  - It computes T_BUP·(1 − e^(−V_G/V_e)) with T_BUP = m_prop/ṁ. The exact inversion of the rocket equation needs τ = m/ṁ, as `peg_guidance_new` (l.83, l.91) and Ch. 4 Eq. `tgo_peg` already use.
  - Effect: t_go is short by the factor m_prop/m. That is about 4 % at second-stage ignition (96 570 kg against 92 670 kg of propellant) and about 20 % at `show_apollo`'s insertion (15.4 t left).
  - Cases affected: `show_apollo` (coefficients and freeze timing). Also `show_ref_track_apollo`, whose final burn ends when this estimate expires (`reference_track_solver.py` l.242–253).
  - Ch. 4 Eq. `tgo_rocket` documents the code faithfully and calls it "exact". The decision (fix and re-fly, or disclose) sets the thesis wording.
- **C9.** In `simulation_parameters.py`, the l.304 comment calls `"rocket_equation"` the "current default", but l.310 sets `TGO_ESTIMATOR = "peg_new"`. The matrix overrides it to `"rocket_equation"`. Separately, `apollo_guidance.estimate_apollo_time_to_go` (the truncated form) is called only by its tests.
- **C10.** The `simulation_parameters.py` l.203 comment still gives γ_p ∈ [1.54, 1.57] rad for the instantaneous kick. The bounds are [1.50, 1.57] (l.689, l.760, l.808).
- **C11. The achieved-inclination diagnostic uses the launch heading far from the launch site** (found 2026-10-04 while preparing XR1).
  - `rocket_ascent.py` l.2731 passes `heading_stop = LAUNCH_AZIMUTH` to `earth_rotation.achieved_inclination_from_local_state` at the stop latitude. Latitude follows the great circle (`get_latitude_from_downrange`); the heading does not.
  - gt_baseline, evaluated the same way at insertion (latitude 48.4°, 3 896 km downrange): 61.09°, a "drift" of +9.49°. With the great-circle heading at that latitude: 51.35° (−0.25°). At the launch latitude: 49.83° (−1.77°).
  - It is computed only on the legacy `run()` path and is not archived; the swarm architectures never compute it. The pmp archives carry no latitude row.
  - Blocks XR1 (Ch. 6 reporting the inclination gap and the lateral load, so that Ch. 7's Future Work stays true). Peak lateral load under thrust for gt_baseline: 21.5 kN (0.71 m/s², t = 820 s).
- **C12. FIXED 2026-10-04, uncommitted.** `_style.LAW_LABELS` and `CASE_LABELS` printed "Polynomial shooting" for `exp_shooting`; both now read "Exponential pitch", as Ch. 4 and `tab:showcase_laws` do. All 18 results figures were re-rendered into the thesis. Five changed: showcase laws, law ranking, arc structure and the accuracy trade carry the label. The fifth, `results_solve_cost.png`, was stale: it showed gt_apogee's search as 57 s, while the archive from the 2026-10-01 clean re-run records 61 s.
- **C13. `tab:architecture_cost` prints the indirect row's tail improvement as `0.00000--0.00067`** (found 2026-10-05, when `pmp_vacuum` moved to 750×1500).
  - `tables._pct_range` keeps enough decimals for the largest value, 6.7×10⁻⁴ % (`pmp_vacuum`), and the smallest, 1.0×10⁻⁷ % (`pmp_norot`), then rounds to zero.
  - Until 2026-10-05 all three were below 10⁻⁵ % and the cell read `<10^{-5}`.
  - Fix in `_pct_range`, e.g. print `$\le 0.00067$` when the low end rounds to zero, then re-render the tables.

## Closed 2026-10-01

- §2.8.2–2.8.3 removed and relocated; the atmosphere-exit claim corrected everywhere (thesis `2f951df`).
- Post-separation drag disclosed in `tab:loss_budget`; today's additions tightened (thesis `46062bb`).
- The regenerated cost values (0.016→0.017 h, 0.03→0.032 h) are quoted in no prose.
- §3.4 review (R1–R17 except R3) applied; thesis `b29f810`.
- §3.1–3.3 review (S1–S22) applied 2026-10-02; thesis `625ed26`.
- Ch. 3 cross-chapter de-duplication (A1–A11, B1–B2, C1–C17, O1–O2) applied 2026-10-02; thesis `6327c4e`, figure script `0b91c5f`.
  - The plan is in `~/.claude/plans/let-s-review-section-3-4-serialized-blanket.md`.
  - `fig_frame_strategy.py` was relabelled and re-rendered.
- Ch. 4 review applied 2026-10-02 (thesis `08ba055`, label fix `1e34ad8`); the spec is `dev-notes/ch4-implementation-spec.md`.
  - Ch. 4: A1–A3, A5–A7, B1–B8, B10, C2–C8, O1–O12, R1–R3, R5–R7, R9, R13, E1–E9, K1–K5.
  - Ch. 3: B4. Ch. 6: B7. Bib: `mahajan2025peg`, `orbiterwiki_peg`.
  - Excluded by the user: A4, A8, C8 (code), and anything in Ch. 1 or Ch. 2 (C1, B9).
  - Not implementable without the sources: T16.
  - Ch. 4 went from 6431 to 5270 words; the thesis from 35578 to 34442.

## Closed 2026-10-03

- Ch. 5 review, rounds 1 and 2, applied and pushed as thesis `12f255e`.
  - Decisions are stored on the "Chapter 5 Review" artifact (https://claude.ai/artifact/TyN3GYgPDrfmm3xadCy8KJ), db collection `reviews`, one doc per item ID.
  - Rejected: O1 (title), O10, X7, X8. Left as is: D10, E3. Parked: T17 (X6), T18 (O7).
  - Ch. 5 is now §5.1 (Vehicle, with 5.1.1 Engine Model, 5.1.2 Payload Fairing, 5.1.3 Baseline Mission) and §5.2 Experimental Design. Numerical settings moved to Ch. 3 §3.3.1.
