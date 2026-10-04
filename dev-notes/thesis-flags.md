# Thesis flags: backlog

Started 2026-10-01. Items found while fixing the thesis, parked here so they do not interrupt the user's own edit list. Review them together once that list is done. New flags are appended. Thesis line numbers are as of Overleaf `46062bb`.

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
- **T9. The PMP swarms are presented as 250×1000, seed 42** (user decision 2026-10-01, R3: omit the exception).
  - In fact the PMP extremals come from seed-3 swarms, the atmospheric one at 750×1500 (`run_results_matrix.py` l.467, l.476).
  - Ch. 5's optimal-reference paragraph (§5.2.3) still states this, against `tab:pso_settings` ("seed 42"). Since `12f255e`, Ch. 6 §6.7 also points to `tab:pso_settings` for "the fixed seed".
  - Decide whether Ch. 5 keeps the sentence.
- **T10. CLOSED 2026-10-02 (Ch. 3 de-duplication, A4).** Ch. 4 §4.4.1 now gives the open-loop tangent form flown under `pso_coast` (`eq:tangent_openloop`) with its bounds, and l.374–375 names the tangent and exponential laws together.
- **T11. CLOSED 2026-10-02 (Ch. 4 review, A1/R3).** §4.4.4.2 now cites `mahajan2025peg`, added to the bib.
- **T12. §3.4.3.3 (Multi-arc integration) may overstate the corner conditions.**
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
- **T17. Two cases have no reference of their own environment** (Ch. 5 review X6, 2026-10-03).
  - `gt_norot` and `gt_sea_level_engine`. Ch. 5 §5.2.4 quotes the shortfall against "the reference of the same environment" and has no clause for them; Ch. 6 `tab:gt_results` handles them locally.
  - User decision: address it later by adding a reference for those cases, not by rewording.
  - 2026-10-04: plan in `dev-notes/pmp-references-750x1500-plan-2026-10-04.md`. A new `pmp_norot` reference (750×1500, five seeds, refined) covers `gt_norot`. The user chose not to build one for `gt_sea_level_engine`, so `tab:gt_results` keeps its local treatment of that case.
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
