# Session summary — 2026-09-10/11

Everything below is committed; the working tree is clean and the test suite passes at 113. This
session's commits run from `df7a781` to `f9a16cc`.

## Implemented

1. **MECO is now root-found** (`df7a781`). Main-engine cutoff is a proper solver event instead of
   a flag set inside the equations of motion, and both dispatchers share one Stage-1 routine.
   Every case now reaches burnout at exactly 120270.0 kg, where the scatter was up to 700 kg
   before. 15 tests.
2. **Paired-arm comparison tool** (`8bda29e`). [compare_arms.py](compare_arms.py) compares two
   results-matrix runs case by case.
3. **Classical PEG gravity term restored** (`2c9ab0c`). The steering now includes the term C its
   cited reference requires, and the burn-time estimate matches the reference term for term.
   Before, it opened Stage 2 at α = −97°.
4. **Apollo fixed** (`2c9ab0c`). It now subtracts the correct local-frame accelerations. When the
   polynomial asks for more thrust than exists, it gives the vertical channel priority, as the
   Apollo ascent programme did. 12 tests cover this fix and the PEG fix.
5. **Arc-1 targeting mismatch documented** (`2c9ab0c`). Before the coast, the closed-loop laws aim
   at the final orbit. By the user's decision this is recorded in the code and in CLAUDE.md, not
   changed.
6. **Open-loop tangent laws under the coast architecture** (`67441bd`). The swarm chooses their
   constants, and the law runs continuously through the coast. 25 tests; the worktree and
   CLAUDE.md are updated.

## Investigated and recorded

- **Transversality weight:** keep it at 10.0. With the weight at zero the swarm fails to reach
  orbit (`238437c`).
- **Killed runs:** the pseudo-force comparison runs died because Windows shut the session down,
  not because of a fault. One complete pair survived, showing a 41.9 kg difference (`d45029f`).
- **Guidance audit:** all nine laws were checked against Chapter 4 and their sources. The full
  record is in [guidance-audit-2026-09-10.md](guidance-audit-2026-09-10.md).
- **Bibliography checks:** the linear tangent law's collapse to α = 0 does not appear in the
  literature. Large steering angles at guidance start are something flight practice prevents,
  not something it reports.
- **Tangent-law performance:** both laws can reach 21.32 t, but at production budget the swarm
  finds only 20.41 t (linear) and 16.60 t (bilinear) (`eb260ad`, `f9a16cc`).

## On hold

**Waiting on the user's decision**

1. **How to close the tangent-law search gap.** The recommendation is to bound the angle of attack
   at ignition instead of the absolute pitch.
2. **Whether the exponential law should also run continuously through the coast** instead of
   restarting each burn arc.
3. **A common attitude-rate limit and a free kick time.** Both were recommended; neither was
   requested.

**Runs not yet flown**

4. **The force-model comparison:** 12 cases remain, and they should now use the fixed laws.
5. **The production PMP row** after the MECO fix, at 250×1000.
6. **The full 20-case re-fly.** The whole existing batch has been stale since the MECO fix. The
   tangent cases should wait for decision 1.

**Thesis text, on hold at the user's request**

7. **Chapter 4:**
   - Classical PEG: add the C term.
   - Apollo: the local-frame accelerations and the thrust-priority rule.
   - Both tangent laws: the open-loop form, continuous time, and the curvature parameter, plus
     the t_go column of the guidance-law summary table.
   - The arc-1 limitation paragraph, drafted in the audit note.
   - Smaller mismatches: the bilinear terminal-rate sign, where the costate normalisation is
     applied, and the unenforced sign condition on the Hamiltonian.
8. **Chapter 6:** the five prose edits listed in the handoff, and the transversality limitation
   with its number attached.

**Documented, no action planned**

- Some guidance state is still updated inside the equations of motion, a milder form of the MECO
  hazard.
- The legacy second-stage ignition event is still latched, about 49 ms late, and only on the
  brute-force path.
- Classical PEG's validity limit on this low thrust-to-weight stage.
- The small internal-gravity gaps shared by every law.
