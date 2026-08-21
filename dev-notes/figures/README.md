# Thesis figure sources

One script per figure. Each writes a 300 dpi PNG named exactly as the
`\includegraphics` line in the thesis expects, so no LaTeX edit is needed when a
figure is regenerated. `figstyle.py` holds the shared palette, fonts and drawing
helpers — change it and all ten figures follow.

Every script names the `% FIG (TO DRAW)` or `% ARTWORK NOTE` block it was built
from, in its module docstring. Those blocks, in the thesis `.tex` files, are the
specification; this directory is only the implementation.

## Rendering

Output goes to `_preview/` (gitignored) unless `FIG_OUT` says otherwise:

```bash
C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe dev-notes/figures/fig_angle_conventions.py
```

To render everything straight into the thesis repository:

```bash
FIG_OUT="C:/Users/eduar/Desktop/Tese/Thesis_Overleaf/Figures" sh -c 'for f in dev-notes/figures/fig_*.py; do C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe "$f"; done'
```

`matplotlib` and `numpy` are enough for eight of the ten; `fig_segmented_schedule.py`
also needs `scipy`. All are in `pygmo-env`.

## The figures

| Script | Output | Placed in |
|---|---|---|
| `fig_angle_conventions.py` | `angle_conventions.png` | §2.4.2 |
| `fig_azimuth_correction.py` | `azimuth_correction.png` | §2.2.3 |
| `fig_enu_triad.py` | `ENU_Bate.png` | §2.2.2 |
| `fig_frame_strategy.py` | `frame_strategy.png` | §2.2.4 |
| `fig_mission_phase_timeline.py` | `mission_phase_timeline.png` | §2.8.2 |
| `fig_architecture_arc_structures.py` | `architecture_arc_structures.png` | §3.4 |
| `fig_flowchart.py` | `FlowChart.png` | §3.4.1 |
| `fig_segmented_schedule.py` | `segmented_schedule.png` | §3.4.4 |
| `fig_kick_profiles.py` | `kick_profiles.png` | §4.2 |
| `fig_module_dependency_graph.py` | `module_dependency_graph.png` | ch. 5 |

## What is real and what is illustrative

Eight of the ten are definitional or structural: they state conventions, layouts
or dependencies, and there is nothing in them to get numerically wrong.

Two carry curves, and **neither is simulator output**:

- `fig_kick_profiles.py` — the `alpha` traces are the configured profiles
  exactly (`TIME_TO_START_KICK`, `DURATION_INITIAL_KICK`, `INITIAL_KICK_ANGLE`
  from `Input_File/simulation_parameters.py` §7). The `gamma` traces are a
  smooth gravity-turn shape anchored on the documented MECO flight-path angle.
  Integrating a literal −3° angle of attack held for 45 s over-turns badly —
  `gamma` goes negative before 160 s — because the simulator's triangular
  profile ramps *pitch*, not `alpha`.
- `fig_segmented_schedule.py` — an illustrative ascent shape. Altitude is
  plotted against time, not downrange: on a downrange axis both hand-offs and
  staging fall inside the first few per cent of the plot and the point of the
  figure disappears. The spec permits either.

Both should be rebuilt from a real run once the PSO regeneration owed from the
pseudo-force fix has happened.

## The results figures are elsewhere

The sixteen figures of the results chapter are **not** in this directory. They
are simulator output, built offline from the results-matrix batch by
`Tese/src/Plots/results_figures/`, and their filenames all begin `results_`:

```bash
FIG_OUT="C:/Users/eduar/Desktop/Tese/Thesis_Overleaf/Figures" C:/Users/eduar/miniforge3/envs/pygmo-env/python.exe -m Plots.results_figures.make_all
```

(run from `Tese/src`). The thesis reaches them through the `\resultsfig` macro
in `Thesis_Preamble.tex`, which draws a labelled placeholder when the PNG is not
there yet, so the chapter compiles before the batch has been run.

They share this directory's palette and rcParams — restated in
`results_figures/_style.py` rather than imported, because `dev-notes/` is not
part of the simulator and is not importable from it. **Change one and the other
does not follow**; the two files have to be edited together.
