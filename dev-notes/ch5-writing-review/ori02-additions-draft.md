# ORI-02: additions to Chapters 2 and 4 so that Chapter 5's option tables have a place to point to (DRAFT, nothing written to the thesis)

Checked 2026-11-22 against the current source. Items of Tables `cap_rocket` and `cap_guidance` and where they are described:

| Option named in Chapter 5 | Where it is described now | Action |
|---|---|---|
| Ambient-pressure engine model | Ch 2 §2.1 Thrust (Eqs. thrust_pressure, thrust_pressure_alt) | none |
| One or two impulsive burns to close the orbit | Ch 2 l.468 (single impulse at apoapsis; Hohmann pair) | none |
| Triangular kick, instantaneous kick | Ch 4 l.118-126, Appendix A.5 | none |
| Time-to-go estimators (rocket equation, PEG predictor) | Ch 4 (laws), Appendix A.7.1 | none |
| Update period, coefficient freeze | Ch 4 l.224, 271; Appendix A.7.2 (2 s, 10 s) | none |
| Sea-level, vacuum, mean-value and ramp engine models | Appendix A.4 only | Draft 1 |
| Coast ends after a set duration or on reaching the target altitude | not described | Draft 2 |
| Burn ends at an optimized time or on the law's own time-to-go | implied by `t_go` in Ch 4 l.192, never stated | Draft 3 |

One tension to decide: the writing rules say implementation options belong in Chapter 5 or Appendix A, not in the theory chapters.
Drafts 1 and 2 are therefore written as modeling choices, with no mention of the simulator. If you prefer, they can go in Chapter 5 §5.1 instead.

## Draft 1: Chapter 2, §2.1 Thrust, after the paragraph that ends "... as the vehicle climbs~\cite{sutton2017rocket}."

```latex
The altitude dependence of the first stage can also be modeled in less detail. The thrust and the specific impulse can be held at their sea-level values, at their vacuum values or at the mean of the two, or they can rise from the sea-level to the vacuum values with flight time, in steps of a set interval. These models do not need the ambient pressure, but they do not follow the altitude at which the thrust changes. The thrust and the specific impulse must follow the same model, since a mixed pair implies a mass flow that the engine does not have.
```

Source of the content: Appendix A.4 ("First-stage engine"), which says the same of the simulator.

## Draft 2: Chapter 2, after the sentence "Circularizing from the coast ellipse takes a single impulse at its apoapsis, ..." (l.468)

```latex
The coast that precedes it can be ended in two ways: after a chosen duration, or when the vehicle reaches the target altitude, at the latest at its apoapsis, where the circularizing impulse is applied.
```

Source of the content: the coast-end rule of the reference-tracking flight (`REFERENCE_TRACK_COAST_MODE = "target_altitude"`: the coast ends where the flight climbs through the target altitude, or at its apoapsis if that is lower).

## Draft 3: Chapter 4, §4.2, at the end of the paragraph on closed-loop guidance (l.81)

```latex
A closed-loop law can also decide when its burn ends: its time-to-go reaches zero at the cut-off it plans, so the burn duration is a result of the law and not a choice. In the other case the duration is chosen from outside, for instance by an optimizer.
```

Source of the content: Ch 4 l.192 ("$t_{go}=t_f-t$ is the time remaining until cut-off") and Chapter 5 Table `cap_guidance` ("Burn end: at an optimized time, or on the law's own time-to-go").

## What Chapter 5 then does

Table `cap_rocket` and `cap_guidance` rows keep their options. ORI-02's original sentence (pointer to Appendix A) is added after Figure `ascent_variables`, as proposed on the page, and names Chapter 2 for the engine and coast options and Chapter 4 for the burn end.
