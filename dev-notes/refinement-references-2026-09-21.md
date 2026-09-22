# Citable support for the PMP refinement (`dev-notes/pmp_swarm_polish.py`) — 2026-09-21

> **Superseded 2026-09-22 by [refinement-bibliography-2026-09-22.md](refinement-bibliography-2026-09-22.md)**, which
> explains every reference in full and carries the BibTeX. This file is kept as the first-pass record.

Research notes only. **Nothing has been added to `Thesis_Bibliography_DB.bib`**, because thesis edits are on hold. Every
entry below was checked against Crossref or the publisher's metadata, or both. The *content* each one is cited for was
checked at the level shown in the "Checked" column. Read the paper before quoting it for anything more specific than
that.

## 1. What the refinement does, as claims that need support

| # | Claim | Where in the method |
|---|---|---|
| C1 | A global heuristic (the swarm) supplies the initial guess, and a local Newton-type solver on the necessary conditions produces the extremal | overall structure |
| C2 | The unknowns are the costate direction (normalised, on the unit sphere) and the arc durations. The conditions are the terminal orbit and the stationarity of the free burn and coast durations, expressed through the Hamiltonian at the arc junctions | Stage A system |
| C3 | The square system is solved by Levenberg–Marquardt (a damped, trust-region Gauss–Newton) with a finite-difference Jacobian | `newton()`, SciPy `least_squares(method="lm")` |
| C4 | When the coast hits its bound it is pinned there, and the stationarity condition becomes one-sided (KKT) | coast pinning |
| C5 | The Stage-1 parameter γ_p, which no costate condition governs, is found by stepping it and warm-starting each re-solve from the previous extremal (natural-parameter continuation) | Stage B |
| C6 | The limits: shooting is sensitive and ill-conditioned; a continuation ends where the arc structure changes (here the last burn shrinks to zero); an extremal satisfies first-order conditions only | evaluation / Chapter 6 caveats |

## 2. References by claim

"Bib" says whether the entry is already in `Thesis_Bibliography_DB.bib` (key given) or is new (BibTeX in §6).

Checked:
- **full text:** I read the paper.
- **abstract:** the abstract, or the publisher's summary.
- **secondary:** described by other sources or search snippets; the paper itself not read.
- **metadata:** only the bibliographic details were confirmed.

### C1: global heuristic, then local refinement

| Reference | Bib | Checked | Supports |
|---|---|---|---|
| Hecht & Botta 2023, *Acta Astronautica* 211 | `hecht2023pso` | **full text** (arXiv 2302.04128) | Closest precedent. The swarm minimises a weighted sum of squares of the boundary-condition residuals of the shooting problem. Its costates then seed single shooting with homotopy, solved by a trust-region nonlinear solver. Also reports that a rank-deficient shooting Jacobian makes the Newton steps stall (see C6). |
| Jiang, Baoyin & Li 2012, *JGCD* 35(1) | new `jiang2012practical` | secondary | Normalising the initial costates onto a unit hypersphere, and searching for them with PSO before the shooting solve. Supports the normalisation in C2. |
| Conway 2012, *JOTA* 152(2) | `Conway2012SurveyMethodsContinuousDynamicSystems` | abstract | Survey placing metaheuristics alongside collocation, nonlinear-programming and indirect methods; the general framing for a hybrid. |
| Pontani & Conway 2014, *JOTA* 162(1) | `pontani2014indirectswarming` | abstract | "Indirect swarming": PSO combined with the analytical necessary conditions, needing no starting guess. Note that PSO alone enforces the conditions here; the abstract mentions no local refinement. Cite it as the swarm half, not as a precedent for the polish. |
| Pontani & Conway 2015, *JGCD* 38(5) | `pontani2015indirectheuristic` | abstract | Same indirect-heuristic idea; a switching function decides the sequence and durations of the thrust and coast arcs (C2). |

### C2: unknowns and conditions for a burn–coast–burn with free durations

| Reference | Bib | Checked | Supports |
|---|---|---|---|
| Lu, Griffin, Dukeman & Chavez 2008, *JGCD* 31(6) | `lu2008rapid` | abstract | Exoatmospheric ascent of **two burns with an optimal coast between them**, solved by multiple shooting. The paper calls the problem highly sensitive and uses a **dogleg trust-region method, more robust than Newton–Raphson**. Supports C2, C3 and C6 together. |
| Pallone, Pontani & Teofilatto 2016, ICATT | `pallone2016` | **full text** | Upper stage with a coast arc, costate equations, and the free-final-time transversality condition H(t_f) + ∂Φ/∂t_f = 0 (their Eq. 45), solved by a heuristic. Cites Lu et al. 2008 for the burn–coast–burn case. |
| Gath & Calise 2001, *JGCD* 24(2) | `gath2001optimization` | metadata (already in bib) | Ascent optimisation with coast arcs. |
| Brown, Harrold & Johnson 1969, NASA CR-1430 | `brown1969rapid` | metadata (already in bib) | Classical multiple-burn optimisation. |
| Bryson & Ho 1975, *Applied Optimal Control* | new `bryson1975applied` | metadata | Textbook source for free-final-time and interior-point (junction) conditions, from which the duration-stationarity conditions are *derived*. Cite a derivation, not a quoted formula. |
| Longuski, Guzmán & Prussing 2014 | new `longuski2014optimal` | metadata | Aerospace textbook alternative to Bryson & Ho: multi-arc necessary conditions, coast arcs, primer vector. |

### C3: Levenberg–Marquardt on the shooting system

| Reference | Bib | Checked | Supports |
|---|---|---|---|
| Levenberg 1944, *Q. Appl. Math.* 2(2) | new `levenberg1944method` | metadata + abstract | The damped least-squares idea. |
| Marquardt 1963, *J. SIAM* 11(2) | new `marquardt1963algorithm` | metadata + abstract | The algorithm. |
| Moré 1978, LNM 630 | new `more1978levenberg` | metadata + abstract; **SciPy's documentation names it** | The implementation used. SciPy's `least_squares` documents `method='lm'` as "Levenberg–Marquardt as implemented in MINPACK" and cites Moré. It is a trust-region formulation, so it is the same algorithm class as the dogleg method in Lu et al. 2008. |
| Virtanen et al. 2020 (SciPy) | `virtanen2020scipy` | already in bib | The software. |
| Nocedal & Wright 2006 | `nocedal2006numerical` | already in bib | Levenberg–Marquardt as a trust-region method, finite-difference Jacobians, and KKT conditions (**C4**). |
| Colasurdo & Casalino 2012, Springer SOIA | new `colasurdo2012indirect` | secondary | The multi-point boundary-value problem of indirect trajectory optimisation, solved by Newton iteration, with homotopy for the starting guesses (C3 and C5). |

### C5: continuation in γ_p

| Reference | Bib | Checked | Supports |
|---|---|---|---|
| Allgower & Georg 2003, SIAM | new `allgower2003introduction` | metadata | Standard reference: predictor–corrector continuation, and natural-parameter versus pseudo-arclength continuation. |
| Keller 1977 | new `keller1977numerical` | secondary | Pseudo-arclength continuation, the fix if a fold appears. It was **not** needed here, because the family ends where the last burn vanishes rather than at a fold. |
| Trélat 2012, *JOTA* 154(3) | new `trelat2012optimal` | abstract | Continuation (homotopy) as the standard remedy for the initialisation difficulty of shooting in aerospace problems. Also conjugate-point theory (C6). |
| Pan, Ran, Zhao & Qing 2026, *Astrodynamics* 10(4) | new `pan2026review` | abstract | Recent review of homotopy methods for trajectory optimisation, indirect and direct. |
| Pan, Lu, Pan & Ma 2016, *JGCD* 39(8) | new `pan2016double` | metadata | Double-homotopy method for optimal control problems. Read before citing for anything specific. |
| Calise, Melamed & Lee 1998, *JGCD* 21(6) | new `calise1998design` | secondary | **Launch-ascent precedent** for continuation: starting from the vacuum optimal ascent and continuing to the atmospheric problem. |
| Bonalli, Hérissé & Trélat 2020, *IEEE TAC* 65(6) | new `bonalli2020optimal` | abstract | **Launch-vehicle precedent**: shooting combined with continuation for endo-atmospheric launch vehicles. |

### Stage-1 parameter outside the costate problem (supports treating γ_p separately)

| Reference | Bib | Checked | Supports |
|---|---|---|---|
| Pontani & Teofilatto 2014, *Acta Astronautica* 94(1) | new `pontani2014simple` | abstract | Close structural precedent. The flight-path angle at stage separation is *guessed* (an outer parameter). The last stage's steering comes from PSO with the Euler–Lagrange equations and the minimum principle. |
| Pontani 2014 (PSO ascent), Pontani 2013, Pontani & Teofilatto 2019 | `pontani2014particle`, `pontani2013ascent`, `pontani2019ascent` | already in bib | The same school's ascent work. `pontani2013ascent` verifies second-order conditions, which gives the C6 caveat that an extremal is not yet a proven optimum. |

### C6: limits

| Reference | Bib | Checked | Supports |
|---|---|---|---|
| Betts 1998, *JGCD* 21(2) | new `betts1998survey` | metadata + secondary | Survey: indirect shooting (Newton on the boundary-value problem) and its sensitivity to the costate guess. Check the exact wording before quoting it. |
| Lu et al. 2008 | `lu2008rapid` | abstract | "Highly sensitive" burn–coast–burn problem (see C2). |
| Hecht & Botta 2023 | `hecht2023pso` | full text | A rank-deficient shooting Jacobian makes the Newton steps stall. This is the same mechanism as the refinement's measured condition numbers, 10⁷–10¹⁰ (the 250×1000 step-5 slide). |
| Pesch 1994, *Control and Cybernetics* 23(1/2) | new `pesch1994practical` | metadata + secondary | Practical multiple shooting plus homotopy for real-life problems, including changes of switching structure. |
| Bonnans & Hermant 2008, *ESAIM: COCV* 14(4) | new `bonnans2008stability` | secondary | Continuation across **changes in arc structure** (arcs appearing or vanishing), proved for state-constrained problems. Cite as an analogy for the family ending when the last burn reaches zero, not as the same situation. |
| Stoer & Bulirsch 2002 | new `stoer2002introduction` | metadata | Textbook: single versus multiple shooting and their conditioning. Supports "multiple shooting would condition this better". |

## 3. Minimum set for one defensible paragraph

- **"PSO supplies the initial guess and a local solve on the necessary conditions refines it"**: `hecht2023pso`, `jiang2012practical`, `Conway2012SurveyMethodsContinuousDynamicSystems`.
- **"Burn–coast–burn with free durations and the Hamiltonian junction conditions"**: `lu2008rapid`, `pallone2016`, `bryson1975applied`.
- **"Solved by Levenberg–Marquardt (MINPACK, via SciPy)"**: `levenberg1944method`, `marquardt1963algorithm`, `more1978levenberg`, `virtanen2020scipy`.
- **"γ_p by natural-parameter continuation"**: `allgower2003introduction`, `trelat2012optimal`; ascent precedents `calise1998design`, `bonalli2020optimal`.
- **"The Stage-1 parameter sits outside the costate problem"**: `pontani2014simple`.
- **Caveats**: `betts1998survey` (sensitivity), `pontani2013ascent` (second-order conditions not checked), `bonnans2008stability` (structure change ends the family).

That is **12 new entries** and 6 already in the bib. §6 has BibTeX for all 19 new candidates.

## 4. What the literature does *not* give (be explicit in the thesis)

- **No single precedent for the whole combination.** I found no source that does exactly this for a launch ascent: swarm, then Levenberg–Marquardt on the duration-stationarity conditions, then continuation in the Stage-1 kick parameter. Each component is established; the combination is this thesis's. Say so, rather than cite a paper as if it did the same thing.
- **The duration-stationarity conditions are derived, not quoted.** The conditions are H_coast_end = 0, H_burn1_end = H_last_burn_start and H_burn_end < 0; the method deliberately carries no mass costate. They are derived from the free-final-time and interior-point theory (Bryson & Ho; Longuski et al.). Lu et al. 2008 solve the same burn–coast–burn structure, but I have not confirmed that they write the conditions in this form. Present them as a derivation.
- **The end of the family (last burn → 0) is this thesis's observation.** Bonnans & Hermant and Pesch give the general phenomenon of arc structure changing along a continuation, not this case.
- **The coast-to-apoapsis result is a property of the problem as posed** (the unprojected rotation credit), not of the method. No citation makes it acceptable; it stays a Chapter 6 disclosure.

## 5. The two claims the 19b handoff flagged

- **§4.4, `pontani2014indirectswarming`.** At the level I could reach (the abstract; the publisher's full text is paywalled), it is PSO combined with the analytical necessary conditions and needs no starting guess. It does **not** show a local refinement after the swarm. Anything more specific attributed to it still needs the full text.
- **§5.4, attributing the spherical-angle parameterisation.** Jiang, Baoyin & Li 2012 is the usual source for normalising the costates onto a unit hypersphere and searching for them with PSO (secondary-level check). I did **not** confirm that they parameterise that sphere with angles. Attribute the *normalisation* to them and present the angle coordinates as this work's choice, unless the full text shows otherwise.

## 6. BibTeX for the new entries

```bibtex
@article{jiang2012practical,
  author  = {Jiang, Fanghua and Baoyin, Hexi and Li, Junfeng},
  title   = {Practical Techniques for Low-Thrust Trajectory Optimization with Homotopic Approach},
  journal = {Journal of Guidance, Control, and Dynamics},
  volume  = {35}, number = {1}, pages = {245--258}, year = {2012},
  doi     = {10.2514/1.52476}
}
@book{bryson1975applied,
  author    = {Bryson, Arthur E., Jr. and Ho, Yu-Chi},
  title     = {Applied Optimal Control: Optimization, Estimation, and Control},
  publisher = {Hemisphere}, address = {Washington, DC}, year = {1975},
  note      = {Reprinted by Routledge, 2018, doi:10.1201/9781315137667}
}
@book{longuski2014optimal,
  author    = {Longuski, James M. and Guzm{\'a}n, Jos{\'e} J. and Prussing, John E.},
  title     = {Optimal Control with Aerospace Applications},
  publisher = {Springer}, address = {New York}, year = {2014},
  series    = {Space Technology Library},
  doi       = {10.1007/978-1-4614-8945-0}
}
@article{levenberg1944method,
  author  = {Levenberg, Kenneth},
  title   = {A Method for the Solution of Certain Non-Linear Problems in Least Squares},
  journal = {Quarterly of Applied Mathematics},
  volume  = {2}, number = {2}, pages = {164--168}, year = {1944},
  doi     = {10.1090/qam/10666}
}
@article{marquardt1963algorithm,
  author  = {Marquardt, Donald W.},
  title   = {An Algorithm for Least-Squares Estimation of Nonlinear Parameters},
  journal = {Journal of the Society for Industrial and Applied Mathematics},
  volume  = {11}, number = {2}, pages = {431--441}, year = {1963},
  doi     = {10.1137/0111030}
}
@incollection{more1978levenberg,
  author    = {Mor{\'e}, Jorge J.},
  title     = {The {L}evenberg--{M}arquardt Algorithm: Implementation and Theory},
  booktitle = {Numerical Analysis},
  editor    = {Watson, G. A.},
  series    = {Lecture Notes in Mathematics}, volume = {630},
  publisher = {Springer}, address = {Berlin, Heidelberg}, year = {1978},
  pages     = {105--116},
  doi       = {10.1007/BFb0067700}
}
@incollection{colasurdo2012indirect,
  author    = {Colasurdo, Guido and Casalino, Lorenzo},
  title     = {Indirect Methods for the Optimization of Spacecraft Trajectories},
  booktitle = {Modeling and Optimization in Space Engineering},
  series    = {Springer Optimization and Its Applications},
  publisher = {Springer}, address = {New York}, year = {2012},
  pages     = {141--158},
  doi       = {10.1007/978-1-4614-4469-5_6}
}
@book{allgower2003introduction,
  author    = {Allgower, Eugene L. and Georg, Kurt},
  title     = {Introduction to Numerical Continuation Methods},
  publisher = {Society for Industrial and Applied Mathematics}, address = {Philadelphia}, year = {2003},
  series    = {Classics in Applied Mathematics},
  doi       = {10.1137/1.9780898719154}
}
@incollection{keller1977numerical,
  author    = {Keller, Herbert B.},
  title     = {Numerical Solution of Bifurcation and Nonlinear Eigenvalue Problems},
  booktitle = {Applications of Bifurcation Theory},
  editor    = {Rabinowitz, Paul H.},
  publisher = {Academic Press}, address = {New York}, year = {1977},
  pages     = {359--384}
}
@article{trelat2012optimal,
  author  = {Tr{\'e}lat, Emmanuel},
  title   = {Optimal Control and Applications to Aerospace: Some Results and Challenges},
  journal = {Journal of Optimization Theory and Applications},
  volume  = {154}, number = {3}, pages = {713--758}, year = {2012},
  doi     = {10.1007/s10957-012-0050-5}
}
@article{pan2026review,
  author  = {Pan, Binfeng and Ran, Yunting and Zhao, Mengxin and Qing, Wenjie},
  title   = {Review of Homotopy Methods for Aerospace Trajectory Optimization},
  journal = {Astrodynamics},
  volume  = {10}, number = {4}, pages = {507--536}, year = {2026},
  doi     = {10.1007/s42064-026-0315-7}
}
@article{pan2016double,
  author  = {Pan, Binfeng and Lu, Ping and Pan, Xun and Ma, Yangyang},
  title   = {Double-Homotopy Method for Solving Optimal Control Problems},
  journal = {Journal of Guidance, Control, and Dynamics},
  volume  = {39}, number = {8}, pages = {1706--1720}, year = {2016},
  doi     = {10.2514/1.G001553}
}
@article{calise1998design,
  author  = {Calise, Anthony J. and Melamed, Nahum and Lee, Seungjae},
  title   = {Design and Evaluation of a Three-Dimensional Optimal Ascent Guidance Algorithm},
  journal = {Journal of Guidance, Control, and Dynamics},
  volume  = {21}, number = {6}, pages = {867--875}, year = {1998},
  doi     = {10.2514/2.4350}
}
@article{bonalli2020optimal,
  author  = {Bonalli, Riccardo and H{\'e}riss{\'e}, Bruno and Tr{\'e}lat, Emmanuel},
  title   = {Optimal Control of Endoatmospheric Launch Vehicle Systems: Geometric and Computational Issues},
  journal = {IEEE Transactions on Automatic Control},
  volume  = {65}, number = {6}, pages = {2418--2433}, year = {2020},
  doi     = {10.1109/TAC.2019.2929099}
}
@article{pontani2014simple,
  author  = {Pontani, Mauro and Teofilatto, Paolo},
  title   = {Simple Method for Performance Evaluation of Multistage Rockets},
  journal = {Acta Astronautica},
  volume  = {94}, number = {1}, pages = {434--445}, year = {2014},
  doi     = {10.1016/j.actaastro.2013.01.013}
}
@article{betts1998survey,
  author  = {Betts, John T.},
  title   = {Survey of Numerical Methods for Trajectory Optimization},
  journal = {Journal of Guidance, Control, and Dynamics},
  volume  = {21}, number = {2}, pages = {193--207}, year = {1998},
  doi     = {10.2514/2.4231}
}
@article{pesch1994practical,
  author  = {Pesch, Hans Josef},
  title   = {A Practical Guide to the Solution of Real-Life Optimal Control Problems},
  journal = {Control and Cybernetics},
  volume  = {23}, number = {1/2}, pages = {7--60}, year = {1994}
}
@article{bonnans2008stability,
  author  = {Bonnans, J. Fr{\'e}d{\'e}ric and Hermant, Audrey},
  title   = {Stability and Sensitivity Analysis for Optimal Control Problems with a First-Order State Constraint and Application to Continuation Methods},
  journal = {ESAIM: Control, Optimisation and Calculus of Variations},
  volume  = {14}, number = {4}, pages = {825--863}, year = {2008},
  doi     = {10.1051/cocv:2008016}
}
@book{stoer2002introduction,
  author    = {Stoer, Josef and Bulirsch, Roland},
  title     = {Introduction to Numerical Analysis},
  edition   = {3rd},
  publisher = {Springer}, address = {New York}, year = {2002},
  series    = {Texts in Applied Mathematics},
  doi       = {10.1007/978-0-387-21738-3}
}
```

## Sources consulted

- Crossref metadata API for every DOI above.
- SciPy `least_squares` documentation.
- arXiv 2302.04128 (Hecht & Botta preprint, full text).
- ESA Indico ICATT 2016 PDF (Pallone et al., full text).
- Publisher pages (Springer, AIAA, ScienceDirect, IEEE through Crossref).
- Search-engine summaries for the entries marked "secondary".
