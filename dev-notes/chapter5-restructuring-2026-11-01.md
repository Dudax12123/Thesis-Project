# Chapter 5 restructuring: review cards and status

Exported 2026-11-01 from the artifact page *Supervisor Annotation Review*, tab **Chapter 5 restructuring**. The page is https://claude.ai/artifact/Mw2HC2yUsWpezUBANTWUdt (private, owned by the account that published it). This file holds the whole tab so the work can continue from another account without the page.

## Where things stand

Thesis repo: `C:\Users\eduar\Desktop\Tese\Thesis_Overleaf` (Overleaf git bridge), head **d96ce1e** on `main`, nothing uncommitted. Two commits came from this work: **69cea40** (Chapter 6 opening and the 5.1 text) and **d96ce1e** (the baseline mission). Both are pushed. No compile was run, only a label sweep (`sweep.py`), which found no new missing or duplicate labels.

Chapter 6 now opens with a section *Experimental Setup* (label `sec:vehicle`) holding the stage table, the environment table, the lift, drag and path-constraint paragraphs, and the baseline mission as its last subsection. Chapter 5 (`Thesis_Methodology.tex`) still holds the engine-model and fairing subsections under a section heading, 5.2 Guidance Laws as Implemented, 5.3 Experimental Design and 5.4 Numerical Integration and Event Detection.

| Card | Subject | Your decision | Implementation status | Note |
| --- | --- | --- | --- | --- |
| [CH5-00](#ch5-00) | Dissolve Chapter 5: where each piece goes | Approved | Partly implemented | Opening paragraph of Chapter 6 and the 5.1 text (thesis 69cea40), baseline mission (d96ce1e). The rest follows the other cards. |
| [CH5-01](#ch5-01) | 5.1 before the subsections: what repeats Chapter 2, and the text at its new place | Approved | Implemented | Thesis 69cea40 (pushed to Overleaf). |
| [CH5-02](#ch5-02) | 5.1.3 Baseline mission to the start of Chapter 6 | Approved | Implemented | Thesis d96ce1e (pushed to Overleaf). |
| [CH5-03](#ch5-03) | Change the flown mission to a 280 km circular orbit | Rejected | Nothing to do | The mission stays a 500 km circular orbit. The 280 km re-flight is not done. |
| [CH5-04](#ch5-04) | Remove the engine-model and fairing subsections: what is already said elsewhere | Not answered | To implement once approved | Audit of the engine-model and fairing subsections with six insertions (I1 to I6). |
| [CH5-05](#ch5-05) | 5.2 Guidance Laws as Implemented: you did not say where it goes | Not answered | To implement once approved | Dissolve 5.2 into Chapter 4. |
| [CH5-06](#ch5-06) | 5.3 Experimental Design: parked, with what it ties to | Approved | Nothing to implement | 5.3 Experimental Design is parked. The card only records what it depends on. |
| [CH5-07](#ch5-07) | 5.4 Numerical Integration and Event Detection to the appendix | Approved | Approved, not yet implemented | 5.4 Numerical Integration and Event Detection to the appendix, plus one sentence in Chapter 4. |
| [CH5-08](#ch5-08) | The commented Software Implementation text | Not answered | To implement once approved | Delete the commented-out blocks of Thesis_Methodology.tex, keep a copy in dev-notes. |

**Resolved (answered):** CH5-00, 01, 02, 03, 06, 07. **Implemented:** CH5-01, CH5-02, and the parts of CH5-00 that they cover. **Still to implement:** CH5-07 (approved), and CH5-04, CH5-05, CH5-08 once you answer them. **Nothing to do:** CH5-03, CH5-06.

## Rules from the earlier work that apply

- Fetch Overleaf before editing (`git fetch`, read the incoming diff). Never handle the Overleaf password or token; mask it with `sed -E 's#//[^@]*@#//***@#'`.
- No LaTeX toolchain on this machine: verify with a `\ref` / `\label` / `\cite` sweep.
- The working copies are CRLF while the index is LF. Edit with scripts that detect the line ending, and put LaTeX in raw strings in a script file, never in a heredoc.
- An Implement request means: apply, commit and push in the same turn. Commit messages end with the Co-Authored-By line.
- Review cards show the whole affected text with additions green (`<ins>`) and removals red (`~~`).
- Do not republish the page over unsaved answers without asking.

## Cards

Review of Chapter 5, Implementation (2026-10-30). You proposed dissolving it: the vehicle and the mission move to the start of Chapter 6, the numerics go to the appendix, and Section 5.3 waits. Each card states what you proposed, what I think of it, and then **the text as it will read at its new place**, in full: <ins>green</ins> is added, red (`~~`) is removed, plain text is unchanged. Nothing is applied.

Line numbers are those of `Thesis_Methodology.tex` at Overleaf 6ff424a. Cross-references and citations are shown by their label (for example Table stage_params, Chapter ssec:thrust), because their numbers change with the move. Equations are shown as plain text.

Printed numbering: 5.1 Vehicle Configuration and Baseline Mission (5.1.1 First-Stage Engine Model, 5.1.2 Payload Fairing, 5.1.3 Baseline Mission), 5.2 Guidance Laws as Implemented, 5.3 Experimental Design, 5.4 Numerical Integration and Event Detection. Lines 175 to 393 hold the Software Implementation, Configuration and Usage text inside comment blocks, so they are not printed.

<a id="ch5-00"></a>

### CH5-00. Dissolve Chapter 5: where each piece goes

*Ch. 5, whole chapter · Structural · **Decision:** Approved · **Status:** Partly implemented*

**Done so far:** the Chapter 6 opening and the 5.1 text before the subsections (CH5-01) at thesis 69cea40, the baseline mission (CH5-02) at d96ce1e. The rest follows the cards below.

###### You proposed

Chapter 5 is not needed. Distribute its contents and delete some of it: 5.1 up to its subsections and 5.1.3 go to the start of Chapter 6, 5.3 is addressed later, 5.4 goes to the appendix.

##### My opinion

I agree. After the moves, what is left in the chapter is the guidance-laws section (25 lines) and the experimental design. Chapter 5 would be a short bridge between a theory chapter and the results, and the reader needs the vehicle and the mission only when the results start.

Three points to keep in view:

- **It reverses a rule of yours.** On 2026-10-03 you set Chapter 5 as "how and why" and Chapter 6 as "results only, pointing to Chapter 5". Chapter 6 would now open with a set-up. I would keep that set-up to one section so the results sections stay results.
- **One section, not two.** If 5.3 also ends up in Chapter 6 later, the opening would hold the vehicle, the mission and the cases. I propose one section, *Experimental Setup*. If 5.3 goes elsewhere, the section is retitled *Vehicle and Mission*.
- **Chapters renumber.** Results becomes Chapter 5 and the conclusions Chapter 6. Every cross-reference in the thesis is by label, so they follow; only typed numbers and the notes in my own memory and `CLAUDE.md` need updating.

##### Where each piece goes

| Piece | Lines | You decided | My proposal |
| --- | --- | --- | --- |
| Chapter opening | 4–6 | – | deleted with the chapter; the Chapter 6 opening is rewritten (below) |
| 5.1 text before the subsections | 12–96 | start of Chapter 6 | [CH5-01](#CH5-01) |
| 5.1.1 First-Stage Engine Model | 98–140 | not said | [CH5-04](#CH5-04): stays with the vehicle, in the new section |
| 5.1.2 Payload Fairing | 142–158 | not said | [CH5-04](#CH5-04): stays with the vehicle, shortened |
| 5.1.3 Baseline Mission | 160–172 | start of Chapter 6 | [CH5-02](#CH5-02), and [CH5-03](#CH5-03) for the 280 km orbit |
| 5.2 Guidance Laws as Implemented | 394–418 | not said | [CH5-05](#CH5-05): dissolved into Chapter 4 |
| 5.3 Experimental Design | 421–691 | later | [CH5-06](#CH5-06): parked, with what it ties to |
| 5.4 Numerical Integration and Event Detection | 693–725 | appendix | [CH5-07](#CH5-07) |
| Commented-out Software Implementation, Configuration, Usage | 175–393 | – | [CH5-08](#CH5-08) |

##### How Chapter 6 will start

Order in Chapter 6 after the move: the opening paragraph, then *Experimental Setup* (5.1 text, 5.1.1, 5.1.2, 5.1.3), then the existing results sections, which keep their order.

**Chapter 6 opening paragraph** (Thesis_Results.tex l.21–24):

~~Chapter implementation fixed the seventeen cases, the conditions held across them and the conventions under which they are reported (Chapter sec:experimental_design).~~ <ins>This chapter reports seventeen cases. Section sec:vehicle fixes the vehicle and the mission they fly, and the cases, the conditions held across them and the conventions under which they are reported are set out in Chapter sec:experimental_design.</ins> This chapter presents and discusses their results. Each section first presents its figures and tables and then, in its own discussion, compares the results with what the preceding chapters lead one to expect.

The last pointer stays until you decide 5.3: whether it ends up in Chapter 5 or in this chapter, only the words around it change.

##### What else is affected

- `Thesis.tex` line 206 (`\input{Thesis_Methodology}`) goes once the file is empty.
- The chapter list in Section 1.4 and the Abstract name no Chapter 5 by number; I found no typed chapter number there.

<a id="ch5-01"></a>

### CH5-01. 5.1 before the subsections: what repeats Chapter 2, and the text at its new place

*Thesis_Methodology.tex l.12–96 · Moderate · **Decision:** Approved · **Status:** Implemented*

**Implemented 2026-10-30, thesis 69cea40.**

###### You proposed

Move 5.1, up to its subsections, to the start of Chapter 6. Check whether some of it repeats Chapter 2. A reminder is fine and does not need a pointer on every sentence. The tables repeat information, but they present it in an organized and condensed way, so they are fine.

##### What I checked

I read each paragraph against Chapters 2 and 4.

| Paragraph | Lines | Chapter 2 holds | Verdict |
| --- | --- | --- | --- |
| Falcon-9-class vehicle, values editable in the code | 12–16 | nothing | keep |
| Table of stage parameters | 18–42 | nothing | keep |
| Nozzle exit area is derived, not sourced | 44–47 | the equation for A_e (ssec:thrust) | keep the first sentence, cut the second |
| Constant C_D | 49–51 | "C_D is held constant and lift neglected, for the reasons given in Chapter sec:vehicle" | keep: Chapter 2 points here for the reasons |
| Lift approximated as zero | 53–62 | that lift requires α ≠ 0 (l.154), and the pointer above | cut to the fact, as you decided. The reasons are in Chapters 2 and 4 (below) |
| No path constraint during the search | 64–68 | – | keep, one clause trimmed. The Guidance chapter (l.63–67) says the same in one sentence |
| Environment constants, Mach number, p_a(h) | 70–76 | only ρ(h) = ρ₀ exp(−h/h_s) | keep: the speed-of-sound profile and the pressure law are not in Chapter 2 |
| Table of environment constants | 78–96 | the symbols, not the values | keep |

##### My opinion

Only two places repeat something: the second sentence of the nozzle-area paragraph, which the engine-model subsection restates, and the future-work clause, which the Conclusions and the Guidance chapter already hold. As you decided, the lift paragraph keeps only the fact that the lift is zero. Its reasons are already given elsewhere: Chapter 2 says lift requires a nonzero angle of attack (l.154), and Chapter 4 holds the angle of attack at zero in the kick and the gravity turn (Guidance l.121, l.135; Optimization l.424). What is lost is the argument for the segmented law (dynamic pressure below 4 kPa at hand-off), which you chose not to carry.

##### The text at its new place

**Section: Experimental Setup** <ins>(new heading; label sec:vehicle kept on it)</ins>

The simulator is configured with parameters representative of a Falcon-9-class two-stage launcher presented in Table stage_params. The framework is not specific to this vehicle: the stage masses, engine parameters, reference area and drag coefficient can be edited in the code to describe any two-stage launcher, although they are not selectable in the configuration. The Falcon-9 values are used here as a concrete, well-documented baseline.

*Table stage_params. Launch vehicle stage parameters (NASA_Falcon9v12_Datasheet_2018, SpaceX_Falcon9_2025, Wevolver_Falcon9v12_Block5_2019).*

| Parameter | First Stage | Second Stage |
| --- | --- | --- |
| I_sp (sea level) | 283 s | – |
| I_sp (vacuum) | 311 s | 348 s |
| Maximum thrust (SL) | 7607 kN | – |
| Maximum thrust (V) | 8227 kN | 934 kN |
| Propellant | LOX/RP-1 | LOX/RP-1 |
| Propellant mass | 395 700 kg | 92 670 kg |
| Empty mass | 25 600 kg | 3 900 kg |
| Payload capacity (LEO) | – | 22 800 kg |
| Payload capacity (GTO) | – | 8 300 kg |
| Diameter | 3.7 m | 3.7 m |
| Drag coefficient | 0.3 | 0.3 |
| Reference area | 10.52 m² | 10.52 m² |
| Nozzle exit area | 6.12 m² | – |

The nozzle exit area is derived rather than sourced: Eq. nozzle_exit_area fixes it from the two first-stage thrust figures, at some 0.68 m² for each of the nine engines. ~~It is listed only because the pressure-dependent engine model of Section ssec:engine_model reads it, and editing either thrust figure moves it.~~

The drag coefficient of Eq. cd is held constant. This removes the need for a Mach-dependent aerodynamic database while keeping the altitude and speed dependence that dominates the drag loss.

The lift is approximated as zero and is not included in the simulation. ~~A rotationally symmetric vehicle produces lift only at a nonzero angle of attack (Turner, griffin2004space, sutton2017rocket), and every case of Chapter results_and_discussion flies its first stage, where the dynamic pressure is significant, as a gravity turn at α = 0. Ascent studies neglect the lift for the same reason (Ulrich, Morgado2022_MultistagePrelimDesignTrajectoryOpt). Only the segmented architecture can steer before staging (Chapter ssec:segmented). In the cases flown its explicit law takes over at or after staging, where the dynamic pressure is below 4 kPa, a tenth of its peak, so the lift that steering would produce is small and is approximated as zero as well.~~

No path constraint bounds the aerodynamic loads during the search. Every flight records instead the dynamic pressure, from which the drag follows, and the lateral force F⊥ of Chapter ssec:pseudo_forces, so a trajectory can be checked after the flight against the limits a user sets for the vehicle. ~~Imposing them in the search is left as future work (Chapter section:future).~~

The environment constants are those of Table env_params. A Mach number is reported but never enters the motion; its speed of sound comes from the standard-atmosphere temperature profile, a lapse of 6.5 K/km to 11 km and 216.65 K above (usstandard1976), which the isothermal density model of Chapter sec:atm_models cannot supply. The ambient pressure of the engine model decays with the same scale height, `p_a(h) = p_0 exp(−h/h_s)`, since pressure and density are proportional in an isothermal atmosphere.

*Table env_params. Environment constants; μ, R_E, ρ₀ and h_s from curtis2020orbital.*

| Parameter | Symbol | Value |
| --- | --- | --- |
| Gravitational parameter | μ | 3.986004418×10¹⁴ m³/s² |
| Equatorial radius | R_E | 6378 km |
| Rotation rate | ω_E | 7.2921159×10⁻⁵ rad/s |
| Standard gravity | g₀ | 9.81 m/s² |
| Sea-level pressure | p₀ | 101 325 Pa |
| Sea-level density | ρ₀ | 1.225 kg/m³ |
| Scale height | h_s | 8500 m |

The subsections that follow (engine model, fairing, mission) are in [CH5-04](#CH5-04) and [CH5-02](#CH5-02).

##### What else is affected

- **Chapter 2 l.170** says the lift is neglected "for the reasons given in Chapter sec:vehicle". The reasons given there are now only for the drag coefficient, so the sentence is reworded:

  In this work C_D is held constant, for the reasons given in Chapter sec:vehicle, <ins>and the lift is neglected</ins>. ~~and the lift is neglected, for the reasons given in Chapter sec:vehicle.~~

  The appendix (l.118) and the Conclusions (l.157) point here for the fact, which is still stated, and stay as they are.
- The moved text keeps its labels (`sec:vehicle`, `tab:stage_params`, `tab:env_params`), so the references from Chapters 2, 4, 7 and the appendix keep working.
- The Guidance chapter sentence "No path constraint is imposed: the loads they would bound are recorded along every flight instead (Chapter sec:vehicle), and imposing them is left as future work (Chapter section:future)" is the one place that keeps the future-work pointer. It is unchanged.

<a id="ch5-02"></a>

### CH5-02. 5.1.3 Baseline mission to the start of Chapter 6

*Thesis_Methodology.tex l.160–172 · Moderate · **Decision:** Approved · **Status:** Implemented*

**Implemented 2026-10-30, thesis d96ce1e.**

###### Your instruction

Move 5.1.3, the baseline mission, to the start of Chapter 6, where the mission flown is defined.

##### What Approve does

I move 5.1.3 to the end of *Experimental Setup* with the text shown below (500 km, as it is today).

##### My opinion

I agree. The mission is what the results answer to, and Chapter 6 already cites it for the figure of merit, the validation section and the Conclusions. It follows the vehicle in *Experimental Setup*: stage table, environment table, engine model, fairing, then the mission.

The paragraph needs one fix when it moves: "every case of Chapter 6 flies the baseline mission" is written inside Chapter 6 afterwards. The statement that the launch site and orbit are "set independently by the user" is about the code; I keep it. The orbit values are the current ones. [CH5-03](#CH5-03) decides whether 500 km becomes 280 km.

##### The text at its new place

**Subsection: Baseline Mission** (label ssec:baseline_mission kept; last subsection of *Experimental Setup*)

The launch site and target orbit are set independently by the user; every case ~~of Chapter results_and_discussion~~ <ins>in this chapter</ins> flies the baseline mission. It targets a 500 km circular orbit at an inclination of 51.6° (representative of the International Space Station), launched from latitude 28.5° (Kennedy Space Center). The resulting lift-off mass is approximately 517.9 t, being the sum of both stages with zero payload in the reference configuration. Setting the payload to zero is itself deliberate. With the delivered mass fixed at nothing, the propellant still unburnt at insertion becomes the single figure of merit separating one guidance law from another, and it converts directly into a payload capability through the rocket equation as soon as a specific mission is substituted.

##### What else is affected

- The label `ssec:baseline_mission` is cited in the Conclusions (l.16) and in the validation section (l.718, l.754). It moves with the text.

<a id="ch5-03"></a>

### CH5-03. Change the flown mission to a 280 km circular orbit

*Mission, Ch. 6 and the Abstract · Structural · **Decision:** Rejected · **Status:** Nothing to do*

##### Your instruction

Change the objective of the flown mission to a 280 km circular orbit. You are thinking about it.

##### What Approve does

Approve changes nothing in the thesis yet. It tells me to prepare the 280 km re-flight as a separate job. Please answer the three questions below in the comment box.

##### My opinion

I would do it, but as a separate job with its own decision on the payload, after the text is moved. The move does not depend on the number.

**For**

- It is the orbit Falcon 9 flies on the Starlink mission, so the validation section compares like with like. At 500 km the reference's first second-stage burn ends 123 s before the flown ones; flown to the Starlink orbit it ends 71 s before (Thesis_Results l.742–744).
- The Starlink-mission reference already exists as a side case of Section 6.8 (250×1000 swarm, coast bound 2500 s). It would become the baseline and the extra column of Table validation_missions would go.
- 500 km is far from what the vehicle flies, and the long coast that comes with it is where the reference's coast-bound problem sits (it grows to the bound, and above about 3500 s it dips to 5–38 km).

**Against**

- **Everything is re-flown.** The matrix is about 26 h. The three references are a 750×1500 swarm from five seeds each, with refinement, and the two segmented cases took 8 h and 6 h. That is several days of machine time.
- **Every number in the text changes**: Chapters 6 and 7, the Abstract, the Resumo and the Extended Abstract. The tables and figures regenerate from the archives; the prose does not.
- **The coast-bound question does not go away.** The Starlink-mission reference coasted 2093 s against a 2500 s bound, so the coast still grows with the bound. It has to be checked again for the references and the refined gravity turns.
- **The payload.** The zero payload is deliberate (CH5-02). The Starlink mission carries 15.6 t. If the baseline keeps zero payload at 280 km, the validation still differs from the flight by the payload.

##### What I need from you

1. 280 km with zero payload, or with the 15.6 t Starlink payload?
2. Keep 51.6° and latitude 28.5°, or write 53° for the Starlink match? The inclination enters no equation (Thesis_Results l.719–720), so it is a label. The launch latitude does matter, and 28.5° is already the Starlink site.
3. When: after the restructure, as its own re-flight.

##### The mission paragraph if you choose 280 km with zero payload

Only the orbit changes against CH5-02. The lift-off mass stays at 517.9 t. If you choose the 15.6 t payload, the lift-off mass sentence changes as well.

The launch site and target orbit are set independently by the user; every case in this chapter flies the baseline mission. It targets a ~~500~~ <ins>280</ins> km circular orbit at an inclination of ~~51.6°~~ <ins>53°</ins> ~~(representative of the International Space Station)~~ <ins>(that of the Starlink missions)</ins>, launched from latitude 28.5° (Kennedy Space Center). The resulting lift-off mass is approximately 517.9 t, being the sum of both stages with zero payload in the reference configuration. Setting the payload to zero is itself deliberate. With the delivered mass fixed at nothing, the propellant still unburnt at insertion becomes the single figure of merit separating one guidance law from another, and it converts directly into a payload capability through the rocket equation as soon as a specific mission is substituted.

The 53° and the Starlink wording are the alternative to keeping 51.6°. Say which you want.

##### What else is affected

If decided yes: `run_results_matrix.BASELINE`, `pmp_reference.npz` (rebuilt), the Chapter 6 set, the CLAUDE.md results paragraph, the thesis text above. If no: nothing.

<a id="ch5-04"></a>

### CH5-04. Remove the engine-model and fairing subsections: what is already said elsewhere

*Thesis_Methodology.tex l.12–72 (5.1.1, 5.1.2) · Structural · **Decision:** Not answered · **Status:** To implement once approved*

##### Your instruction

Remove the two subsections about the fairing (5.1.2) and the first-stage engine model (5.1.1). Before that, check whether everything they say is already said somewhere else; what is not, we decide where to insert.

##### What I found

I read each statement of the two subsections against the whole thesis. Of 15 statements, 5 are already said elsewhere and 10 are not, or not completely. The table has all of them; the insertions I propose for the missing ones follow it.

| # | Statement | Said elsewhere? | Proposed place |
| --- | --- | --- | --- |
| E1 | Thrust and I_sp switch at the staging events; the second stage flies its vacuum values, less the nozzle's pressure deficit in the cases that carry it | **No.** Section 5.3 mentions the pressure deficit for the references only | I4: Experimental Setup |
| E2 | Two first-stage models are flown | Yes: Chapter 2 l.136 names them; Chapter 6 (gravity turn, engine model) describes the sea-level one | – |
| E3 | The pressure-dependent model is flown in every case but one | Yes: the case table of 5.3 (`gt_sea_level_engine`) and Chapter 6 | I6 repeats it in one clause |
| E4 | I_sp scaled linearly with ambient pressure between its published endpoints; ṁ_p = T/(g₀ I_sp) | **No.** Chapter 2 l.134 derives I_sp(h) = T(h)/(ṁ_p g₀) with a constant ṁ_p, which is not what is flown | I5: Chapter 2, thrust |
| E5 | The published ratios (0.925 and 0.910) differ, so the mass flow falls by 1.6 % over the burn | **No.** Chapter 2 l.134 says the mass flow is held at the vacuum value, which contradicts it | I5: Chapter 2, thrust |
| E6 | It is the only model under which the pressure loss is defined | Yes: Chapter 2 l.409–414 defines the loss as the thrust deficit T_vac − T(h) | – |
| E7 | The sea-level model holds the lift-off values, is the most conservative constant choice, ends the burn earliest and delivers the least impulse | **No.** Chapter 6 (l.369) cites this very sentence | I6: Chapter 6, engine model |
| E8 | Table of the two models, with the burn times 146.0 s and 144.4 s | **No.** Only the figure of Chapter 6 shows the curves; Chapter 6 l.368 cites the table | I6: Chapter 6, engine model |
| F1 | The fairing weighs 1900 kg | Only in the Extended Abstract, not in the thesis | I1: Experimental Setup |
| F2 | It is booked within the first-stage empty mass; separation drops only the rest | **No** | I1: Experimental Setup |
| F3 | Jettison at 65 km, atmosphere taken as left | Yes: Chapter 2 l.180 (the criterion), Chapter 4 events table, Chapter 6 validation | – |
| F4 | The altitude is preferred because it is the same for every trajectory | Yes: footnote of the Chapter 4 events table, in full | – |
| F5 | The criterion governs the jettison alone: drag is modeled throughout the first stage | **No.** The footnote speaks of "the atmosphere-exit criterion" and could be read as ending the drag at 65 km | I3: Chapter 4 footnote |
| F6 | The jettison is an instantaneous mass drop at the crossing, handled as an arc boundary like staging | **No.** The events table lists stage separation as an "inert-mass drop" but not the fairing as one | I2: Chapter 4 events table |
| F7 | If the first stage burns out below 65 km, the drop is applied at the next arc boundary | Only its effect, for the reference, in the Chapter 6 validation (l.871) | I2: Chapter 4 events table |

Two further findings, neither caused by this move:

- **Chapter 2 and the flown model disagree.** Chapter 2 says the mass flow is "held at the value given by the vacuum figures". The simulator flies a mass flow that falls by 1.6 %, which only this subsection says. I5 repairs Chapter 2. Without it, removing the subsection would leave the thesis stating the idealised model as the flown one.
- **The Extended Abstract** says the first stage burns "at a fixed mass flow in every case, so main-engine cut-off falls at the same time, 146 s". The sea-level case cuts off at 144.4 s. That is the Extended Abstract's own sentence (`ExtendedAbstract_imple.tex` l.81), not part of this card; I list it so you can decide whether it needs a word.

##### What Approve does

I delete 5.1.1, 5.1.2 and the then-empty section heading of Chapter 5, make the six insertions below (I3 and I4 only if you keep them, see the notes), and retarget the references to the two deleted labels. Nothing else changes.

##### The text at its new place

**I1. Experimental Setup**, straight after Table stage_params (Thesis_Results.tex):

<ins>The 1900 kg payload fairing (NASA_Falcon9v12_Datasheet_2018) is booked within the first-stage empty mass of Table stage_params rather than added to it; stage separation drops only the rest of that mass.</ins>

The nozzle exit area is derived rather than sourced: Eq. nozzle_exit_area fixes it from the two first-stage thrust figures, at some 0.68 m² for each of the nine engines.

**I4. Experimental Setup**, same place, after the paragraph above (optional, see below):

<ins>Thrust and specific impulse switch to the second-stage values at stage separation. The second stage flies its vacuum values, less the pressure deficit of its nozzle in the cases that carry it (Chapter ssec:case_definitions).</ins>

**I5. Chapter 2, Thrust** (Thesis_Ascent_Background.tex l.134–136). The paragraph keeps its derivation and gains the flown model:

The mass flow is the quantity that throttling would vary. Here it is held at the value given by the vacuum figures, `ṁ_p = T_vac/(I_sp0 g₀)` with I_sp0 the vacuum specific impulse: a fixed-geometry engine at fixed throttle consumes propellant at a constant rate, since the flow through a choked throat cannot be increased by lowering the pressure downstream of it (sutton2017rocket, pp. 58–59), and the ambient pressure changes only the thrust it delivers. The specific impulse therefore follows the thrust, `I_sp(h) = T(h)/(ṁ_p g₀)`, rising from its sea-level to its vacuum value as the vehicle climbs (sutton2017rocket). <ins>The published figures do not satisfy this exactly. A constant mass flow requires `T_SL/T_vac = I_sp,SL/I_sp,vac`, and the published figures give 0.925 and 0.910. They are kept as published, so the simulator scales the specific impulse linearly with the ambient pressure between its published endpoints,</ins>

<ins>`I_sp(h) = I_sp,vac − (p_a(h)/p_0) (I_sp,vac − I_sp,SL)`</ins>   <ins>(Eq. isp_pressure)</ins>

<ins>with the mass flow following as `ṁ_p = T/(g₀ I_sp)`; it falls by 1.6 % over the burn.</ins> The simulator also offers a simpler model, with thrust and specific impulse held at their sea-level values; ~~the two models flown are described in Chapter ssec:engine_model~~ <ins>it is flown in one case, described in Chapter ssec:gt_engine_model</ins>.

**I6. Chapter 6, Gravity Turn, Stage-1 Engine Model** (Thesis_Results.tex l.298–306). The subsection gains the description of the model and the table:

The last variation flies the first stage at constant sea-level thrust and specific impulse (Chapter ~~ssec:engine_model~~ <ins>ssec:thrust</ins>). <ins>Every other case flies the pressure-dependent model. The sea-level model holds the lift-off values for the whole burn. It is the most conservative constant choice: with the same propellant and the highest mass flow, it ends the first-stage burn earliest and delivers the least impulse. Table engine_models gives the two models.</ins> Figure fig:gt_engine plots the first stage's (a) thrust, (b) specific impulse and (c) mass flow under both models, with each main-engine cut-off dotted.

<ins>*Table engine_models. The two first-stage engine models.*</ins>

| | Pressure-dependent (baseline) | Sea level, constant |
| --- | --- | --- |
| Thrust | 7.61 MN at lift-off to 8.23 MN in vacuum | 7.61 MN throughout |
| Specific impulse | 283 to 311 s | 283 s throughout |
| Mass flow | 2741 to 2698 kg/s (−1.6 %) | 2741 kg/s throughout |
| First-stage burn | 146.0 s | 144.4 s |
| Pressure loss | defined | not defined |

And the discussion sentence at l.366–371, which cited the deleted subsection:

With constant sea-level thrust and specific impulse the gravity turn keeps 2.96 t less. Its first-stage burn ends 1.6 s earlier and delivers less impulse (Table engine_models, Figure fig:gt_engine), and the flight compensates with a steeper kick, which raises the gravity loss by 97 m/s. That the sea-level model costs propellant is what ~~Chapter ssec:engine_model~~ <ins>its description above</ins> anticipates of the most conservative constant model; the run adds the size of the cost.

**I2. Chapter 4, Table events** (Thesis_Optimization_Background.tex l.355), the atmosphere-exit row:

| Event | Fires when | Role |
| --- | --- | --- |
| Atmosphere exit | h = 65 km | Fairing jettison (Chapter ~~ssec:fairing_jettison~~ <ins>ssec:fairing</ins>) <ins>as an instantaneous mass drop, applied at the next arc boundary if the first stage ends below this altitude</ins> |

**I3. Chapter 4, footnote of the same table**, one sentence added at its end (optional, see below):

The altitude is adopted as the atmosphere-exit criterion so that the fairing is always jettisoned at the same point of the ascent, whatever the trajectory flown. A load- or heating-based threshold would move the jettison with the variables being optimized and compromise the comparison between results (Chapter ~~ssec:fairing_jettison~~ <ins>ssec:fairing</ins>). <ins>The criterion governs the jettison alone: drag is modeled throughout the first-stage flight.</ins>

##### The text that is removed

**5.1.1 First-Stage Engine Model** (l.12–54)

~~Thrust and specific impulse switch at the staging events, and the second stage flies its vacuum values, less the pressure deficit of its nozzle in the cases of Chapter ssec:case_definitions that carry it. The first stage is flown with one of two models of the ambient-pressure dependence of Chapter ssec:thrust, summarized in Table engine_models.~~

~~The pressure-dependent model, flown in every case but one, evaluates Eq. thrust_pressure_alt with the nozzle exit area of Table stage_params. As in Chapter ssec:thrust, its specific impulse is linear in the ambient pressure, and is scaled between its published endpoints, `I_sp(h) = I_sp,vac − (p_a(h)/p_0)(I_sp,vac − I_sp,SL)` (Eq. isp_pressure), with the mass flow following as `ṁ_p = T/(g₀ I_sp)`. A constant mass flow requires `T_SL/T_vac = I_sp,SL/I_sp,vac`; the published figures give 0.925 and 0.910. They are kept as published, so the mass flow falls by 1.6 % over the burn. This is the only model under which the pressure loss of Chapter sec:performance is defined.~~

~~The sea-level model holds the lift-off values for the whole burn. It is the most conservative constant choice: with the same propellant and the highest mass flow, it ends the first-stage burn earliest and delivers the least impulse.~~

~~Table engine_models: the two first-stage engine models (moves to Chapter 6, I6).~~

**5.1.2 Payload Fairing** (l.56–72)

~~The 1900 kg payload fairing (NASA_Falcon9v12_Datasheet_2018) is booked within the first-stage empty mass of Table stage_params rather than added to it; stage separation drops only the rest of that mass. It is jettisoned, and the atmosphere taken as left, as the vehicle climbs through 65 km (kluever2018space). Of the criteria of Chapter ssec:fairing, the altitude is preferred because it is the same for every trajectory, whereas a load- or heating-based threshold would move the jettison with the variables being optimized. The criterion governs the jettison alone: drag is modeled throughout the first-stage flight, and the exo-atmospheric laws engage at second-stage ignition (Chapter sec:numerical_methods).~~

~~The jettison is an instantaneous drop in mass at the altitude crossing, handled as an arc boundary, like staging (Chapter ssec:integration, BettsSurvey1998). If the first stage burns out below 65 km, the drop is applied at the next arc boundary, stage separation or second-stage ignition, so the fairing is never carried under thrust above that altitude.~~

##### Notes on the choices

- **I3 is optional.** Its only purpose is to stop the footnote being read as "drag ends at 65 km". The second half of the removed sentence, that the exo-atmospheric laws engage at second-stage ignition, is already in Chapter 4 (the first stage is a gravity turn, the law steers the second), so I drop it.
- **I4 is optional.** It says where the second stage's thrust comes from. If you prefer, it goes into Section 5.3 with the other second-stage facts instead.
- I dropped "so the fairing is never carried under thrust above that altitude" and the citation BettsSurvey1998 (a general reference on arc boundaries). Say if you want either back in I2.

##### What else is affected

Retargeted references (labels that disappear):

- `ssec:engine_model`, cited from Chapter 2 l.136 (now rewritten in I5), the appendix l.65 and l.122, Chapter 6 l.301 and l.370 (I6), Chapter 6 l.860 (validation) and Section 5.3. All go to `ssec:thrust` or `ssec:gt_engine_model`.
- `ssec:fairing_jettison`, cited from the appendix l.66 and l.129, Chapter 6 l.872, and the Chapter 4 table and footnote. All go to `ssec:fairing`.
- `tab:engine_models` moves to Chapter 6 with its label.
- `ssec:fairing_jettison` is also cited from Section 5.3 (l.347); it goes to `ssec:fairing`.

<a id="ch5-05"></a>

### CH5-05. 5.2 Guidance Laws as Implemented: you did not say where it goes

*Thesis_Methodology.tex l.394–418 · Moderate · **Decision:** Not answered · **Status:** To implement once approved*

##### Your instruction

You did not mention Section 5.2, Guidance Laws as Implemented. Without Chapter 5 it has no home, so **this card is my proposal, not yours**.

##### What Approve does

I dissolve the section into Chapter 4, as shown below: one paragraph added to the chapter opening, one sentence added to each of two laws, and one sentence deleted.

##### My proposal and why

Dissolve it into Chapter 4. The section is 25 lines and has no home once the chapter is gone. Its four paragraphs go to three places in `Thesis_Guidance.tex`, and one is deleted. The destination texts are below.

Moving the constant-pitch-rate paragraph shows something to decide: Chapter 4 presents that law under *Steering during the Atmospheric Arc* and describes it as shaping the atmospheric segment, while the simulator flies it from second-stage ignition. The paragraph says so, and it also needs to be there. If you would rather keep Chapter 4 as a description of the law alone, the paragraph goes to the appendix with the CPR variants (Thesis_Appendix_Capabilities l.238).

##### The text at its new place

**1. Opening of Chapter 4** (Thesis_Guidance.tex l.13–15; the stateless-function paragraph):

This chapter presents the guidance laws compared in this work. Each law is the command that flies the vehicle along the trajectories of Chapter ascent_background, and is given as derived in the literature. <ins>Each law is a stateless function: given the state, the target and a time-to-go, it returns a steering command, and whatever memory it needs is held by the simulation core and handed back at its next call. This makes the laws interchangeable within the simulation and the comparison between them fair. Appendix chapter:appendix_guidance_options describes the options not used in this work: further time-to-go estimators, kick programs, and variants of several laws.</ins>

**2. Gravity Turn** (l.130–145): the paragraph "The gravity turn commands α = 0 after the kick for the remainder of the flight, exactly as derived." is deleted. Chapter 4 already says that the angle of attack is kept close to zero.

~~The gravity turn commands α = 0 after the kick for the remainder of the flight, exactly as derived.~~

**3. Constant Pitch Rate** (l.146–166), after the ramp equation `θ_cmd(t) = max(θ_init − θ̇ (t − t_start), 0)`, `α = θ_cmd − γ` (Eq. cpr) and "the clamp holding the command at the local horizontal once the ramp reaches it.":

<ins>In the simulator the law engages at second-stage ignition, with θ_init equal to the flight-path angle there, and the pitch rate θ̇ is its one free constant (Table law_constants). The ramp therefore shapes the second-stage arc rather than the climb through the atmosphere. The apogee-circularization form of the law, presented in Appendix sec:app_cpr, ramps from the vertical and so steers from the atmospheric arc until insertion.</ins>

**4. Exponential Pitch Law** (l.233–241), after "The constants (a, b) are the free constants of the law.":

<ins>They are searched by the optimizer (Table law_constants). Architectures that do not optimize them fix them by a shooting solve (Appendix sec:app_exp_shooting).</ins>

##### What else is affected

- `sec:laws_implemented` is cited only from the commented-out text. `tab:law_constants` is in Chapter 4 and stays.
- I have not checked every sentence of Chapter 4 against these four; if it already says one, that one goes as a repeat.

<a id="ch5-06"></a>

### CH5-06. 5.3 Experimental Design: parked, with what it ties to

*Thesis_Methodology.tex l.421–691 · Structural · **Decision:** Approved · **Status:** Nothing to implement*

##### Your instruction

Address 5.3 later.

##### What Approve does

Nothing to approve or reject here. Leave this card open. It only records what 5.3 is connected to, so the decision is easier when you come back to it.

##### What I noted

- It is the largest piece: One Factor at a Time, Fixed Conditions (Table fixed_conditions), the Cases (Table case_matrix and five paragraphs), Reporting Conventions.
- The Chapter 6 opening and ten lines of Chapter 6 point at it by label, so wherever it goes the pointers hold.
- Two things in it depend on [CH5-03](#CH5-03): the case table says "seventeen" and the baseline conditions; the reference paragraph names the 5000 s coast bound.
- If it also goes to Chapter 6 it becomes the second half of *Experimental Setup*. If part goes to the appendix, the case definitions are the candidate: the table stays and the five paragraphs move.

I show no text because I propose no change now.

<a id="ch5-07"></a>

### CH5-07. 5.4 Numerical Integration and Event Detection to the appendix

*Thesis_Methodology.tex l.693–725 · Quick edit · **Decision:** Approved · **Status:** Approved, not yet implemented*

##### Your instruction

5.4 can go to the appendix.

##### What Approve does

I move the section to the appendix and add one sentence to Chapter 4, with the text shown below.

##### My opinion

I agree. The section is about the tool, not about the problem. It is three paragraphs with the Dormand–Prince pair and Brent's event location. A reader of the results needs none of it.

Where: *Simulator Capabilities* (`Thesis_Appendix_Capabilities.tex`), as its own section after *Settings* (before *Environment and Vehicle Options*, l.111), because that appendix already holds the integration-settings table (`tab:numerics`, l.95) and cites the section at l.90. The section keeps its label `sec:numerical_methods`, so the existing citations work.

Two things to repair when it moves: the sentence "the optimization algorithms and the architectures built on them are set out in Chapter sec:optimization" reads as a forward pointer in an appendix, and I drop it. Chapter 4 mentions Brent's method in the grid search and the events in Table events without saying how they are detected, so one sentence there points to the appendix.

##### The text at its new place

**Appendix section: Numerical Integration and Event Detection** (new place, after Settings; label sec:numerical_methods)

The equations of motion, Eqs. planar_eom_a to planar_eom_f, of Chapter ascent_background form a system of non-linear ordinary differential equations (ODEs) with no closed-form solution, and are integrated numerically. ~~This section sets out the integration and its event handling; the optimization algorithms and the architectures built on them are set out in Chapter sec:optimization.~~ Explicit Runge–Kutta methods are the standard choice for such non-stiff initial-value problems, advancing the state through a weighted combination of derivative evaluations within each step (burden2015numerical). The whole ascent up to insertion, coasts included, is integrated in the rotating frame of Chapter ssec:frames, and only the insertion state is converted, by Eqs. sim_ecef_eci_split to sim_ecef_eci_recombine.

The Dormand–Prince RK5(4) pair is adopted here (dormand1980family). This embedded scheme computes two solutions, of orders five and four, from the same derivative evaluations and advances with the fifth-order one; their difference provides a local truncation-error estimate that is used to control the step size adaptively, taking large steps in smooth coasting arcs and small steps where the dynamics change rapidly, as through maximum dynamic pressure (burden2015numerical). Every phase is propagated with that adaptive control and a bounded maximum step. The integrator and its event location are those of SciPy (virtanen2020scipy), as are the grid searches. The particle swarm described in Chapter ssec:pso is from PyGMO (biscani2020pagmo).

A distinctive requirement of ascent simulation is the precise detection of *events*: the instants at which the dynamics change qualitatively. Planned instants simply end the integration interval. Every other event is expressed as a scalar function S(t, **x**) whose sign change marks the transition. Once the integrator brackets that sign change, the exact crossing time is located by Brent's method, which combines the guaranteed convergence of bisection with the speed of inverse quadratic interpolation (brent1973algorithms), and the phase is terminated. Integration then restarts with the updated configuration, so every discontinuity falls between two integrations rather than within a step, where it would corrupt the error estimate. Coast arcs are integrated with the same equations at zero thrust.

The integration-settings table (Table numerics, by execution path) is already in the appendix and stays where it is, in *Settings*, which cites this section.

**Chapter 4, Trajectory Framework** (Thesis_Optimization_Background.tex l.344), the paragraph before Table events:

The events common to every ascent are listed in Table events. The second-stage arcs end at instants set by the design vector or by the guidance law's own time-to-go. <ins>Each event is located by Brent's method on a sign change of a function of the state, as set out in Appendix sec:numerical_methods.</ins>

##### What else is affected

- Thesis_Methodology.tex l.153 and l.156 cite `sec:numerical_methods` and `ssec:integration`; both follow their labels.
- The bibliography entries burden2015numerical, dormand1980family, brent1973algorithms, virtanen2020scipy and biscani2020pagmo stay cited.

<a id="ch5-08"></a>

### CH5-08. The commented Software Implementation text

*Thesis_Methodology.tex l.175–393 · Quick edit · **Decision:** Not answered · **Status:** To implement once approved*

##### Your instruction

You did not mention the commented-out text in Chapter 5 (it is not printed). **This card is my proposal, not yours.**

##### What Approve does

I delete the three commented blocks from `Thesis_Methodology.tex` and save a copy in `dev-notes/`. Nothing printed changes.

##### My proposal and why

Remove the three commented blocks (l.175–294, 296–393, 704–723) from the thesis file and keep a copy in `dev-notes/`. Together they are about 200 lines of software description and a copy of the integration-settings table, which is printed in the appendix (Thesis_Appendix_Capabilities l.95). They are not printed, they will not be printed after the move, and they make the file harder to read when you edit in Overleaf. The code description lives in `Tese/worktree.md` and the appendix.

##### Text at its new place

None. Nothing prints differently.

##### What else is affected

- `tab:numerics` is defined twice, once in the comment block and once in the appendix. Only the appendix one is printed, so no reference breaks either way.
