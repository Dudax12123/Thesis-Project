# Chapter 4 review: implementation spec

Written 2026-10-02. **Applied** in thesis commit `08ba055`, pushed 2026-10-02, with one correction: the brute-force CPR ramp starts at the end of the vertical rise, not at lift-off. The label fix `1e34ad8` followed.

**User decision for this pass:**
- Implement the Chapter 4 review.
- Do not touch Ch. 1 or Ch. 2, which drops C1 and B9.
- Do not implement C8, A4 or A8.
- Commit only when asked.

**Before editing:**
- Overleaf's git bridge returned 503 ("no healthy upstream") on 2026-10-02.
- Fetch first and read any incoming diff.
- Line numbers below are as of `6327c4e`.

## Thesis edits

### Ch. 4 `Thesis_Guidance.tex`

#### Header, intro and table

- **E9.** Header comment l.3: "Chapter 3" → "Chapter 4".
- **E6.** l.36:
  - the second "program" → "programme";
  - "cutoff" → "cut-off".
- **B6, l.53–57.** Replace with: "This work compares the trajectories that different laws produce on a common vehicle and mission. Nine laws are implemented (Table~\ref{tab:guidance_laws}). Three are explicit closed-loop laws: the Apollo law and two variants of powered explicit guidance. The gravity turn is passive; the constant-pitch-rate, tangent and exponential laws are open-loop programmes; and the indirect law steers on costates supplied by an optimiser. The literature assigns two of them to the atmospheric arc and seven to the exo-atmospheric one, the division that organises the two sections that follow."
- **A5 + B2 + B5, `tab:guidance_laws`.**
  - Replace the $t_{go}$ column with a "Flown as (Ch. 6)" column:

    | Law | Flown as |
    |---|---|
    | Gravity turn | passive |
    | Constant pitch rate | open-loop (swarm) |
    | Linear tangent | open-loop (swarm) |
    | Bilinear tangent | open-loop (swarm) |
    | Apollo | closed-loop, shared $t_{go}$ |
    | Classical PEG | not flown |
    | Vector P–C | closed-loop, own $t_{go}$ |
    | Exponential pitch | open-loop (swarm) |
    | Indirect | reference |

  - Row text:
    - classical PEG: "Scalar pitch programme, $\sin\theta$ linear in time";
    - exponential: keep the basis column.
  - Caption: drop "serves as the baseline against which the active laws are measured". Explain the new column: open-loop constants are chosen by the swarm (Section `ssec:pso_coast`); Apollo uses the shared estimate of `ssec:tgo_estimation`, and the vector variant uses its own.

#### §4.2 Common structure

- **C5, l.104–107.** Replace with: "under the brute-force architecture the constant-pitch-rate law performs no kick, its ramp starting from the vertical (Section~\ref{ssec:dispatch})".
- **C4, l.111–117.** Replace with: "First, each law is a stateless function: given the state, the target and a time-to-go, it returns a steering command, and whatever memory it needs is held by the simulation core (Section~\ref{ssec:dispatch}) and handed back at its next call." Keep the fairness sentence.
- **l.119–121.** Replace with: "Second, outside the two exceptions above, every active law engages at second-stage ignition; until then the vehicle flies the kick and the unguided arc of Section~\ref{sec:atmospheric_laws}, whichever law is selected."

#### §4.2.1 Time-to-go

- **T14, l.128–133.** Replace with: "Several laws express their commands in terms of $t_{go}$: the Apollo law always, the tangent laws when no optimiser supplies their constants, and the constant-pitch-rate law when its rate is derived rather than chosen. In Chapter 6 only the Apollo law uses it." Keep the "implemented once … steering logic" point.
- **E8, headings.**
  - "Reverse Rocket Equation Time-to-Go Estimation" → "Reverse Rocket Equation".
  - "PEG Derived Time-to-Go Estimation" → "PEG-Derived Estimate".
- **A4 paragraph, l.143–169.** Leave untouched; R10 inside it is also left.
- **R9, l.175.** Drop `sostaric2005powered` from the cite.
- **O10, l.186.** Delete "It is correspondingly the more accurate … near insertion."
- **A7, l.191–204.** Replace with: "The third is not an estimate at all. Under the population-based architectures the burn and coast durations are decision variables, so the end of each burn is known in advance and $t_{go}$ may be taken as the countdown to that planned deadline. It is a separate override, covering the Apollo, tangent, constant-pitch-rate and classical explicit laws but not the vector predictor–corrector variant, whose own gravity-aware estimate it would defeat; no case of Chapter 6 uses it. The segmented mode counts down to a deadline of the same kind, taken from the reference timing, to fly a law across staging (Section~\ref{ssec:segmented})."

#### §4.3 Kick, gravity turn and CPR

- **E1.** "manoeuvrer" → "manoeuvre" at l.218 (×2), l.220, l.234, l.321 and l.329.
- **B4, l.218.** "is commonly termed the kick angle and is an important trajectory parameter" → "The tilt of the velocity vector from the local vertical at the end of this manoeuvre is an important trajectory parameter~\cite{teofilatto2025trajectories}."
- **O12, l.234–240.** θ̇₀ → θ̇.
- **O7 + B4, l.259–261.** Replace with: "where $\alpha_k$ is the kick angle, the peak angle of attack; $\alpha_k<0$ pitches the axis below the velocity vector, towards the horizontal. The profile returns α to zero at the end of the manoeuvre, the condition identified above for the gravity turn to begin from a well-defined state."
- **O8, l.262–263.** Delete "After the kick, and until second-stage ignition, … engages."
- **B4, l.267.** "equivalent to a kick angle of $\gamma_p-\pi/2$" → "a step of $\gamma_p-\pi/2$ in flight-path angle, $\gamma_p$ being the pitch-over angle".
- **E9.** FIG-6 comment: "(TO DRAW)" → "(DRAWN)".
- **B1 + O1 + E4, l.323–331.**
  - With α ≈ 0, the normal thrust and lift vanish and Eq.~\eqref{eq:planar_eom_b} reduces to **γ̇ = −(g/v − v/r) cos γ**~\cite{Ulrich}. The minus sign is the fix.
  - "In vertical flight γ̇ = 0, which is why the turn must be started by the kick; once started, with α near zero, the vehicle continues to rotate on its own."
  - "As the vehicle accelerates, $g/v-v/r$ falls towards zero, and at circular speed, $v^2/r=g$, γ̇ vanishes … ~\cite{Edberg}."
- **B5, l.333–337.** Replace with: "…exactly as derived. It is the passive case among the laws: what an active law gains is gained relative to letting the vehicle turn under gravity alone."
- **K4.** Delete the CFPAR paragraph (l.348). Append "only the first is implemented here" to l.344.
- **E6, l.346.** "maneuver" → "manoeuvre".
- **A6 + C6 + C8, l.350–375.**
  - Keep Eq. `cpr`.
  - "Under the coast-parameter architecture, the only one under which Chapter 6 flies it, the law engages at second-stage ignition with θ_init equal to the flight-path angle there, and θ̇ is appended to the design vector of Eq. `pso_coast_vector` as a fifth variable, bounded to [0.02, 1.0]°/s."
  - "Under brute force the ramp starts from the vertical at lift-off, at a fixed rate or at θ̇ = (π/2)/t_go."
  - "The clamp holds the command at the local horizontal once the ramp reaches it."
  - "The tangent and exponential laws make the same substitution of an outer search for an inner derivation (Sections …)."
  - Delete the displayed Eq. `pso_coast_vector_cpr`, which is referenced nowhere.

#### §4.4 Exo-atmospheric laws

- **E5.** Heading → "Explicit Guidance in the Exo-atmospheric Arc".

**Tangent laws:**
- **O2 + R13, l.393–401.** Replace with: "…Neglecting aerodynamic forces, appropriate for exo-atmospheric flight, and assuming constant gravitational acceleration, the propellant-optimal thrust direction has the tangent of its angle linear in time~\cite{Edberg}. In general the variational conditions give the bilinear tangent form~\cite{Ulrich}:"
- **O4, l.408–409.** Drop "estimated as described in Section …".
- **O2 + O3, l.425–432.** Replace with: "The law fixes the form of the steering, not its constants: a separate numerical optimization, subject to the initial and terminal constraints, determines them and the burn time $t_f$~\cite{Ulrich}. Two implications follow for terminal-state guidance~\cite{Ulrich}: the command is a ratio of linear functions of time, linear in the reduced form, and the steering at any instant depends on the current time and the boundary conditions, not on the history of the thrust acceleration."
- **K5, l.452–459.** Replace with: "Architectures that supply no constants fall back on a closed-loop form that re-derives them from the current state at every guidance update, with terminal horizontal flight and, for the bilinear law, tuned constants; no case of Chapter 6 flies it."

**Apollo:**
- **C3, l.466.** Replace with: "Polynomial guidance, flown on the Apollo missions (Section~\ref{section:litreview}), assumes the vehicle acceleration, expressed in the ECI frame, to be a prescribed function of time; following the Apollo implementation, its components vary linearly with time~\cite{Mooij_lunar_ascent},"
- **E2, l.481.** ν_x → v_x.
- **O6, l.484.** Drop ", and acceleration".
- **R13 + E3, l.502–504.** Replace with: "…recomputed at each guidance cycle as $t_{go}$ decreases, and they grow without bound as $t_{go}\to0$~\cite{Mooij_lunar_ascent}."
- **l.518–529.** Untouched (A8).
- **A3 + B8 + K3 + O5, l.531–565.** Rewrite as three paragraphs:
  1. Keep the k1 = 0, k2 = (v′ − v_x)/t_go equation, using **v′** and Section `ssec:penalty`. Add: "The four-coefficient form is retained in the code behind a flag that every call site leaves disabled and no configuration setting exposes."
  2. "Both polynomials command the total accelerations of $(v_x,v_y)=(v\cos\gamma,v\sin\gamma)$. Their thrust parts follow by subtracting the non-thrust rates in the local frame, $-g+v_x^2/r$ vertically and $-v_xv_y/r$ horizontally, rather than gravity alone."
     - "A fixed-thrust stage cannot in general meet both demands. As in the Apollo ascent guidance~\cite{bennett1970lunar}, the vertical channel is served first, up to $a_T=F_T/m$, and the horizontal one takes the remainder, new Eq. `apollo_priority` $a_x^{\mathrm{thrust}}=\pm\sqrt{a_T^2-(a_y^{\mathrm{thrust}})^2}$, with the sign of its own demand."
     - "The angle of attack is then $\alpha=\operatorname{atan2}(a_y^{\mathrm{thrust}},a_x^{\mathrm{thrust}})-\gamma$. The horizontal coefficients set only the direction of the horizontal thrust."
     - Then the surrendered/bought and hybrid sentences: the full polynomial vertically, the remaining thrust horizontally.
  3. "The coefficients are frozen once $t_{go}$ falls below 10 s, the remedy described above, so the law flies open-loop over the final seconds."

**PEG:**
- **C2 + K1 + B10, l.675–764.** Replace with:
  - "Powered explicit guidance, the Space Shuttle's ascent guidance (Section~\ref{section:litreview}), is derived under four assumptions~\cite{mchenry1979…}:" followed by the four bullets, kept.
  - "Under them the thrust direction follows a linear tangent law, written in vector form as~\cite{SONG2015463}", followed by Eq. `peg_uF`, kept.
  - "with $\|\vec\lambda_v\|=1$ and $\vec\lambda_v\cdot\dot{\vec\lambda}=0$."
  - "The seven unknowns are fixed by the cut-off constraints through a predictor–corrector cycle~\cite{SONG2015463}. The velocity to be gained, $\vec v_{go}$, is updated by the velocity change measured since the last cycle. From it the predictor evaluates $t_{go}$ and the thrust integrals over the remaining burn, which give $\vec\lambda_v=\vec v_{go}/L$ and $\dot{\vec\lambda}$ in closed form. The corrector compares the predicted cut-off state with the target and updates $\vec v_{go}$, repeating until $t_{go}$ settles."
  - The eq:peg_* labels from LJH to rthrust are referenced nowhere; re-check before deleting.
- **l.766–772.** Replace with: "Two variants are implemented, differing in their primary variable: the classical variant carries a scalar pitch programme, and the vector predictor–corrector variant carries $\vec v_{go}$. They are the `peg` and `peg_new` modes of Table~\ref{tab:guidance_laws}."
- **A2 + K2 + R2 + O11, classical variant.**
  - "It flies $\sin\theta=A+B\,t+c$, with $c=(\mu/r^2-v_\theta^2/r)/a_T$ the fraction of the thrust acceleration spent against gravity net of centrifugal relief, re-evaluated at every call~\cite{orbiterwiki_peg}."
  - "A and B follow each major cycle from a 2×2 system on the target radius and radial velocity, and the burn time from the angular-momentum gap."
  - "Undamped, the guide–estimate iteration enters a two-point limit cycle for early second-stage states; under-relaxing the burn-time update by 0.5 breaks it~\cite{burden2015numerical}."
  - "The variant is implemented but not flown in Chapter 6 (Section~\ref{ssec:experimental_design})."
  - Drop Eq. `peg_sur`, which is referenced nowhere.
- **A1 + R3, vector variant.**
  - "It follows the first-principles derivation of Mahajan and Condon~\cite{mahajan2025peg}, in planar form; its primary variable is $\mathbf v_{go}$, and $t_{go}$ follows by Eq.~\eqref{eq:tgo_peg}."
  - "The radial component depends on the gravity accumulated over the remaining burn. A predictor–corrector major loop obtains the gravity velocity and position integrals by quadrature along the predicted powered trajectory, flown in the law's own planar model without the rotating-frame terms. It feeds the predicted velocity miss back into $\mathbf v_{go}$, with a secant step, until the miss falls below 0.05 m/s."
  - Keep Eq. `pegnew_u` and the α and second-estimator sentences.

**Exponential law:**
- **C7, l.832–834.** Replace with: "…solved for $(a,b)$ by a Newton-type root finder: single shooting (Section~\ref{ssec:rootfinding})." This drops the Betts cite there.
- **C6 + C8, l.849–867.**
  - "…the shooting solve is bypassed and $(a,b)$ are appended to the design vector of Eq. `pso_coast_vector`, re-epoched at the start of each burn arc." Keep the bounds sentence and the root-finder fallback sentence.
  - Delete Eq. `pso_coast_vector_exp` and the "same substitution … indirect architecture" sentences.

**Indirect law:**
- **O9 + E7, l.874–879.**
  - "Equation" → "Eq.".
  - "Unlike the other eight laws it has no form computable from the current state alone: the initial costates that make it an extremal must be supplied by…"

### Ch. 3 (B4)

- **l.443 and Eq. `apogee_objective`.** The kick α → $\alpha_k$; at l.482, $\alpha_k^\star=\arg\min_i J(\alpha_{k,i})$.
- **l.571 and l.660.** "pitch-maneuver angle" → "pitch-over angle".
- **l.614.** "the pitch angle and" → "the pitch-over angle and".
- **`tab:pso_settings`.** "Pitch angle" → "Pitch-over angle".

### Ch. 6 (B7)

- **Note l.540–542.** Replace with: "With the terminal downrange free, the simplified dynamics of Chapter 4 make the linear form the optimal one (Section `ssec:tangent`). The bilinear form adds the mid-span fraction, so the distance between the two measures what that freedom buys on the full model, and the distance of both from the reference what the simplification costs."

### Bibliography

- **`mahajan2025peg`:** @inproceedings, Mahajan, Bharat and Condon, Gerald L., "Enhancements to Space Shuttle Powered Explicit Guidance for Planetary Ascent and Descent", AAS 25-844, preprint, 2025.
  - The file name says ASC25, which suggests the AAS/AIAA Astrodynamics Specialist Conference; confirm before setting `booktitle`.
- **`orbiterwiki_peg`:** @misc in the format of `SpaceX_Falcon9_2025`, at https://www.orbiterwiki.org/wiki/Powered_Explicit_Guidance.
  - Verify that the page holds the A + Bt + C form and the access date. The WebFetch attempt failed on the usage limit.
  - Or ask the user whether a wiki source is acceptable.

## Not implementable without the sources (park as T16)

- **R4:** the exponential law has no source.
- **R8:** `BettsSurvey1998` at l.43.
- **R10:** `ChandlerSmith1967` at l.160, inside the A4 paragraph.
- **R11:** check that Ulrich gives the rate-from-t_go ramp (l.351).
- **R12:** l.211, "flight-path angle" versus attitude.
- Also verify that `bennett1970lunar` describes the radial-first thrust priority. The attribution comes from the `apollo_guidance` docstring.

## After editing

- **Verify:**
  - ref/label/cite sweep (`scratchpad/sweep.py`);
  - `tex_balance.py` on Ch. 3, 4 and 6;
  - LF line endings;
  - word count before and after.
- **`thesis-flags.md`:**
  - close T11 and T14;
  - add T16 (above);
  - add K5: compile check of the new table column and `eq:apollo_priority`.
- **Ch. 4 checks done on 2026-10-02:**
  - The P12 resolution is passed by `pso_coast_solver` (l.595, l.616) and `rocket_ascent` (l.1474, l.1569).
  - Still to do: grep whether `reference_track_solver` reaches `apollo_guidance` with `a_thrust_available`.
