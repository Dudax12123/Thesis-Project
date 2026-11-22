"""Cards of the Chapter 5 writing review. Edits are (file key, old, new); every old string is
extracted from, or checked against, the current Overleaf source by build.py."""
import pathlib

ROOT = pathlib.Path(r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf")
FILES = {
    "IMPL": "Thesis_Implementation.tex",
    "GUID": "Thesis_Guidance.tex",
    "OPT": "Thesis_Optimization_Background.tex",
    "BIB": "Thesis_Bibliography_DB.bib",
}
_cache = {}


def text(fk):
    if fk not in _cache:
        _cache[fk] = (ROOT / FILES[fk]).read_text(encoding="utf-8")
    return _cache[fk]


def S(fk, start, end=None):
    """Exact substring of the source from `start` through `end` (inclusive)."""
    t = text(fk)
    if t.count(start) != 1:
        if TOLERANT:
            return "\0APPLIED"
        raise AssertionError((fk, start, t.count(start)))
    i = t.index(start)
    if end is None:
        return start
    if end not in t[i:]:
        if TOLERANT:
            return "\0APPLIED"
        raise ValueError(end)
    j = t.index(end, i) + len(end)
    return t[i:j]


import os
TOLERANT = bool(os.environ.get("TOLERANT"))


I = "IMPL"
CARDS = []
TERM01_EDITS = []
TERM01_AFFECTED = []


def card(id, tab, loc, size, tags, title, refers, reading, rec, edits=(), where=None, affected=("Nothing else.",), note=None):
    CARDS.append(dict(id=id, tab=tab, loc=loc, size=size, tags=tags, title=title, refers=refers,
                      reading=reading, rec=rec, edits=list(edits), where=where, affected=list(affected), note=note))


# ======================================================================== ORGANIZATION
card("ORG-01", "org", "Ch 5 §5.3 heading (l.309–311)", "quick", [1, 4],
     "Section 5.3 carries a dead label, <code>ssec:integration</code>",
     "<code>\\section{Ascent Cases}</code> opens with &ldquo;This section sets out the four architectures compared in this work.&rdquo; The heading carries two labels, <code>sec:trajectory_framework</code> and <code>ssec:integration</code>. The second names a subsection about numerical integration that no longer exists, and nothing refers to it.",
     "The title &ldquo;Ascent Cases&rdquo; is the name you chose for the four (TERM-01) and stays. The stale label is a leftover (check 7); a later reader would take it for a section about integration.",
     "Drop the dead label and keep <code>sec:trajectory_framework</code>, which the Appendix still uses.",
     [(I, S(I, r"\section{Ascent Cases}", r"\label{ssec:integration}"),
       r"""\section{Ascent Cases}
\label{sec:trajectory_framework}""")],
     "Thesis_Implementation.tex, l.309–311", ["Label <code>ssec:integration</code> is referenced nowhere (checked in all chapters and appendices)."])

card("ORG-02", "org", "Ch 5 §5.3 (l.309–396)", "structural", [1, 7],
     "Section 5.3 runs for ninety lines, three tables and three figures before its first subsection",
     "After the one-line introduction the section holds, with no heading: the events fixed in every ascent (Table 5.4, Figure common_phases, the lift-off guard), the variables left open (Table left_variables) and the constants of the open-loop laws (Table law_constants). Only then does <code>\\subsection{Architecture A}</code> begin.",
     "Three topics sit in an unlabelled run-in, so the reader cannot tell where the common ascent ends and the architectures' own material begins, and the roadmap of the section cannot be stated. Every other section of the chapter divides at a comparable length (check 1, principle 7).",
     "Add two subsection headings and keep the text as it is: <em>The Common Ascent</em> and <em>Variables Left Free</em>. The architectures become 5.3.3 to 5.3.6. The heading wording follows TERM-04.",
     [(I, r"All four fly the same typical ascent:", r"""\subsection{The Common Ascent}
\label{ssec:common_ascent}

All four fly the same typical ascent:"""),
      (I, r"After fixing the ascent variables common to the four architectures the variables of Table~\ref{tab:left_variables} remain.", r"""\subsection{Variables Left Free}
\label{ssec:left_free}

After fixing the ascent variables common to the four architectures, the variables of Table~\ref{tab:left_variables} remain.""")],
     "Thesis_Implementation.tex, l.316 and l.351", ["Subsection numbers of the four architectures each rise by two; no text refers to them by number (all references use labels).", "The section&rsquo;s introduction should then end with a sentence naming the three parts; ORI-01 gives the chapter-level one."])

card("ORG-03", "org", "Ch 5 §5.3.5 and the start of §5.3.1 (l.544–564)", "structural", [2, 7, 10],
     "The summary of the architectures comes after the four architectures it summarizes; it is also the chapter&rsquo;s last passage",
     "&ldquo;Summary of the Architectures&rdquo; closes the section with the waterfall figure (what each architecture fixes and leaves to its design vector) and the arc-structure figure (when each law acts). It is the fifth subsection of §5.3 and the last text of the chapter.",
     "The waterfall is the main proposition of the section: which variables each architecture fixes. Principle 2 asks for it before the details, not after them. As it stands the reader reads A, B, C and D with no map, then gets the map. It also ends the chapter on a figure description (principle 10).",
     "Move the two figures, with a short introduction, to the end of the section&rsquo;s opening, just before Architecture A, and delete the closing subsection. The waterfall caption says &ldquo;architecture&rdquo; instead of &ldquo;case&rdquo; (TERM-01). A chapter coda replaces the old ending (PROG-01).",
     [(I, S(I, r"\subsection{Summary of the Architectures}", "it shows when each choice takes effect."), ""),
      (I, r"\subsection{Architecture A: Segmented Multi-Law Guidance}", r"""The architectures differ in what they fix and in what they leave to the search. Figure~\ref{fig:arch_waterfall} gathers that in one view: the ascent has the same fixed events in every architecture, and each architecture then splits the remaining variables between its own definitions and its design vector $\mathbf{x}$. Figure~\ref{fig:arc_structures} sets the four against the arcs of the ascent: A changes law at the hand-off altitude, B and C fly one law through a coasted second stage, and D burns without a coast. The subsections that follow give each architecture in turn.

\begin{figure}[h]
\centering
\includegraphics[width=\linewidth]{Figures/arch_waterfall.png}
\caption{Division of the variables of the ascent: those defined in all architectures, those defined by each architecture, and those left to its design vector.}
\label{fig:arch_waterfall}
\end{figure}

\begin{figure}[h]
\centering
\includegraphics[width=\linewidth]{Figures/architecture_arc_structures.png}
\caption{Arc structures of the different architectures.}
\label{fig:arc_structures}
\end{figure}

\subsection{Architecture A: Segmented Multi-Law Guidance}""")],
     "Thesis_Implementation.tex, l.398 (insertion) and l.544–563 (deletion)",
     ["Figure <code>arch_waterfall.png</code> prints &ldquo;Case A/B/C/D Defines&rdquo; and must be regenerated with &ldquo;Architecture&rdquo; (<code>dev-notes/figures/fig_arch_waterfall.py</code>).",
      "Depends on REP-03: if the four per-architecture variable figures are kept, they repeat the waterfall."])

card("ORG-04", "org", "Ch 5 §5.1.4 Limitations (l.255–272)", "moderate", [7, 11],
     "The Limitations list restates the &ldquo;constant&rdquo; rows of the three tables, one bullet each",
     "Eleven bullets (&ldquo;The Earth can only be a sphere with point-mass gravity&rdquo;, &ldquo;The vehicle can only have two stages in series&rdquo;, &ldquo;The target orbit can only be circular&rdquo;, &hellip;). Each repeats a row that Tables cap_model, cap_rocket and cap_guidance already give with kind <em>Constant</em>.",
     "The subsection exists to give reasons (the first sentence says so), and only three bullets do: the optimal-control law in the second stage only, the missing waypoint, and the absent propellant reserve. The other eight add nothing the tables do not say (principle 11, and the repetition request). The waypoint argument is also repeated in architectures A and B (RED-2, RED-3).",
     "Summarize the eight in one sentence and keep the three bullets that carry a reason. &ldquo;One plane&rdquo; becomes &ldquo;vertical plane&rdquo; (TERM-07). The waypoint bullet stays as the one place where the argument is made; A and B then point to it.",
     [(I, S(I, r"The variables that the tables list as \emph{constant} are set in the code", r"\end{itemize}"), r"""The variables that the tables list as \emph{constant} are set in the code and cannot be changed, so each one is a limit on what the simulator can represent. Together they restrict it to a point mass that moves in the vertical plane and is steered by its angle of attack, a spherical Earth with point-mass gravity, an exponential atmosphere without wind or Mach-number effects, two stages in series, a circular target, a vertical rise set by time and a single coast. The loads and the heating are recorded and checked after the flight, since no path constraint can bound them. Most of these limits are kept because removing them would add more complexity to the implementation than this study needs. Three need their reason:
\begin{itemize}
  \item The optimal-control law can only be used in the second stage. Using it in the lower stages would add complexity to the implementation and depart from the literature reviewed.
  \item The first of two guidance laws can only aim at orbit insertion, since no waypoint can be set. Aiming at the orbit asks nothing of either law beyond the target. A waypoint would have to be defined by an altitude, a velocity and/or a flight-path angle, depending on what the guidance law requires, and these values would have to come from a reference trajectory computed beforehand or be added to the search as up to three variables. Although possible, this would make the simulator more complex and is left as future work.
  \item The engines can only burn at full thrust, and no propellant reserve can be kept. A reserve would only subtract the same mass from the propellant left at insertion, the quantity compared.
\end{itemize}""")],
     "Thesis_Implementation.tex, l.258–272", ["You approved the wording of the three kept bullets in your earlier cards (L-01 to L-03); I kept them word for word, except &ldquo;literature found&rdquo; (&rarr; &ldquo;reviewed&rdquo;).", "Reject this card if you prefer the full list as a checklist."])

# ======================================================================== ORIENTATION
card("ORI-01", "ori", "Ch 5 opening (l.13–20)", "moderate", [1, 7, 10],
     "The chapter opening says what the earlier chapters supply but not what this chapter does",
     "Two paragraphs: Chapters 2 to 4 &ldquo;supply the parts from which an ascent is built&rdquo;, and &ldquo;the simulator joins these parts&rdquo;. A two-line commented roadmap (l.13–14) was removed. The chapter&rsquo;s own sections are never announced.",
     "The reader learns the inputs but not the plan, nor that this is the &ldquo;how and why&rdquo; chapter. The commented roadmap is stale, but one of its ideas (the architectures decide which variables are searched) is the chapter&rsquo;s main proposition (principle 2).",
     "Keep both paragraphs and add a third that states what the chapter gives and names the three sections, with the two definitions the rest of the thesis relies on (<em>architecture</em>, <em>design vector</em>). It refers to Chapter 6, whose label prints &ldquo;??&rdquo; while Results is commented out in <code>Thesis.tex</code>.",
     [(I, S(I, "The simulator joins these parts.", "with the least propellant."), r"""The simulator joins these parts. It flies one ascent of a two-stage vehicle from the pad to the target orbit, integrating the equations of motion of Chapter~\ref{chapter:ascent_background} while the guidance laws of Chapter~\ref{chapter:guidance} command the angle of attack. An optimizer of Chapter~\ref{chapter:optimization_background} flies the ascent many times and keeps the values of the variables that reach the orbit with the least propellant.

This chapter gives each choice made in building the simulator, and its reason. Section~\ref{sec:capabilities} defines the variables of an ascent and states which of them the simulator implements. Section~\ref{sec:free_variables} reduces them to a design vector that an optimizer can search, which defines an architecture. Section~\ref{sec:trajectory_framework} sets out the four architectures compared in Chapter~\ref{chapter:results_and_discussion}.""")],
     "Thesis_Implementation.tex, l.20", ["The Chapter 6 reference prints &ldquo;??&rdquo; until Results is re-enabled in Thesis.tex.", "Chapter 1 (l.117, 119) and Chapter 3 (l.247, 271) use &ldquo;architecture&rdquo; and &ldquo;design vector&rdquo; before this chapter defines them; see ORI-03."])

card("ORI-02", "ori", "Ch 5 §5.1 opening (l.36–39)", "quick", [1, 6],
     "The tables list options that the chapter never describes, and no pointer says where they are described",
     "Table cap_guidance offers a triangular &alpha; ramp, a PEG time-to-go predictor, impulsive orbit closure and coast end at the target altitude; Table cap_rocket offers &ldquo;mean values&rdquo; and ambient-pressure engine models. None is explained in Chapter 5. The commented line 37 once pointed to the Appendix settings.",
     "A reader meets options with no definition and no place to look (principles 5 and 6). Appendix A does describe them: kick programs (A.5), time-to-go estimators (A.7.1), apogee circularization (A.9), environment and vehicle options (A.4).",
     "Restore the pointer, in one sentence, after the figure reference.",
     [(I, r"Figure~\ref{fig:ascent_variables} places the guidance variables on the phases of the ascent.", r"""Figure~\ref{fig:ascent_variables} places the guidance variables on the phases of the ascent. The tables name options that this chapter does not describe, such as the triangular kick or the time-to-go estimators. Appendix~\ref{chapter:appendix_guidance_options} describes them and names the setting that selects each.""")],
     "Thesis_Implementation.tex, l.36", ["The commented-out line 37 can then be deleted (MECH-04)."])

card("ORI-03", "ori", "Ch 5 §5.2 (l.280–295)", "moderate", [1, 2, 5, 9],
     "&ldquo;Architecture&rdquo; and &ldquo;design vector&rdquo; are defined in passing, after Figure 5.2 has already used both",
     "Figure free_variables_cascade (&ldquo;the variables that are optimized&rdquo;) is introduced first. Then &ldquo;A set of such choices is called an architecture in this work&rdquo; closes a paragraph, and &ldquo;What remains free &hellip; is the design vector x&rdquo; is a separate one-sentence paragraph two paragraphs later. Neither term is emphasized. The same paragraph states that the full search &ldquo;would require computational resources well beyond those available&rdquo;.",
     "Chapters 1 and 4 point here for both terms, so the definitions must be explicit and early (rules, section 5). They are neither: the figure labels a box &ldquo;Design vector&rdquo; before the text defines it (principle 5), and the opening proposition is spread over three paragraphs (principle 2). The resource sentence compares with an unstated quantity.",
     "Put both definitions into the first two paragraphs, with <code>\\emph</code>, and state the resource limit as a limit of this work. Chapter 3 first uses &ldquo;the design vector&rdquo; (l.247) with no gloss; a half-sentence there points to this section (third edit).",
     [(I, S(I, r"The variables of Section~\ref{sec:capabilities} that the code does not fix are free:", "to the variables that are optimized."), r"""The variables of Section~\ref{sec:capabilities} that the code does not fix are \emph{free}: any combination of their values and options defines an ascent. Figure~\ref{fig:free_variables_cascade} shows how this space is reduced, step by step, to the \emph{design vector} $\mathbf{x}$, the variables that are optimized."""),
      (I, S(I, "Optimizing all of these variables together", "is called an architecture in this work."), r"""Searching all the free variables together is beyond the computational budget of this work, so the best possible trajectory over the whole space is not sought. The problem is reduced instead: some variables are fixed, either arbitrarily or from the literature, and the rest are left to be optimized. A set of such choices is an \emph{architecture}."""),
      ("OPT", r"A swarm of particles moves through the space of the design vector $\mathbf{x}$, each particle", r"A swarm of particles moves through the space of the design vector $\mathbf{x}$, the variables that the architecture leaves to the optimizer (Section~\ref{sec:free_variables}), each particle")],
     "Thesis_Implementation.tex, l.280 and l.282; Thesis_Optimization_Background.tex, l.247",
     ["Chapter 1 (l.117, 119) uses &ldquo;architectures&rdquo; unglossed; its sentence can stay if Chapter 5 is explicit, since the Introduction cannot define everything.", "The sentence &ldquo;What remains free &hellip; is the design vector x&rdquo; (l.293) then repeats the definition; keep it as the bridge to the search paragraph."])

# ======================================================================== TERMS
card("TERM-01", "term", "Ch 5, every place that says &ldquo;architecture&rdquo; or &ldquo;Case&rdquo; (about 30)", "moderate", [4],
     "The four are named &ldquo;Ascent Cases&rdquo; (your decision); the chapter still says &ldquo;architecture&rdquo; and a bare &ldquo;Case&rdquo;",
     "Section 5.3 is titled &ldquo;Ascent Cases&rdquo;, the subsections &ldquo;Architecture A&rdquo; to &ldquo;Architecture D&rdquo;, and the text &ldquo;Case A&rdquo;, &ldquo;each case&rdquo;, &ldquo;the four cases above&rdquo; as well as &ldquo;each architecture&rdquo;, &ldquo;the segmented architecture&rdquo;. The waterfall figure prints &ldquo;Case A Defines&rdquo;.",
     "Three names for one object (principle 4). You chose <b>Ascent Case</b>: &ldquo;Ascent Case A&rdquo; as the name and &ldquo;ascent case&rdquo; in running text. Every other card of this page already writes its proposed text with that name.",
     "Rename in Chapter 5 only. The edits below are generated from the source: one per run of occurrences, outside comments and outside the passages that other cards rewrite (those cards already use the new name).",
     TERM01_EDITS,
     "Thesis_Implementation.tex, the lines listed in the edits", TERM01_AFFECTED)

card("TERM-02", "term", "Ch 5 Table left_variables (l.363–364) and Eqs. segmented_vector, pso_coast_vector, direct_vector", "moderate", [4, 5, 7],
     "The design vector uses &Delta;t<sub>r</sub>% and coast-start%, but Chapters 3 and 4 use &Delta;t<sub>1</sub>, &Delta;t<sub>c</sub>, &Delta;t<sub>3</sub> and J = &Delta;t<sub>1</sub> + &Delta;t<sub>3</sub>",
     "Chapter 4 (Eq. optimal_ascent_problem) determines &gamma;<sub>p</sub>, &Delta;t<sub>1</sub>, &Delta;t<sub>c</sub>, &Delta;t<sub>3</sub> and minimizes J = &Delta;t<sub>1</sub> + &Delta;t<sub>3</sub>. Chapter 3 writes the same burn time as t<sub>f</sub> &minus; t<sub>cf</sub>. Chapter 5 searches [&Delta;t<sub>c</sub>, &Delta;t<sub>r</sub>%, coast-start%, &gamma;<sub>p</sub>] and never says how these fix &Delta;t<sub>1</sub> and &Delta;t<sub>3</sub>.",
     "Three notations for one pair of quantities (principle 4), and the reader must infer the link (principle 5). I read it from the code (<code>pso_coast_solver.py</code>, l.908–910): &Delta;t<sub>1</sub> + &Delta;t<sub>3</sub> = (&Delta;t<sub>r</sub>%/100)&middot;T<sub>max,2</sub> and &Delta;t<sub>1</sub> = (coast-start%/100)&middot;(&Delta;t<sub>1</sub> + &Delta;t<sub>3</sub>). So the objective&rsquo;s burn-time term J<sub>nd</sub> equals &Delta;t<sub>r</sub>%/100.",
     "State the relation in the two rows of Table left_variables, where the symbols are introduced. The names of Chapter 4 then carry through. The Chapter 3 notation t<sub>f</sub> &minus; t<sub>cf</sub> is outside this chapter; it is the same fault and I can add a card for it if you want.",
     [(I, "Second-stage burn time, as a percentage of the maximum burn time", r"Second-stage burn time $\Delta t_1+\Delta t_3$, as a percentage of the maximum burn time $T_{\max,2}$"),
      (I, "Onset of the coast, as a percentage of that burn", r"Onset of the coast: the first burn $\Delta t_1$, as a percentage of $\Delta t_1+\Delta t_3$")],
     "Thesis_Implementation.tex, l.363–364", ["$T_{\\max,2}$ is defined in Chapter 3 (Eq. nd_terms); it is not in the nomenclature yet.", "Chapter 3 l.165 and l.290 use t<sub>f</sub> &minus; t<sub>cf</sub> for the same burn time."])

card("TERM-03", "term", "Ch 5 §5.3.1 (l.427)", "quick", [4],
     "&ldquo;The kick angle&rdquo; where Chapter 4 and the rest of Chapter 5 say pitch-over angle &gamma;<sub>p</sub>",
     "&ldquo;the design vector holds the timing of the three arcs and the kick angle&rdquo;. Everywhere else: &ldquo;pitch-over angle&rdquo;, &gamma;<sub>p</sub>.",
     "A synonym for a symbol (principle 4; the rules fix &ldquo;pitch-over angle &gamma;<sub>p</sub>&rdquo;). &ldquo;Kick&rdquo; alone is fine for the maneuver, which Edberg and Walter both call it.",
     "Replace in the sentence.",
     [(I, "the timing of the three arcs and the kick angle", r"the timing of the three arcs and the pitch-over angle")],
     "Thesis_Implementation.tex, l.427", [])

card("TERM-04", "term", "Ch 5 Table left_variables caption (l.355); Figures arch_vars_A–D captions", "quick", [4],
     "&ldquo;Free&rdquo; in §5.2, &ldquo;left open&rdquo; in §5.3: two words for the same step of the reduction",
     "§5.2: &ldquo;the variables &hellip; that the code does not fix are <em>free</em>&rdquo;; Figure free_variables_cascade: &ldquo;Variables left free&rdquo;. §5.3: Table left_variables is &ldquo;Variables left open by Table events&rdquo;, and the four architecture figures are captioned &ldquo;variables defined and left open&rdquo;.",
     "Principle 4. Chapters 3 and 4 use &ldquo;free&rdquo; (&ldquo;each law leaves quantities free&rdquo;, &ldquo;free constants&rdquo;), so &ldquo;free&rdquo; is the established word.",
     "Use &ldquo;free&rdquo;. Text edits below. The titles inside <code>arch_variables_A–D.png</code> (&ldquo;Variables Left Open&rdquo;) need the same change when the figures are regenerated. This is a decision card: reject it if you prefer &ldquo;open&rdquo; and I will change §5.2 and Figure 5.2 instead.",
     [(I, r"\caption{Variables left open by Table~\ref{tab:events}.}", r"\caption{Variables left free by Table~\ref{tab:events}.}"),
      (I, r"\caption{Architecture A: variables defined and left open.}", r"\caption{Architecture A: variables defined and left free.}"),
      (I, r"\caption{Architecture B: variables defined and left open.}", r"\caption{Architecture B: variables defined and left free.}"),
      (I, r"\caption{Architecture C: variables defined and left open.}", r"\caption{Architecture C: variables defined and left free.}"),
      (I, r"\caption{Architecture D: variables defined and left open.}", r"\caption{Architecture D: variables defined and left free.}")],
     "Thesis_Implementation.tex, five captions; figure scripts", ["Four PNGs to regenerate.", "ORG-02 already titles the new subsection &ldquo;Variables Left Free&rdquo;."])

card("TERM-05", "term", "Ch 5 l.291, 368, 474, 485", "quick", [4],
     "The seventh guidance law has three names: &ldquo;optimal control law&rdquo;, &ldquo;optimal-control law&rdquo;, &ldquo;optimal control&rdquo;",
     "§5.1 tables: &ldquo;Optimal-control law&rdquo;. §5.2: &ldquo;the optimal control law its initial costates&rdquo;. Table left_variables: &ldquo;Costates for the Optimal Control&rdquo;. Architecture C: &ldquo;the optimal control of Section 3.2&rdquo;, &ldquo;costate-optimal control&rdquo;. The Abstract says &ldquo;costate-optimal steering&rdquo; and the commented Chapter 4 subsection &ldquo;Indirect Optimal-Control Steering&rdquo;.",
     "Principle 4. The thesis has no single name for it yet, which is a question for you rather than a typo (the guidance-law names are on your list from Chapters 1&ndash;4).",
     "Recommend &ldquo;optimal-control law&rdquo; (hyphenated compound adjective, as in the tables) for the law, and &ldquo;optimal control&rdquo; only for the theory. Three edits below make Chapter 5 follow that. If you prefer &ldquo;indirect law&rdquo;, say so and I change all occurrences.",
     [(I, r"and the optimal control law its initial costates.", r"and the optimal-control law its initial costates."),
      (I, r"Costates for the Optimal Control \\", r"Initial costates of the optimal-control law \\"),
      (I, r"guidance law replaced by the optimal control of Section~\ref{ssec:indirect_ascent}.", r"guidance law replaced by the optimal-control law of Section~\ref{ssec:indirect_ascent}.")],
     "Thesis_Implementation.tex, l.291, 368, 474", ["Table cap_guidance writes &ldquo;Eight laws&rdquo;; the PEG predictor is named &ldquo;PEG predictor&rdquo; there (Chapter 4: powered explicit guidance, two variants, only the first-principles one flown). See COH-02."])

card("TERM-06", "term", "Ch 5 Table cap_rocket (l.149) and Table events (l.339)", "quick", [4],
     "Thrust symbol &ldquo;F&rdquo; and &ldquo;Atmospheric exit&rdquo; against &ldquo;F<sub>T</sub>&rdquo; and &ldquo;Atmosphere exit&rdquo;",
     "Table cap_rocket gives the thrust as $F$; the rest of the thesis uses $F_T$. Table events labels the row &ldquo;Atmospheric exit&rdquo;; Table cap_model and the footnote say &ldquo;atmosphere exit&rdquo;.",
     "Principle 4 (the rules fix F<sub>T</sub>).",
     "Two single-word edits.",
     [(I, r"Thrust and specific impulse & $F,\ I_{sp}$", r"Thrust and specific impulse & $F_T,\ I_{sp}$"),
      (I, "Atmospheric exit", "Atmosphere exit")],
     "Thesis_Implementation.tex, l.149 and l.339", [])

card("TERM-07", "term", "Ch 5 §5.1 (l.64, 68, 72, 108)", "quick", [4],
     "&ldquo;Moves in one plane&rdquo; where the thesis says &ldquo;vertical plane&rdquo;",
     "&ldquo;a point mass moving in one plane&rdquo;, &ldquo;a point mass that moves in one plane and pitches&rdquo; (twice), &ldquo;Since the vehicle moves in one plane&rdquo;. Chapters 1 and 2 say &ldquo;vertical plane&rdquo;.",
     "The rules fix &ldquo;vertical plane&rdquo;; &ldquo;one plane&rdquo; leaves open which plane (and the cross-range paragraph needs the distinction). The fourth occurrence, in the Limitations list, is covered by ORG-04.",
     "Four short edits.",
     [(I, "a point mass moving in one plane,", "a point mass moving in a vertical plane,"),
      (I, "and a point mass that moves in one plane and pitches. The atmosphere", "and a point mass that moves in the vertical plane and pitches. The atmosphere"),
      (I, "Since the vehicle moves in one plane,", "Since the vehicle moves in the vertical plane,"),
      (I, "Three: a point mass that moves in one plane and pitches", "Three: a point mass that moves in the vertical plane and pitches")],
     "Thesis_Implementation.tex", [])

# ======================================================================== DISTANCE AND RECALL
card("REC-01", "rec", "Ch 5 §5.3.3 (l.496)", "quick", [6],
     "&ldquo;The swarm does not enforce them either&rdquo; points back to a sentence in Chapter 3",
     "&ldquo;The costates make the optimality conditions part of the problem. The swarm does not enforce them either, so they enter the cost as a fifth term&rdquo;.",
     "&ldquo;Either&rdquo; recalls the terminal constraints, which Chapter 3 (§3.3.3) says the gradient-free optimizers do not enforce. In this chapter no sentence has said so (principle 6).",
     "Say what &ldquo;either&rdquo; refers to.",
     [(I, "The swarm does not enforce them either, so they enter the cost as a fifth term,", "Like the terminal constraints, they are not enforced by the swarm, so they enter the cost as a fifth term,")],
     "Thesis_Implementation.tex, l.496", [])

card("REC-02", "rec", "Ch 5 §5.3.2 opening (l.439)", "moderate", [3, 6, 4],
     "Architecture B opens with &ldquo;fix it regarding the information on Chapter&hellip;&rdquo;, a pointer instead of the information",
     "&ldquo;The segmented architecture left the placement of the laws free. The architectures that follow fix it regarding the information on Chapter~\\ref{sec:guidance_classification}. &hellip; the gravity turn is the standard way of flying it. From here on the first stage is therefore a gravity turn&rdquo;.",
     "The reader must open Section 4.2 to learn what is being applied (principle 6). The label is a section, so it prints &ldquo;Chapter 4.2&rdquo;. &ldquo;The standard way&rdquo; goes beyond the sources: Edberg calls the gravity turn &ldquo;one common pitch program&rdquo;, and Walter says it only removes steering losses and that an optimized ascent is close to it (check 9). Morgado et al., whose architecture this is, start the second phase with a gravity turn initiated by the pitch-over, and are not cited here.",
     "Name what is carried over, cite the three sources I opened (Edberg, Walter and Morgado) and say &ldquo;common&rdquo;. &ldquo;The architectures that follow&rdquo; are B, C and D, which the paragraph should name.",
     [(I, S(I, "The segmented architecture left the placement of the laws free.", "the guidance law steers the second stage only."), r"""Architecture A leaves the placement of the laws free. Architectures B, C and D fix it, following the classification of Section~\ref{sec:guidance_classification}. Along the endo-atmospheric arc the steering is shaped to limit aerodynamic loading, with a small, ideally near-zero, angle of attack, and the gravity turn is a common way of flying it~\cite{Ulrich, Edberg, teofilatto2025trajectories}. In these three architectures the first stage is therefore a gravity turn started by the pitch-over, as in~\cite{Morgado2022_MultistagePrelimDesignTrajectoryOpt}, and one guidance law steers the second stage.""")],
     "Thesis_Implementation.tex, l.439", ["Teofilatto could not be checked (the PDF text is damaged); it stays where it was.", "Edberg and Ulrich are the keys <code>Edberg</code> and <code>Ulrich</code> in the bibliography."])

card("REC-03", "rec", "Ch 5 Table law_constants (l.385)", "quick", [6],
     "The bilinear row says &ldquo;as above&rdquo;",
     "<code>Bilinear tangent &amp; &theta;_i, &theta;_f, &eta; &amp; as above; &eta;&isin;[0.1, 0.9]</code>.",
     "The coordinator&rsquo;s rule: never &ldquo;above&rdquo;; name the row. In a table the referent is also ambiguous after a page break.",
     "Name the row.",
     [(I, r"as above; $\eta\in[0.1,\,0.9]$", r"as for the linear tangent; $\eta\in[0.1,\,0.9]$")],
     "Thesis_Implementation.tex, l.385", [])

# ======================================================================== PROGRESSION AND CLOSINGS
card("PROG-01", "prog", "Ch 5 end (after Architecture D, l.540)", "quick", [10],
     "The chapter ends on the grid-search sizes of Architecture D, with no coda and no bridge to Chapter 6",
     "The last sentences are &ldquo;For each value of &gamma;<sub>p</sub> on a 500-point grid &hellip; With the burn length given by the law, only the outer search remains.&rdquo; Chapters 2 to 4 also lack codas, as the Ch 1&ndash;4 review noted.",
     "The coordinator asked for a coda (principle 10): a chapter should end by saying what it was for. Here the reader should leave knowing that these architectures are what Chapter 6 flies.",
     "Add one paragraph after the last one (with ORG-03 the summary subsection is gone, so this paragraph follows Architecture D).",
     [(I, S(I, "For each value of $\\gamma_p$ on a $500$-point grid", "only the outer search remains."), r"""For each value of $\gamma_p$ on a $500$-point grid over $[1.50,\,1.57]$~\unit{\radian}, the burn length minimizing $J$ is found by a $21$-point grid over \qtyrange{50}{100}{\percent} refined by Brent's method, and the outer grid and its refinement act on that minimum. With the burn length given by the law, only the outer search remains.

The four architectures are what Chapter~\ref{chapter:results_and_discussion} compares. It flies the guidance laws of Chapter~\ref{chapter:guidance} under each of them, on the vehicle and the mission it defines, and compares the results through the propellant left at insertion.""")],
     "Thesis_Implementation.tex, l.540", ["The paragraph sits inside the Architecture D subsection once the summary is removed; a short unnumbered closing paragraph would avoid that and I can use <code>\\paragraph*</code> if you prefer."])

card("PROG-02", "prog", "Ch 5 Table law_constants (l.375–389)", "moderate", [7],
     "The table of free constants lists four of the eight laws; the others do not end with their free constants",
     "&ldquo;Constants appended to the design vector x by each open-loop law&rdquo;: constant pitch rate, linear tangent, bilinear tangent, exponential. The gravity turn, Apollo guidance, powered explicit guidance and the optimal-control law (initial costates) are absent.",
     "The rules ask Chapter 5 to state, per law, what the optimizer chooses (principle 7: laws that do not end with the same information). A reader cannot see from one place that the three closed-loop laws and the gravity turn add nothing.",
     "Extend the table to all eight laws, in the order of Chapter 4 (gravity turn, constant pitch rate, tangent laws, exponential, Apollo, PEG, optimal control), and change the caption. This also makes the &ldquo;closed-loop adds none&rdquo; statements of §5.1.3 and §5.2 checkable.",
     [(I, r"\caption{Constants appended to the design vector $\mathbf{x}$ by each open-loop law, and their bounds.}", r"\caption{Constants appended to the design vector $\mathbf{x}$ by each guidance law, and their bounds.}"),
      (I, "\\midrule\n    Constant pitch rate", "\\midrule\n    Gravity turn        & none & --- \\\\\n    Constant pitch rate"),
      (I, S(I, r"    Exponential pitch   & $a,\ b$", "\n    \\bottomrule"), r"""    Exponential pitch   & $a,\ b$ & $[0,\,1.6]$~\unit{\radian}, $[-0.05,\,0.005]$~\unit[per-mode=power]{\per\s} \\
    Apollo guidance     & none & --- \\
    Powered explicit guidance & none & --- \\
    Optimal-control law & $\boldsymbol{\lambda}_0$ & $[-1,\,1]$ each \\
    \bottomrule""")],
     "Thesis_Implementation.tex, l.375–389", ["REC-03 edits the bilinear row of the same table; the two cards do not overlap.", "Bounds were checked against <code>simulation_parameters.py</code>: CPR 0.02–1.0 &deg;/s, tangent &theta;<sub>i</sub> [−20, 70]&deg;, &theta;<sub>f</sub> [−40, 30]&deg;, &eta; 0.1–0.9, a [0, 1.6] rad, b [−0.05, 0.005] 1/s, costates [−1, 1]. All match."])

card("PROG-03", "prog", "Ch 5 §5.3.2 (l.460–465)", "quick", [7, 10],
     "Architecture B never says what searches its design vector",
     "&ldquo;The vector has four variables for a closed-loop law, and more otherwise. The objective function to be minimized is the penalized sum J&hellip;&rdquo; A says &ldquo;searched by PSO&rdquo;, C says &ldquo;each particle encodes&rdquo;, D names its grid or swarm; B only says the swarm places the coast (l.450).",
     "The four architectures should end alike: vector, size, optimizer, objective (principle 7).",
     "Add the optimizer to B.",
     [(I, "The vector has four variables for a closed-loop law, and more otherwise.", "The vector has four variables for a closed-loop law, and more otherwise. The swarm of Section~\\ref{ssec:pso} searches it.")],
     "Thesis_Implementation.tex, l.465", [])

card("PROG-04", "prog", "Ch 5 §5.3.3 (Architecture C)", "moderate", [7, 10],
     "Architecture C stops at the cost function; it does not say how the reference is finished or used",
     "C ends with &ldquo;&hellip; the residual of the stationarity conditions &hellip; in the objective function of Section 3.3.3&rdquo;. Chapter 6 reports C as five seeds, each refined, with the best kept, and Chapter 3 §3.3.5 describes the refinement.",
     "The other architectures end with their search; C ends before it. Whether Chapter 5 should say that the swarm&rsquo;s best vector is then refined (and that the reference cache feeds the segmented architecture) is your call: your rule of 30 October says that only the PMP references are refined.",
     "Question, no edit proposed. If yes, I would add two sentences: the swarm&rsquo;s best vector is refined by the local solver of Section 3.3.5, and this is the only architecture whose result is refined.",
     [], "Thesis_Implementation.tex, end of §5.3.3", ["Relates to MECH-02, where I removed &ldquo;followed by a local refinement&rdquo; from the general rule of §5.2 for the same reason."], note="Needs your decision: does Chapter 5 mention the refinement of the references?")

# ======================================================================== STRENGTH OF CLAIMS
card("CLAIM-01", "claim", "Ch 5 §5.2 (l.306)", "quick", [9, 2],
     "&ldquo;Any optimizer can be paired with any guidance law&rdquo; is contradicted three pages later, and the reason for the separation is commented out",
     "&ldquo;The separation is deliberate. Any optimizer can be paired with any guidance law.&rdquo; Next line (commented): &ldquo;and a comparison between results changes one of them at a time.&rdquo;",
     "False as stated (check 9): Architecture D searches closed-loop laws by grid, which needs one or two variables; the optimal-control law needs the swarm and a transversality term. The sentence also announces a deliberate choice without its reason, which is what this chapter is for.",
     "Give the reason, drop the universal.",
     [(I, "The separation is deliberate to ensure the modularity of this simulator.", "The separation is deliberate: a comparison between results then changes one of the two at a time, the optimizer or the guidance law.")],
     "Thesis_Implementation.tex, l.306", [])

card("CLAIM-02", "claim", "Ch 5 §5.3.3 (l.474)", "quick", [9],
     "Architecture C &ldquo;delivers the optimal ascent&rdquo;",
     "&ldquo;It is treated as an architecture of its own because of what it delivers: the optimal ascent, the one that maximizes the propellant left at insertion.&rdquo;",
     "A swarm followed by a necessary-condition solver finds an extremal; it does not prove it is the global optimum. Chapter 3 itself says the swarm does not guarantee convergence, even to a local one, and that a different swarm point can lead to a different refined solution. Chapter 6 compares seeds.",
     "State what is found.",
     [(I, S(I, "It is treated as an architecture of its own because of what it delivers:", "left at insertion."), "It is treated as an architecture of its own because of what it delivers: the reference ascent, the one that leaves the most propellant at insertion among those the search finds.")],
     "Thesis_Implementation.tex, l.474", ["Chapter 3 l.1&ndash;50 (&ldquo;optimal reference trajectory&rdquo;) and the Abstract use &ldquo;optimal&rdquo; more loosely; that is a Ch 1&ndash;4 matter."])

card("CLAIM-03", "claim", "Ch 5 §5.3.1 (l.401)", "quick", [9],
     "&ldquo;The most general of the architectures, and those that follow are obtained by fixing parts of it&rdquo;",
     "Architecture A: &ldquo;It is the most general of the architectures, and those that follow are obtained by fixing parts of it.&rdquo;",
     "B fixes the number of laws, the hand-off and the first stage; D fixes the coast duration at zero; C additionally replaces the law by the optimal-control law, which adds the costates to the vector. Costates are not a variable of A in the sense of the waterfall figure, so C is not strictly &ldquo;A with parts fixed&rdquo;.",
     "Say what is true of all three.",
     [(I, "It is the most general of the architectures, and those that follow are obtained by fixing parts of it.", "It is the most general of the architectures, and each of the three that follow fixes more of its variables.")],
     "Thesis_Implementation.tex, l.401", ["Needs your check: can Architecture A fly the optimal-control law as its second law? §5.1.4 says it is only used in the second stage, which would allow it, but the waterfall figure shows no costates for A."])

# ======================================================================== CONCISION AND MECHANICS
card("MECH-01", "mech", "Ch 5 l.327 and l.295 and l.417", "quick", [11],
     "A missing word, a plural verb and informal phrasing",
     "&ldquo;How the events of the table are and enforced in the simulation is explained in Appendix&hellip;&rdquo; (a word is missing). &ldquo;&hellip; with more, it becomes too expensive and the particle swarm is used &hellip; (the description of these methods are found in Section&hellip;)&rdquo;.",
     "Mechanics (principle 11). &ldquo;Too expensive&rdquo; is also the phrase the rules list as informal; Chapter 3 gives the real statement (cost grows exponentially with the number of variables). The same sentence says the swarm is &ldquo;followed by a local refinement to improve the results found&rdquo;. Your rule of 30 October limits refinement to the PMP references; I removed it from this general sentence (see PROG-04).",
     "Two edits.",
     [(I, S(I, "How the events of the table are and enforced in the simulation", "Appendix~\\ref{sec:numerical_methods}."), r"""Appendix~\ref{sec:numerical_methods} explains how the events of the table are detected and enforced in the simulation."""),
      (I, S(I, "The number of variables in $\\mathbf{x}$ decides how it is searched.", "from another."), r"""The number of variables in $\mathbf{x}$ decides how it is searched. With one or two, the grid search with Brent refinement is used; with more, its cost grows exponentially with the number of variables and the particle swarm is used instead (Section~\ref{sec:optimization_algorithms}). Fixing variables is therefore what makes the optimization possible, and the choice of what to fix is what distinguishes one architecture from another.""")],
     "Thesis_Implementation.tex, l.295 and l.327", ["Check the text I did not read at the end of the Appendix: it says the events are &ldquo;detected&rdquo; (title &ldquo;Numerical Integration and Event Detection&rdquo;)."])

card("MECH-02", "mech", "Ch 5 l.485", "quick", [11],
     "&ldquo;Chapter~\\ref{ssec:indirect_ascent}&rdquo; prints &ldquo;Chapter 3.2&rdquo;",
     "&ldquo;(Eqs. costate_r&ndash;costate_gamma of Chapter~\\ref{ssec:indirect_ascent})&rdquo;: a subsection label with a <code>Chapter</code> prefix.",
     "The rule of check 7: a section label needs <code>Section~</code>. B&rsquo;s opening has the same fault and is corrected in REC-02.",
     "One word.",
     [(I, r"of Chapter~\ref{ssec:indirect_ascent})", r"of Section~\ref{ssec:indirect_ascent})")],
     "Thesis_Implementation.tex, l.485", [])

card("MECH-03", "mech", "Ch 5 header and dead comments", "quick", [11],
     "Stale header, dead comment blocks, an unfinished-sentence trail",
     "The file header says &ldquo;Moved from Thesis_Optimization_Background.tex: the former Section 4.2 introduction and the former Section 4.3&rdquo;. Eleven commented passages remain (l.13&ndash;14, 37, 130, 193, 278, 307, 325&ndash;326, 417&ndash;418, 486, 527). Four of them held content that cards above restore (l.37 in ORI-02, l.307 in CLAIM-01, l.417 in HW-04, l.527 in RED-5).",
     "Stale comments mislead the next editor and the model history rule applies to them too.",
     "Replace the header now. Delete the commented blocks after you accept the cards that restore their content; I list them so none is lost. No edit to the comments is proposed in this page.",
     [(I, S(I, "%  Moved from Thesis_Optimization_Background.tex: the former Section 4.2 introduction", "now one section (Section 5.1)."), """%  Sections: 5.1 what the simulator can fly, 5.2 how an ascent is reduced to a design vector,
%  5.3 the four architectures compared.""")],
     "Thesis_Implementation.tex, l.5–6", [])

# ======================================================================== MISSING CITATIONS
card("MCIT-01", "mcit", "Ch 5 §5.1.1 (l.59–62)", "quick", [9],
     "&ldquo;An empirical model that follows season and solar activity&rdquo; and winds as dispersions carry no source",
     "&ldquo;&hellip; tabulated as in the U.S. Standard Atmosphere, or an empirical model that follows season and solar activity. Winds may be ignored, given as a profile or drawn as dispersions&rdquo;.",
     "The standard atmosphere is cited; the empirical model is not named. I opened no source for either clause and will not invent one.",
     "Either name a source for the empirical model and the wind options (send me the reference) or cut the clause. The cut is shown.",
     [(I, S(I, "tabulated as in the U.S.", "solar\nactivity."), "tabulated as in the U.S.\nStandard Atmosphere~\\cite{usstandard1976}.")],
     "Thesis_Implementation.tex, l.59", ["Needs a source from you if you keep it."])

card("MCIT-02", "mcit", "Ch 5 §5.1.2 (l.129)", "quick", [9],
     "&ldquo;The launcher data follow the manufacturer&rsquo;s specifications&rdquo; has no citation, and overstates the source",
     "&ldquo;The launcher data follow the manufacturer’s specifications, and every value can be changed&hellip;&rdquo; The values live in Chapter 6, Table stage_params, which cites a NASA data sheet, a SpaceX page and Wevolver. Chapter 6 itself says the parameters are &ldquo;representative of a Falcon-9-class launcher&rdquo;; the nozzle exit area is derived, not sourced.",
     "&ldquo;The manufacturer&rsquo;s specifications&rdquo; is wider than the sources (check 9) and the sentence has no <code>\\cite</code>. The curly apostrophe is also the only one in the file.",
     "Name the vehicle class and cite what Chapter 6 cites.",
     [(I, S(I, "The launcher data follow the manufacturer’s specifications, and every value can be changed,", "to model another launch vehicle."), r"""The launcher data are those of a Falcon-9-class vehicle~\cite{NASA_Falcon9v12_Datasheet_2018, SpaceX_Falcon9_2025, Wevolver_Falcon9v12_Block5_2019}, and every value can be changed, either to test how a launcher parameter affects the ascent or to model another launch vehicle.""")],
     "Thesis_Implementation.tex, l.129", ["All three keys exist in the bibliography."])

card("MCIT-03", "mcit", "Ch 5 Table events footnote (l.347)", "quick", [9, 6],
     "The 65 km atmosphere-exit altitude has no source or pointer",
     "&ldquo;The altitude is adopted as the atmosphere-exit criterion so that the fairing is always jettisoned at the same point&rdquo; &mdash; the value (65 km) appears only in the table.",
     "The reason for using an altitude is given; the value is not. Chapter 2 (§2.2.4) gives about 65 km as the geometric criterion and cites Kluever (which I could not open). Pointing there closes the gap without a new claim.",
     "Name the section.",
     [(I, "The altitude is adopted as the atmosphere-exit criterion so that the fairing", r"The altitude, the geometric criterion of Section~\ref{ssec:fairing} with its usual value of about \qty{65}{\km}, is adopted as the atmosphere-exit criterion so that the fairing")],
     "Thesis_Implementation.tex, l.347", ["Kluever (Ch 2) is &ldquo;Not verified&rdquo; in the previous review as well."])

# ======================================================================== WRONG CITATIONS
card("WCIT-01", "wcit", "Ch 5 §5.3.1 (l.427) &mdash; Pontani and Morgado", "quick", [9],
     "Architecture A cites Morgado et al. for a design vector and a search that are partly theirs",
     "&ldquo;The second stage is a thrust&ndash;coast&ndash;thrust sequence of three arcs, following an established multi-arc decomposition~\\cite{pontani2014particle}. The coast is a free interval, so the design vector holds the timing of the three arcs and the kick angle, and is searched by PSO~\\cite{Morgado2022&hellip;}.&rdquo;",
     "<b>Supported, with a limit.</b> I opened Morgado et al. (2022). Their particle carries the coast duration, coast start (%), last-stage duration (%), pitch-over angle and the three initial costates, and the search is a PyGMO PSO with c<sub>1</sub> = c<sub>2</sub> = 2.05, w = 0.7298, v<sub>max</sub> = 0.5. So the vector and the search are theirs. The two-law schedule of A is not: Morgado et al. fly one second-stage law (the optimal-control law). Pontani (2014) could not be opened (not verified). &ldquo;Established&rdquo; rests on that one paper.",
     "No change to the citations. A one-clause limit is enough if you want to be exact: &ldquo;as in~\\cite{Morgado&hellip;}, with a schedule of two laws instead of one&rdquo;. Left as a note.",
     [], "Thesis_Implementation.tex, l.427", ["The same Morgado values give the bounds of HW-01."], note="Verified in the source; no edit proposed.")

card("WCIT-02", "wcit", "Ch 5 §5.3.2 (l.439) &mdash; Ulrich, Teofilatto", "quick", [9],
     "&ldquo;The standard way of flying it&rdquo; goes beyond what the two books say",
     "&ldquo;&hellip; the gravity turn is the standard way of flying it~\\cite{Ulrich, teofilatto2025trajectories}.&rdquo;",
     "<b>Partly unsupported.</b> Walter (6.4.3): the gravity turn only eliminates steering losses; &ldquo;an optimized ascent is close to a gravity-turn ascent&rdquo; but needs more, and with a vertical lift-off the gravity turn &ldquo;needs to be kicked off&rdquo;. Edberg: &ldquo;One common pitch program is called a gravity turn.&rdquo; Teofilatto: not verified (damaged text). Neither says &ldquo;standard&rdquo;.",
     "Handled in REC-02 (&ldquo;a common way&rdquo;, Edberg added).",
     [], "Thesis_Implementation.tex, l.439", [], note="Corrected in REC-02.")

card("WCIT-03", "wcit", "Ch 5 §5.1 (l.56, 60) &mdash; Vallado, U.S. Standard Atmosphere", "quick", [9],
     "Two citations I could not check",
     "&ldquo;&hellip; a full harmonic expansion~\\cite{vallado2013fundamentals}&rdquo; and &ldquo;tabulated as in the U.S. Standard Atmosphere~\\cite{usstandard1976}&rdquo;.",
     "<b>Not verified.</b> Vallado&rsquo;s book is not in the references folder; the Standard Atmosphere is the obvious source for the tabulated model and is cited correctly in Chapter 2. Both are general statements of the option space and carry little weight.",
     "Leave. Tell me if you want a page for Vallado.",
     [], "Thesis_Implementation.tex, l.56 and l.60", [], note="Not verified; no edit proposed.")

# ======================================================================== BIBLIOGRAPHY
card("BIB-01", "bib", "Thesis_Bibliography_DB.bib, Morgado2022_MultistagePrelimDesignTrajectoryOpt (l.268&ndash;275)", "quick", [11],
     "The Morgado entry has an ACM URL for a Springer paper, and no volume or article number",
     "<code>url = {https://dl.acm.org/doi/10.1007/s00158-022-03285-y}</code>; no <code>volume</code>. The PDF header reads &ldquo;Structural and Multidisciplinary Optimization (2022) 65:192&rdquo;. This is the most cited source of Chapter 5 (three citations) and of Chapter 3 (eight).",
     "dl.acm.org does not host a Springer article; the DOI is correct, so the link would fail. Volume and article number are given by the PDF.",
     "Add the volume and the article number (as <code>number</code>, the BibTeX field the style prints) and drop the URL, since the DOI is there.",
     [("BIB", S("BIB", "  doi     = {10.1007/s00158-022-03285-y},\n  url     = {https://dl.acm.org/doi/10.1007/s00158-022-03285-y}"), "  volume  = {65},\n  number  = {192},\n  doi     = {10.1007/s00158-022-03285-y}")],
     "Thesis_Bibliography_DB.bib, l.268&ndash;275", ["The Walter entry (key <code>Ulrich</code>, 2019 against the 2024 PDF) is still the open BIB-03 of the Chapter 1&ndash;4 review."])

# ======================================================================== HOW AND WHY
card("HW-01", "why", "Ch 5 §5.3 (l.391&ndash;394), and Ch 6 Table pso_settings", "structural", [2, 5, 9],
     "The bounds of &gamma;<sub>p</sub>, &Delta;t<sub>c</sub>, &Delta;t<sub>r</sub>%, coast start and the costates are in Chapter 6, without a reason; the exponential-law paragraph leaves &ldquo;re-epoched&rdquo; undefined",
     "Table law_constants gives bounds for the law constants and two sentences of reason. The bounds of the other design variables ([1.50, 1.57] rad, [0, 2000] s, [0, 100] %, [&minus;1, 1], and [50, 100] % for D) appear only in Chapter 6, Table pso_settings. Chapter 5 ends the exponential paragraph with &ldquo;The exponential law is re-epoched at the start of each burn arc&rdquo;.",
     "Chapter 5 is the &ldquo;how and why&rdquo; chapter and Chapter 6 results only, so bounds belong here with their reason. The source is available: Morgado et al. (Table 9) searched coast time 500&ndash;3000 s, pitch angle 1.54&ndash;1.57 rad, last-stage duration 70&ndash;95 (100) %, coast start 0&ndash;100 % and costates &minus;1 to 1 (&ldquo;homogeneous in &lambda;&rdquo;). The thesis departs from them in three bounds, with no reason given anywhere. &ldquo;Re-epoched&rdquo; is jargon the thesis never defines; it means the time of the exponential law restarts at each burn, where the tangent laws run through the coast (Chapter 4 says so for them).",
     "Add the bounds table with the source column, define the re-epoch, and keep Chapter 6&rsquo;s table to budgets only. <b>I propose no reasons for the three departures</b> (coast to 2000 s, &gamma;<sub>p</sub> down to 1.50 rad, burn fraction from 0 %): tell me them and I add them to the paragraph.",
     [(I, S(I, "The bound on $a$ keeps the initial commanded pitch", "of each burn arc."), r"""The bound on $a$ keeps the initial commanded pitch in the physically meaningful quadrant, and the one on $b$ admits the decaying programs the exponential law is intended to produce while leaving a narrow margin for a slowly rising one. The time of the exponential law restarts at the beginning of each burn, so that the pitch returns to $a$ at the start of the final burn; the tangent laws instead run continuously through the coast (Section~\ref{ssec:tangent}).

Table~\ref{tab:bounds} gives the bounds of the other variables of the design vector, with those that~\cite{Morgado2022_MultistagePrelimDesignTrajectoryOpt} searched. The initial costates are searched on $[-1,1]$ because the costate equations are homogeneous in $\boldsymbol{\lambda}$ (Section~\ref{ssec:indirect_ascent}).

\begin{table}[htbp]
  \centering
  \caption{Bounds of the design variables other than the law constants, and those searched in~\cite{Morgado2022_MultistagePrelimDesignTrajectoryOpt}.}
  \label{tab:bounds}
  \begin{tabular}{l l l}
    \toprule
    \textbf{Variable} & \textbf{Bounds here} & \textbf{Bounds in~\cite{Morgado2022_MultistagePrelimDesignTrajectoryOpt}} \\
    \midrule
    Pitch-over angle $\gamma_p$ & \qtyrange{1.50}{1.57}{\radian} & \qtyrange{1.54}{1.57}{\radian} \\
    Coast duration $\Delta t_c$ & \qtyrange{0}{2000}{\s} & \qtyrange{500}{3000}{\s} \\
    Burn fraction $\Delta t_r\%$ & \qtyrange{0}{100}{\percent} ($\qtyrange{50}{100}{\percent}$ in D) & \qtyrange{70}{95}{\percent} \\
    Coast start & \qtyrange{0}{100}{\percent} & \qtyrange{0}{100}{\percent} \\
    Initial costates $\boldsymbol{\lambda}_0$ & $[-1,\,1]$ & $[-1,\,1]$ \\
    \bottomrule
  \end{tabular}
\end{table}""")],
     "Thesis_Implementation.tex, l.391&ndash;394; Thesis_Results.tex Table pso_settings (bounds rows)",
     ["Chapter 6 Table pso_settings keeps its budget rows (swarm size, generations, seeds) and drops the bound rows with a pointer to Table bounds; I have not edited Chapter 6.", "&ldquo;Last-stage duration (%) 70 to 95 (100)&rdquo; is how Morgado et al. print it; I give 70&ndash;95.", "The refinement bound of 5000 s (Chapter 6) is a result-level setting and stays there."], note="Needs reasons from you for the three departures from Morgado et al.")

card("HW-02", "why", "Ch 5 Table events (l.337&ndash;343), D (l.540), Appendix A.7", "moderate", [5, 9],
     "Values fixed without a source or a reason",
     "Vertical rise 7.5 s; stage separation 3 s and second-stage ignition 8 s after MECO; final orbit 280 km; the 500-point and 21-point grids of Architecture D, the 2 s guidance update and the 10 s coefficient freeze (Appendix A.7).",
     "A &ldquo;how and why&rdquo; chapter should say why each fixed value was chosen, or where it comes from (the rules list the fairing threshold as the example of this fault). Only the 65 km altitude has a reason in Chapter 5 (the footnote). I checked the first three against the code: 3 s and 8 s are <code>TIME_First_STAGE_SEPARATION</code> and <code>TIME_SECOND_ENGINE_IGNITION</code>, 7.5 s is <code>TIME_TO_START_KICK</code>.",
     "Question for you, no edit: for each value, a source or a reason in one clause (&ldquo;as in a Falcon 9 webcast timeline&rdquo;, &ldquo;representative of Starlink missions&rdquo;, &ldquo;resolution of 1.4&times;10<sup>&minus;4</sup> rad&rdquo;, &hellip;). The 280 km orbit is also stated in Chapter 6 §6.1.1; one of the two should point to the other.",
     [], "Thesis_Implementation.tex, Table events", ["Chapter 6 §6.1.1 (Baseline Mission) repeats the 280 km, and gives the inclination and latitude, which Chapter 5 says &ldquo;any&rdquo;."], note="Needs sources or reasons from you.")

card("HW-03", "why", "Ch 5 §5.3.1 (l.349)", "quick", [2],
     "The kick is fixed as an instantaneous jump with no reason",
     "Table events: &ldquo;Kick profile: Instantaneous jump of &gamma; to &gamma;<sub>p</sub>&rdquo;. Table cap_guidance also offers a triangular &alpha; ramp. The reason for choosing the jump appears nowhere in Chapter 5 or in Chapter 4.",
     "Appendix A.5 says the jump is at most 4.1&deg; in &gamma; over the range of &gamma;<sub>p</sub> and keeps one variable per kick (the ramp has &alpha;<sub>k</sub> with &Delta;T<sub>k</sub> held). I read the reason as: one variable, a step that is small. That is my reading, not your statement.",
     "Add the reason after the lift-off sentence, in your words if mine are wrong.",
     [(I, S(I, "At lift-off, a guard against the $1/v$ singularity", "keep the vehicle vertical during the rise."), r"""At lift-off, a guard against the $1/v$ singularity of Eq.~\eqref{eq:planar_eom_b} and a hold of $\dot\gamma$ at zero keep the vehicle vertical during the rise. The kick is an instantaneous jump of $\gamma$ to $\gamma_p$, a step of at most \ang{4.1} over the range of $\gamma_p$ (Appendix~\ref{sec:app_kick}), which leaves the first stage with a single variable to optimize.""")],
     "Thesis_Implementation.tex, l.349", ["The sentence about the guard is a numerical detail in the middle of the common-ascent text; it fits Appendix A.3 better and could move there."], note="The reason is my reading; confirm or replace.")

card("HW-04", "why", "Ch 5 §5.3.1 (l.413 and l.417&ndash;418)", "quick", [2],
     "The reason for a hand-off by altitude is commented out",
     "&ldquo;The hand-off is therefore set by an altitude and not by an event of the vehicle, so it may fall before or after MECO.&rdquo; The next lines, commented: &ldquo;Being an input, it lets a law be moved up or down the ascent, into or out of the first stage, with everything else held&hellip;&rdquo;.",
     "The choice is stated, its reason is not (principle 2: a decision before its reasons; here, with no reason at all).",
     "Restore the reason, in one sentence.",
     [(I, S(I, r"The \emph{hand-off} is therefore set by an altitude and not by an event of the vehicle, so it may fall before or after MECO (Figure~\ref{fig:segmented_waypoints}).", "(Figure~\\ref{fig:segmented_waypoints})."), r"""The \emph{hand-off} is therefore set by an altitude and not by an event of the vehicle, so it may fall before or after MECO (Figure~\ref{fig:segmented_waypoints}). Being an input, the altitude lets a law be moved up or down the ascent, into or out of the first stage, with everything else held, so the effect of changing law at that point can be read against a run in which only the altitude differs.""")],
     "Thesis_Implementation.tex, l.413", ["The commented lines 417&ndash;418 can then be deleted (MECH-03)."])

# ======================================================================== REPETITION AND COHESION
card("REP-01", "rep", "Ch 5 §5.1.3 (l.181), §5.2 (l.291), §5.3 (l.373)", "moderate", [11],
     "What a law adds to the design vector is explained three times",
     "§5.1.3: &ldquo;A closed-loop law takes its coefficients from the state at each update, an open-loop law needs its constants c, and the optimal-control law its initial costates&rdquo;. §5.2: &ldquo;A closed-loop law determines its remaining variables by itself &hellip; and adds none. An open-loop law adds its own constants, and the optimal control law its initial costates.&rdquo; §5.3: &ldquo;An open-loop law prescribes the pitch as a function of time, and the constants of that function are not given by the law: the optimizer chooses them, and they are appended to the design vector&rdquo;.",
     "Three nearly identical statements, in three sections. The first (a description of the options) and the second (the step of the reduction, Figure 5.2) have a job. The third repeats them next to the table that answers it.",
     "Shorten the third to a pointer to the table (PROG-02 makes the table cover all laws).",
     [(I, S(I, "An open-loop law prescribes the pitch as a function of time, and the constants of that function are not given by the law:", "with their bounds."), r"""Table~\ref{tab:law_constants} lists, for each guidance law, the constants that the optimizer appends to the design vector $\mathbf{x}$, with their bounds. An open-loop law prescribes the pitch as a function of time but not the constants of that function; a closed-loop law determines its own and appends none.""")],
     "Thesis_Implementation.tex, l.373", ["Order: PROG-02 first (table), then this."])

card("REP-02", "rep", "Ch 5 §5.3.2 (l.467) and §5.3.4 (l.501)", "moderate", [11],
     "The closed-loop laws&rsquo; &ldquo;made for a direct insertion&rdquo; argument is made twice, with the same words",
     "B: &ldquo;Each plans a single burn to its target from the current state, with a time-to-go taken from the propellant that remains, and none accounts for a coast; they are made for a direct insertion.&rdquo; D: &ldquo;The closed-loop laws of Chapter 4 plan a single burn from the current state to the target, with a time-to-go taken from the propellant that remains. They are made for a direct insertion, and some missions are flown that way.&rdquo; The last clause of D has no source.",
     "The first statement is the argument; the second repeats it and its last clause has no source.",
     "State it once (B, where it first matters, RED-4 rewrites it) and point to it from D. Edberg supports the last clause (&ldquo;Low Earth orbits may be reached by a direct ascent&rdquo;), so the claim gets a source in the same edit.",
     [(I, S(I, r"The closed-loop laws of Chapter~\ref{chapter:guidance} plan a single burn from the current state to the target,", "and some missions are flown that way."), r"""The closed-loop laws of Chapter~\ref{chapter:guidance} are made for a direct insertion (Section~\ref{ssec:pso_coast}), and a low Earth orbit may be reached that way~\cite{Edberg}.""")],
     "Thesis_Implementation.tex, l.501", ["Order: RED-4 supplies the full explanation in B."])

card("REP-03", "rep", "Ch 5 §5.3 (Figs. arch_vars_A&ndash;D, vector equations, Fig. arch_waterfall)", "structural", [2, 11],
     "The variables of each architecture are shown three times",
     "(1) The four per-architecture figures &ldquo;variables defined and left open&rdquo;; (2) the design-vector equation of each architecture; (3) the waterfall figure of ORG-03. A and B also carry the same equation with the same label content, and A and B state the &ldquo;four variables when closed-loop&rdquo; count in the same words.",
     "The equations are needed. The waterfall carries (1) for all four at once, so the four small figures repeat it. They also repeat the information of Table left_variables.",
     "Question: keep the waterfall (moved before A, ORG-03) and drop the four per-architecture figures? The per-architecture figures take a quarter page each. No edit is proposed; if you agree I remove the four figure blocks and their references in the &ldquo;Figure X shows which variables&rdquo; sentences.",
     [], "Thesis_Implementation.tex, four figures and their lead-in sentences", ["The figure scripts stay in dev-notes."], note="Needs your decision: keep the four per-architecture figures or not.")

card("REP-04", "rep", "Ch 5 §5.3 (l.396)", "quick", [11, 4],
     "&ldquo;Any law may be flown&rdquo; is said in §5.3, again in A, again in B; the roles of the gravity turn and the constant pitch rate are not stated",
     "&ldquo;Unless an architecture states otherwise, the guidance law that steers an arc is not part of its definition: it is left for the user to choose, and any of the laws of Chapter 4 may be flown.&rdquo; Chapter 4 places the gravity turn and the constant pitch rate in the atmospheric arc, but B, C and D fly one law in the second stage, and Table law_constants lists the constant pitch rate for them.",
     "A repetition, and a gap: the reader is never told that the atmospheric laws may be flown in the second stage (and in A, as the first law).",
     "State the general rule once, with the exception stated.",
     [(I, S(I, "Unless an architecture states otherwise, the guidance law that steers an arc", "may be flown."), r"""Unless an architecture states otherwise, the guidance law that steers an arc is not part of its definition: the user chooses it. Any law of Chapter~\ref{chapter:guidance} may steer a free arc, the gravity turn and the constant pitch rate included, although Section~\ref{sec:atmospheric_laws} places them in the atmospheric arc.""")],
     "Thesis_Implementation.tex, l.396", ["Needs your check that the gravity turn and the constant pitch rate really are flown in the second stage in the results (Table law_constants implies it for the constant pitch rate)."])

card("COH-01", "rep", "Ch 4 §4.1 (l.49&ndash;51) against Ch 5 Architecture A", "moderate", [3, 9],
     "Chapter 4 says the first stage is steered only through &gamma;<sub>p</sub>; Architecture A lets a law steer it",
     "Chapter 4: &ldquo;In this work the first stage is steered only through the pitch-over angle &gamma;<sub>p</sub>, and the second stage flies a burn&ndash;coast&ndash;burn sequence steered by the angle of attack&rdquo;. Chapter 5, A: &ldquo;The first law steers from the pitch-over kick until the vehicle climbs through h<sub>ho</sub>&rdquo;, which may be before MECO. D has no coast.",
     "A contradiction between chapters (coherence). The problem statement of Chapter 4 is also the one the optimizer solves, so it should hold for all four architectures.",
     "Edit Chapter 4 so it holds for A and D.",
     [("GUID", S("GUID", "In this work the first stage is steered only", r"steered by the angle of attack $\alpha(t)$."), r"""In this work the first stage is steered through the pitch-over angle $\gamma_p$, followed by a gravity turn or, in the segmented architecture, by the first law of the schedule (Section~\ref{ssec:segmented}), and the second stage flies a burn--coast--burn sequence steered by the angle of attack $\alpha(t)$, with a coast of zero length in direct insertion.""")],
     "Thesis_Guidance.tex, l.49&ndash;51", ["A Chapter 4 edit; I am proposing it here because the fault is visible only against Chapter 5."])

card("COH-02", "rep", "Ch 5 Table cap_guidance (l.219) and §5.1.4", "quick", [6, 4],
     "&ldquo;Eight laws (Chapter 4)&rdquo; points to a chapter that presents seven",
     "Table cap_guidance: &ldquo;Guidance law &mdash; Choice &mdash; Eight laws (Chapter~\\ref{chapter:guidance})&rdquo;. Chapter 4 contains seven law subsections; its eighth, the indirect law, is a commented-out subsection (l.497) and is presented in Chapter 3 §3.2.",
     "A count that cannot be checked at the place it points to (principle 6). Chapter 6 also says &ldquo;all nine&rdquo; laws and &ldquo;seventeen cases&rdquo;, and Appendix A &ldquo;nine laws&rdquo; (the classical PEG, which is not flown); Chapters 1, 4 and 5 say eight.",
     "Point to both.",
     [(I, r"Eight laws (Chapter~\ref{chapter:guidance})", r"Eight laws (Chapter~\ref{chapter:guidance} and Section~\ref{ssec:indirect_ascent})")],
     "Thesis_Implementation.tex, l.219", ["Not edited here: Chapter 6 l.129 (&ldquo;all nine&rdquo;), l.21 (&ldquo;seventeen cases&rdquo;) and Appendix A Table app_settings (&ldquo;nine laws&rdquo;)."])

# ======================================================================== RED NOTES
card("RED-1", "red", "Ch 5 §5.3 (l.314)", "moderate", [1, 11],
     "Unfinished red paragraph: the four architectures are not the only ones the simulator can fly",
     "<span style='color:var(--pen)'>&ldquo;Despite defining 4 specific cases which are already implemented in the simulator by default, the user can make up its own ascent case by playing with the variables definitions while respecting the simulator limitations and configurations defined in Section 5.1. This case definitions are only for study and comparison purposes given the huge number of possible combinations otherwise.&rdquo;</span>",
     "A good idea in draft form: it says the architectures are a choice for comparison, not a limit (principle 1). It uses &ldquo;case&rdquo;, hard-coded &ldquo;Section 5.1&rdquo; and ungrammatical English, and it sits before the common-ascent text.",
     "Rewrite as one paragraph, with references and &ldquo;architecture&rdquo;.",
     [(I, S(I, r"\textcolor{red}{Despite defining 4 specific cases", "possible combinations otherwise.}"), r"""The four architectures are those compared in this work. They are not the only ones the simulator can fly: any set of choices that respects the limitations of Section~\ref{ssec:cap_limits} defines another architecture, and the settings of Appendix~\ref{chapter:appendix_guidance_options} select it. Four are defined here because the number of possible combinations is too large to compare them all.""")],
     "Thesis_Implementation.tex, l.314", ["The Appendix pointer is the same as ORI-02."])

card("RED-2", "red", "Ch 5 §5.3.1 (l.415)", "moderate", [9, 7],
     "Unfinished red paragraph in Architecture A: the first law and the missing waypoint",
     "<span style='color:var(--pen)'>&ldquo;Following the limitattions of section 5.1.4, the first guidance law will point to orbit insertion and not a middle waypoint. Despite being able to use closed loop laws as the first law, given the waypoint limitation it wouldnt lead to feasible results, while open loop laws constants that shape the trajectory flown are optimized by the optimizer so even if they dont have a middle waypoint their constants will be optimized taking into account the rest of the flight after the law handoff.&rdquo;</span>",
     "Typos and a hard-coded section number. The statement &ldquo;it wouldn&rsquo;t lead to feasible results&rdquo; is a result claim with no evidence in this chapter, and Chapter 6 flies a gravity turn followed by powered explicit guidance, so a closed-loop law is a valid second law. The argument for the limitation is also made in §5.1.4 and in B (RED-3).",
     "Say what follows from the limitation, without the feasibility claim. <b>Check my reading:</b> a closed-loop first law, aimed at the orbit, plans a burn that the second law abandons at the hand-off; an open-loop first law has its constants chosen together with those of the second.",
     [(I, S(I, r"\textcolor{red}{Following the limitattions of section 5.1.4", "after the law handoff.}"), r"""Following the limitation of Section~\ref{ssec:cap_limits}, the first law aims at orbit insertion and not at a waypoint. A closed-loop first law then plans a burn to the orbit that the second law abandons at the hand-off, so its steering is not shaped by the rest of the flight. An open-loop first law has no such difficulty: the optimizer sets its constants together with those of the second law, so the first arc is shaped by the flight that follows the hand-off.""")],
     "Thesis_Implementation.tex, l.415", ["Confirm that you accept dropping &ldquo;it wouldn&rsquo;t lead to feasible results&rdquo;."])

card("RED-3", "red", "Ch 5 §5.3.2 (l.450)", "quick", [11, 9],
     "Unfinished red sentence in Architecture B: why the waypoint limitation does not arise",
     "<span style='color:var(--pen)'>&ldquo;In this case there will be no problem with the waypoint aim because the guidance law chosen will allways be the final law aiming at insertion and starting from a well defined point coinciding with MECO.&rdquo;</span>",
     "Content is right and is the contrast with A; &ldquo;case&rdquo;, future tense, &ldquo;allways&rdquo;, and no sentence boundary with the one before it.",
     "Rewrite, keep the point.",
     [(I, S(I, r"\textcolor{red}{In this case there will be no problem with the waypoint aim", "coinciding with MECO}."), r"""The waypoint limitation does not arise here: the law starts from MECO, a well-defined state, and is the final law, so aiming at the insertion is what it is meant to do.""")],
     "Thesis_Implementation.tex, l.450", [])

card("RED-4", "red", "Ch 5 §5.3.2 (l.467)", "moderate", [11, 4],
     "Unfinished red paragraph in Architecture B: the closed-loop laws and the coast",
     "<span style='color:var(--pen)'>&ldquo;The closed-loop laws set some limitations to this trajectory architecture. Each plans a single burn &hellip; the steering of the first burn is made for a trajectory that doesn&rsquo;t know there is a coast.&rdquo;</span>",
     "A substantive point and the only explanation of why a closed-loop law is a poor fit for a coasted second stage. Register faults: &ldquo;doesn&rsquo;t&rdquo;, &ldquo;trajectory that doesn&rsquo;t know&rdquo; (metaphor), &ldquo;limitations&rdquo; (the word is the Limitations subsection&rsquo;s). It is also the first place the argument appears; D then repeats it (REP-02).",
     "Rewrite without metaphor, as the single statement of the argument.",
     [(I, S(I, r"\textcolor{red}{The closed-loop laws set some limitations", "that doesn't know there is a coast.}"), r"""The closed-loop laws fit this architecture only in part. Each plans a single burn to its target from the current state, with a time-to-go taken from the propellant that remains, and none accounts for a coast, so they are made for a direct insertion. Before the coast, such a law steers the first burn as a direct insertion into the final orbit; the swarm then cuts that burn short and inserts a coast that the law did not plan, and after the coast the law re-targets from the state it finds. The laws remain admissible, but the steering of the first burn is made for a trajectory without a coast.""")],
     "Thesis_Implementation.tex, l.467", ["Matches the documented behavior in the code (comment above Arc 1 of <code>run_pso_coast_trajectory</code>)."])

card("RED-5", "red", "Ch 5 §5.3.4 (l.526&ndash;527)", "quick", [2, 9],
     "Unfinished red clause in Architecture D: the weight w<sub>&gamma;</sub> = 3 is stated without its reason",
     "<span style='color:var(--pen)'>&ldquo;with the flight-path-angle weight lowered to w<sub>&gamma;</sub> = 3.&rdquo;</span> The reason is in a commented line: &ldquo;For a law that does not steer the flight-path angle, such as the gravity turn, a single burn reaches the target altitude or the target flight-path angle but not both, and the weight chooses which of the two misses is reported.&rdquo;",
     "A changed weight with no reason is an implementation choice without its why, the opposite of what this chapter is for. The reason matches the code comment of <code>PSO_DIRECT_W_FPA</code> (3.0 sits inside a plateau; the altitude is what Chapter 6 compares). The measured misses in that comment (190 km, 502 km) are results and belong in Chapter 6, not here.",
     "Restore the commented reason as two sentences, with the lower weight stated as the choice.",
     [(I, S(I, r"The objective is the penalized sum $J$ of Section~\ref{ssec:cost_function}, \textcolor{red}{with the flight-path-angle weight lowered to $w_\gamma = 3$.}"), r"""The objective is the penalized sum $J$ of Section~\ref{ssec:cost_function}, with the flight-path-angle weight lowered to $w_\gamma = 3$. For a law that does not steer the flight-path angle, such as the gravity turn, a single burn reaches the target altitude or the target flight-path angle but not both, and the weight chooses which of the two misses is reported; the lower value favors the altitude.""")],
     "Thesis_Implementation.tex, l.526", ["The commented line 527 can then be deleted (MECH-03).", "&ldquo;Favors the altitude&rdquo; follows the code comment: the altitude is what Chapter 6 compares."])


# Cards whose text depends on another card's decision; shown on both cards and on the Overview.
DEPS = [
    ("REP-01", "PROG-02", "REP-01 says the table lists every guidance law; that is true only if PROG-02 extends it. Approve both or neither."),
    ("REP-02", "RED-4", "REP-02 removes the explanation from Architecture D and points to B; RED-4 is the one that writes it in B. Approve both or neither."),
    ("ORG-02", "TERM-04", "ORG-02 titles a subsection &ldquo;Variables Left Free&rdquo;; if you choose &ldquo;open&rdquo; in TERM-04, the title changes."),
    ("ORG-03", "REP-03", "ORG-03 moves the waterfall figure to the front; REP-03 asks whether the four per-architecture figures stay. If they stay, the waterfall repeats them."),
    ("PROG-01", "ORG-03", "PROG-01 adds the coda after Architecture D; ORG-03 deletes the old closing subsection. Both touch the end of the chapter, in different text; apply them together."),
    ("ORI-01", "ORI-03", "Both state what an architecture and a design vector are, in two places. ORI-01 only names them in the roadmap; ORI-03 defines them. Approve ORI-03 if you approve ORI-01."),
    ("TERM-01", "ORG-01", "ORG-01 retitles the section &ldquo;The Four Architectures&rdquo;, which follows TERM-01&rsquo;s choice of word."),
    ("PROG-02", "TERM-05", "The new table row must carry the name TERM-05 chooses for the optimal-control law."),
    ("ORG-04", "RED-2", "The waypoint argument is kept in ORG-04&rsquo;s bullet; RED-2 and RED-3 refer to it and must not restate it."),
    ("MECH-01", "PROG-04", "MECH-01 removes &ldquo;followed by a local refinement&rdquo; from the general rule; PROG-04 asks whether to restore it for Architecture C only."),
]


# ---------------------------------------------------------------------------------------------
# Naming decision (user, 2026-11-22): the four are "Ascent Cases" (A to D), "ascent case" in running text.
# Every proposed text and every statement in my own voice is renamed here; TERM-01 gets one edit per run of
# remaining occurrences in the source, outside comments and outside the passages other cards rewrite.
import re as _re
_LB = r"(?<![\w/_:\\])"


def rn(s):
    s = _re.sub(_LB + r"Architectures (?=[A-D]\b)", "Ascent Cases ", s)
    s = _re.sub(_LB + r"Architecture ([A-D])\b", r"Ascent Case \1", s)
    s = _re.sub(r"(?<=, )Architectures(?= and the)", "Ascent Cases", s)
    s = _re.sub(_LB + r"Architectures\b", "Ascent cases", s)
    s = _re.sub(_LB + r"Architecture\b", "Ascent case", s)
    s = _re.sub(_LB + r"architectures\b", "ascent cases", s)
    s = _re.sub(_LB + r"architecture\b", "ascent case", s)
    s = _re.sub(_LB + r"(?<!Ascent )Case ([A-D])\b", r"Ascent Case \1", s)
    return s


_term01 = [c for c in CARDS if c["id"] == "TERM-01"][0]
for c in CARDS:
    if c["id"] == "TERM-01":
        continue
    for f in ("title", "reading", "rec", "note"):
        if c.get(f):
            c[f] = rn(c[f])
    c["affected"] = [rn(a) for a in c["affected"]]
    c["edits"] = [(fk, old, rn(new)) for fk, old, new in c["edits"]]

_T = text("IMPL")
_spans = []
for c in CARDS:
    if c["id"] == "TERM-01":
        continue
    for fk, old, new in c["edits"]:
        if fk == "IMPL":
            i = _T.index(old)
            _spans.append((i, i + len(old)))
_pat = _re.compile(r"(?<![\w/_:\\])(?:Architectures?|architectures?|Case [A-D])\b")
_occ = []
for m in _pat.finditer(_T):
    ls = _T.rfind("\n", 0, m.start()) + 1
    pre = _T[ls:m.start()]
    if pre.lstrip().startswith("%") or _re.search(r"(?<!\\)%", pre):
        continue
    if any(a <= m.start() < b for a, b in _spans):
        continue
    _occ.append((m.start(), m.end()))
_clusters = []
for o in _occ:
    if _clusters and o[0] - _clusters[-1][1] < 90:
        _clusters[-1][1] = o[1]
    else:
        _clusters.append([o[0], o[1]])
_TL = lambda p: _T.count("\n", 0, p) + 1
_lines = []
_wins = []
for a, b in _clusters:
    k = 30
    s0, e0 = max(0, a - k), min(len(_T), b + k)
    for x, y in _spans:                      # stay clear of the other cards' passages
        if x < e0 <= y or (x >= b and x < e0):
            e0 = x
        if x < s0 < y or (y <= a and y > s0):
            s0 = y
    if _wins and s0 < _wins[-1][1]:
        _wins[-1][1] = max(_wins[-1][1], e0)
    else:
        _wins.append([s0, e0, a])
_fin = []
for s0, e0, a in _wins:
    old = _T[s0:e0]
    while _T.count(old) != 1:
        s0 = max(0, s0 - 15)
        for x, y in _spans:
            if x < s0 < y:
                s0 = y
        old = _T[s0:e0]
    if _fin and s0 < _fin[-1][1]:
        _fin[-1][1] = max(_fin[-1][1], e0)
    else:
        _fin.append([s0, e0, a])
for s0, e0, a in _fin:
    old = _T[s0:e0]
    assert _T.count(old) == 1
    _term01["edits"].append(("IMPL", old, rn(old)))
    _lines.append(_TL(a))
TERM01_AFFECTED.extend([
    "Edited at source lines " + ", ".join(str(x) for x in _lines) + ".",
    "Figure <code>arch_waterfall.png</code> prints &ldquo;Case A/B/C/D Defines&rdquo; and must read &ldquo;Ascent Case&rdquo;; the four <code>arch_variables_*.png</code> titles say &ldquo;Case&rdquo; or &ldquo;Architecture&rdquo; too (scripts in <code>dev-notes/figures/</code>).",
    "<b>Outside Chapter 5 the word is still &ldquo;architecture&rdquo;</b>: Abstract 3, Introduction 4, Chapter 3 9, Chapter 4 1, Chapter 6 39, Conclusions 7, Appendix A 19 and 1 (about 83 places), and the Resumo and Extended Abstract. Chapter 6 also uses &ldquo;case&rdquo; for a flown run (&ldquo;seventeen cases&rdquo;), which now clashes with &ldquo;ascent case&rdquo;. No card renames them; see the questions on the Overview.",
])
# TERM-01 depends on TERM-04 (captions) and ORG-03 (figure captions)
DEPS[:] = [d for d in DEPS if d[0:2] != ("TERM-01", "ORG-01")]
DEPS.append(("TERM-01", "TERM-04", "The four figure captions &ldquo;Ascent Case A: variables defined and left open&rdquo; are rewritten by TERM-04 (&ldquo;free&rdquo;); TERM-01 leaves them to it, so approve TERM-04 or the captions keep &ldquo;Architecture&rdquo;."))


# post-fixes: statements about what OTHER chapters currently say keep their own word
for _c in CARDS:
    for _f in ("title", "reading", "rec", "note"):
        if _c.get(_f):
            _c[_f] = _c[_f].replace("per-ascent case", "per-case")
    _c["affected"] = [a.replace("per-ascent case", "per-case") for a in _c["affected"]]
    if _c["id"] == "ORI-01":
        _c["affected"] = [a.replace("use &ldquo;ascent case&rdquo; and &ldquo;design vector&rdquo; before", "use &ldquo;architecture&rdquo; and &ldquo;design vector&rdquo; before") for a in _c["affected"]]
    if _c["id"] == "ORI-03":
        _c["affected"] = [a.replace("uses &ldquo;ascent cases&rdquo; unglossed", "uses &ldquo;architectures&rdquo; unglossed (renamed there if you approve the question on the Overview)") for a in _c["affected"]]
