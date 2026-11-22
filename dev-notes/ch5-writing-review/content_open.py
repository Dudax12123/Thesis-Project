"""Open items of the Chapter 5 review: the eight cards still without an answer plus new drafts.
Cards come from content.py (executed up to the rename block, tolerant of already-applied edits)."""
import os, pathlib
os.environ["TOLERANT"] = "1"
HERE = pathlib.Path(__file__).parent
_src = (HERE / "content.py").read_text(encoding="utf8")
_src = _src[: _src.index("import re as _re")]
_ns = {"__name__": "content_head"}
exec(compile(_src, "content_head", "exec"), _ns)

ROOT = _ns["ROOT"]
FILES = dict(_ns["FILES"])
FILES.update({"ABS": "Thesis_Abstract.tex", "CON": "Thesis_Conclusions.tex", "RES": "Thesis_Results.tex"})
_cache = {}


def text(fk):
    if fk not in _cache:
        _cache[fk] = (ROOT / FILES[fk]).read_text(encoding="utf-8")
    return _cache[fk]


I = "IMPL"
OPEN_IDS = ["MCIT-01", "MCIT-02", "MCIT-03", "BIB-01", "HW-02", "WCIT-01", "WCIT-02", "WCIT-03"]
CARDS = []
for _id in OPEN_IDS:
    c = [x for x in _ns["CARDS"] if x["id"] == _id][0]
    c = dict(c)
    c["tab"] = "open"
    CARDS.append(c)
# the old text of these cards names the cases as the page did before the rename
_REN = [("Architecture A", "Ascent Case A"), ("Architecture D", "Ascent Case D"), ("Architecture ", "Ascent Case ")]
for c in CARDS:
    for f in ("title", "refers", "reading", "rec", "note"):
        if c.get(f):
            for a, b in _REN:
                c[f] = c[f].replace(a, b)
    c["edits"] = [(fk, old, new) for fk, old, new in c["edits"]]


def card(id, loc, size, tags, title, refers, reading, rec, edits=(), where=None, affected=("Nothing else.",), note=None):
    CARDS.append(dict(id=id, tab="open", loc=loc, size=size, tags=tags, title=title, refers=refers, reading=reading,
                      rec=rec, edits=list(edits), where=where, affected=list(affected), note=note))


# ======================================================================== NEW DRAFTS
card("TERM-05", "Ch 5 §5.3.3 (l.473, 482, 484)", "quick", [4],
     "Three names for the law of Ascent Case C: &ldquo;the optimal control&rdquo;, &ldquo;the costate-optimal control&rdquo;, &ldquo;optimal-control law&rdquo;",
     "&ldquo;&hellip; with the second-stage guidance law replaced by the optimal control of Section&hellip;&rdquo;; &ldquo;The optimal control takes over at MECO&hellip;&rdquo;; &ldquo;&hellip; under the costate-optimal control of Eq.&hellip;&rdquo;. The title of the subsection is &ldquo;Ascent Case C: Optimal Control&rdquo;. Chapters 4 and 6, the Abstract and the Conclusions write &ldquo;optimal-control law&rdquo; or &ldquo;costate-optimal steering law&rdquo;. Chapter 3&rsquo;s heading is &ldquo;Optimal Control Law&rdquo;; Morgado et al. say &ldquo;optimal thrust vectoring&rdquo;.",
     "&ldquo;Optimal control&rdquo; names the method and the problem, which Chapter 3 poses. The thing that Ascent Case C flies in the second stage is a guidance law, one of the seven the thesis compares, so it needs the same noun as the others (&ldquo;gravity-turn law&rdquo;, &ldquo;PEG law&rdquo;). Three forms in four lines make the reader ask whether they are three things (principle 4).",
     "Keep &ldquo;optimal-control law&rdquo;, which Chapter 3&rsquo;s heading already uses, and leave &ldquo;Optimal control&rdquo; for the method (the sentence in l.471 and the subsection title stay).",
     [(I, "replaced by the optimal control of Section~\\ref{ssec:indirect_ascent}", "replaced by the optimal-control law of Section~\\ref{ssec:indirect_ascent}"),
      (I, "The optimal control takes over at MECO", "The optimal-control law takes over at MECO"),
      (I, "under the costate-optimal control of Eq.~\\eqref{eq:indirect_control}", "under the optimal-control law of Eq.~\\eqref{eq:indirect_control}")],
     "Thesis_Implementation.tex, l.473, 482, 484", ["The Abstract says &ldquo;costate-optimal steering&rdquo; (l.19); it is a description and can stay."])

card("NUM-01", "Abstract l.21&ndash;23 and Conclusions l.26&ndash;28", "quick", [4, 9],
     "The Abstract and the Conclusions still count the ascent cases the old way",
     "Abstract: &ldquo;under four ascent cases and a segmented multi-law schedule&rdquo;. Conclusions: &ldquo;under five ascent cases, from a one-dimensional grid search (A) to the optimal-control solution (C), one of them a segmented multi-law schedule (E)&rdquo;.",
     "Chapter 5 has four ascent cases: A the segmented schedule, B the swarm-optimized coast, C the optimal-control solution, D direct insertion. The Abstract counts the segmented schedule twice (four and one more), and the Conclusions use the old lettering in which A was the grid search and E the segmented schedule. The grid search of the pitch-over angle is not an ascent case of Chapter 5 (Appendix A.5 says so) but it is flown in Chapter 6.",
     "Four ascent cases with the letters of Chapter 5, and the grid search named separately.",
     [("ABS", "under four ascent cases and a segmented multi-law\nschedule.", "under four ascent cases, one of them a segmented multi-law\nschedule."),
      ("CON", "under five ascent cases, from a\none-dimensional grid search (A) to the optimal-control solution (C), one of them a segmented\nmulti-law schedule (E), on one vehicle",
       "under four ascent cases, a segmented\nmulti-law schedule (A), a swarm-optimized coast (B), the optimal-control solution (C) and direct\ninsertion (D), and a one-dimensional grid search, on one vehicle")],
     "Thesis_Abstract.tex l.21&ndash;22; Thesis_Conclusions.tex l.26&ndash;28", ["Extended Abstract (stand-alone folder) still lists five architectures A&ndash;E; not touched."],
     note="The Abstract wording follows the Conclusions; if you prefer the grid search in the Abstract too, say so.")

card("NUM-02", "Ch 6 l.345 and l.377", "quick", [4, 6],
     "Chapter 6 gives the grid search the letter A and points to an appendix as a chapter",
     "&ldquo;The gravity turn is taken through the apogee-circularization ascent case (A, Chapter~\\ref{ssec:grid_search})&rdquo; and &ldquo;the swarm optimized coast (B) of Section~\\ref{ssec:pso_coast} and the apogee-matching grid search of Section~\\ref{ssec:grid_search}&rdquo;.",
     "In Chapter 5, A is the segmented schedule and B the swarm-optimized coast. The grid search has no letter there, and its label <code>ssec:grid_search</code> is in Appendix A.5, so &ldquo;Chapter&rdquo; and &ldquo;Section&rdquo; are both wrong references (check 11). The pso_settings table of Chapter 6 already uses the new letters.",
     "Drop the letter A, point to the appendix by its own name, and give B its full name once.",
     [("RES", "(A, Chapter~\\ref{ssec:grid_search})", "(Appendix~\\ref{ssec:grid_search})"),
      ("RES", "swarm optimized coast (B) of Section~\\ref{ssec:pso_coast} and the\napogee-matching grid search of Section~\\ref{ssec:grid_search}.",
       "swarm optimized coast (Ascent Case B, Section~\\ref{ssec:pso_coast}) and the\napogee-matching grid search of Appendix~\\ref{ssec:grid_search}.")],
     "Thesis_Results.tex l.345, 377", ["The Ch 6 table calls the grid search an &ldquo;apogee-circularization ascent case&rdquo;, while Appendix A.5 says it is not one of Chapter 5&rsquo;s. Tell me if Chapter 6 should call it a reference search instead."])
DEPS = []


# ======================================================================== ORI-02 ADDITIONS (already written, 753c55e)
_W = "<b>Already in the thesis (commit 753c55e).</b> Approve keeps it, Change tells me what to alter, Reject has me revert it. "
_TABLES = "Chapter 5&rsquo;s Tables <code>cap_rocket</code> and <code>cap_guidance</code> list options that no chapter described: only Appendix A.4 and nothing for the coast end and the burn end."
card("ORI-02a", "Ch 2 §2.1 Thrust (Thesis_Ascent_Background.tex, l.141)", "quick", [1, 3],
     "Simpler engine models, described in the theory chapter",
     _W + "Added after the paragraph on the pressure model, replacing &ldquo;The simulator also offers a simpler model, with thrust and specific impulse held at their sea-level values.&rdquo;: <ins>The altitude dependence of the first stage can also be modeled in less detail. The thrust and the specific impulse can be held at their sea-level values, at their vacuum values or at the mean of the two, or they can rise from the sea-level to the vacuum values with flight time, in steps of a set interval. These models do not need the ambient pressure, but they do not follow the altitude at which the thrust changes. The thrust and the specific impulse must follow the same model, since a mixed pair implies a mass flow that the engine does not have.</ins>",
     _TABLES + " The old sentence named the simulator in a theory chapter, which your chapter-role rule forbids; the new text is a modeling choice with no mention of the simulator. Its content comes from Appendix A.4.",
     "Keep. The alternative is to move it to Chapter 5 §5.1, which makes Chapter 2 shorter but leaves the theory chapter without the models that Chapter 5 names.",
     [], "Thesis_Ascent_Background.tex, l.141", ["Chapter 5 Table cap_rocket now has a place to point to (card ORI-02d)."],
     note="No tracked change: the text is already in the source.")

card("ORI-02b", "Ch 2 §2.x circularization (Thesis_Ascent_Background.tex, l.468)", "quick", [1, 3],
     "How the coast ends, described in the theory chapter",
     _W + "Added after the sentence on the circularizing impulse at the apoapsis: <ins>The coast that precedes it can be ended in two ways: after a chosen duration, or when the vehicle reaches the target altitude, at the latest at its apoapsis, where the circularizing impulse is applied.</ins>",
     "Chapter 5 Table cap_guidance lists &ldquo;coast ends after a set duration or on reaching the target altitude&rdquo;, which was described nowhere. The content is the coast-end rule of the reference-tracking flight. The sentence is also in the same paragraph as the apoapsis impulse, so the reader meets the options where the coast is introduced.",
     "Keep.",
     [], "Thesis_Ascent_Background.tex, l.468", [],
     note="No tracked change: the text is already in the source.")

card("ORI-02c", "Ch 4 §4.2 (Thesis_Guidance.tex, l.81)", "quick", [1, 3],
     "A closed-loop law can decide when its burn ends",
     _W + "Added at the end of the paragraph on closed-loop guidance: <ins>A closed-loop law can also decide when its burn ends: its time-to-go reaches zero at the cut-off it plans, so the burn duration is a result of the law and not a choice. In the other case the duration is chosen from outside, for instance by an optimizer.</ins>",
     "The time-to-go t<sub>go</sub> = t<sub>f</sub> &minus; t was used in Chapter 4 but never said to end the burn. Chapter 5 Table cap_guidance offers &ldquo;burn ends at an optimized time or on the law&rsquo;s own time-to-go&rdquo;, and the two ascent cases that let a closed-loop law end its burn (the segmented one and the reference-tracking flight) need the reader to know this.",
     "Keep.",
     [], "Thesis_Guidance.tex, l.81", ["Chapter 5 now points to Chapter 4 for the burn end (ORI-02d)."],
     note="No tracked change: the text is already in the source.")

card("ORI-02d", "Ch 5 §5.1 after Figure ascent_variables (Thesis_Implementation.tex, l.38)", "quick", [1, 3],
     "A pointer from Chapter 5 to where the options are described",
     _W + "Added after &ldquo;Figure~\ref{fig:ascent_variables} places the guidance variables on the phases of the ascent.&rdquo;: <ins>The tables name options that this chapter does not describe. The engine models and the end of the coast are set out in Chapter~\ref{chapter:ascent_background} and the end of the burn in Chapter~\ref{chapter:guidance}; Appendix~\ref{chapter:appendix_guidance_options} gives the remaining options and the setting that selects each.</ins>",
     "Chapter 5 lists options in its tables without saying where each is explained. The sentence names Chapter 2 (engine models and coast end), Chapter 4 (burn end) and Appendix A (the rest, with the setting that selects each) in one place. This is the original ORI-02 proposal.",
     "Keep. If you reject ORI-02a to ORI-02c, this sentence must name only the appendix.",
     [], "Thesis_Implementation.tex, l.38", ["Depends on ORI-02a, ORI-02b and ORI-02c."],
     note="No tracked change: the text is already in the source.")

# answered and applied in f5e9120: no longer on the page
DONE = {"ORI-02a", "ORI-02b", "ORI-02c", "ORI-02d", "MCIT-01", "MCIT-02", "MCIT-03", "BIB-01", "WCIT-01", "WCIT-02", "TERM-05"}
CARDS = [c for c in CARDS if c["id"] not in DONE]
