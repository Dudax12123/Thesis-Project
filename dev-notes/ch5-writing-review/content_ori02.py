"""ORI-02: four proposed additions so that Chapter 5's option tables have a place to point to. Nothing is in the thesis."""
import pathlib
ROOT = pathlib.Path(r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf")
FILES = {"ASC": "Thesis_Ascent_Background.tex", "GUID": "Thesis_Guidance.tex", "IMPL": "Thesis_Implementation.tex", "BIB": "Thesis_Bibliography_DB.bib"}
_c = {}


def text(fk):
    if fk not in _c:
        _c[fk] = (ROOT / FILES[fk]).read_text(encoding="utf-8")
    return _c[fk]


CARDS = []
DEPS = [("ORI-02d", "ORI-02a", "The pointer names Chapter 2 for the engine models; reject ORI-02a and it must not."),
        ("ORI-02d", "ORI-02b", "The pointer names Chapter 2 for the end of the coast."),
        ("ORI-02d", "ORI-02c", "The pointer names Chapter 4 for the end of the burn.")]


def card(id, loc, size, tags, title, refers, reading, rec, edits, where, affected=("Nothing else.",), note=None):
    CARDS.append(dict(id=id, tab="ori", loc=loc, size=size, tags=tags, title=title, refers=refers, reading=reading, rec=rec,
                      edits=list(edits), where=where, affected=list(affected), note=note))


_WHY = ("Chapter 5's Tables <code>cap_rocket</code> and <code>cap_guidance</code> list options that no chapter describes. "
        "Your chapter-role rule keeps implementation options out of the theory chapters, so the text below is written as a modeling choice, with no mention of the simulator.")

card("ORI-02a", "Ch 2 §2.1 Thrust (l.141)", "quick", [1, 3],
     "The simpler engine models are described only in Appendix A.4",
     "Chapter 2 ends its treatment of the first-stage thrust with &ldquo;The simulator also offers a simpler model, with thrust and specific impulse held at their sea-level values.&rdquo; Chapter 5 Table cap_rocket lists four such models (sea-level, vacuum, mean value, and a ramp with flight time).",
     _WHY + " The existing sentence names the simulator in a theory chapter and covers one of the four models. The proposal replaces it with the four models and the one condition that matters (thrust and specific impulse must follow the same model). Its content comes from Appendix A.4.",
     "Replace the sentence. The alternative is to leave Chapter 2 as it is and describe the four models in Chapter 5 §5.1; Chapter 2 is shorter then, but the reader meets the models in the implementation chapter first.",
     [("ASC", "The simulator also offers a simpler model, with thrust and specific impulse held at their sea-level values.",
       "The altitude dependence of the first stage can also be modeled in less detail. The thrust and the specific impulse can be held at their sea-level values, at their vacuum values or at the mean of the two, or they can rise from the sea-level to the vacuum values with flight time, in steps of a set interval. These models do not need the ambient pressure, but they do not follow the altitude at which the thrust changes. The thrust and the specific impulse must follow the same model, since a mixed pair implies a mass flow that the engine does not have.")],
     "Thesis_Ascent_Background.tex, l.141", ["Chapter 5 Table cap_rocket then has a place to point to (ORI-02d)."])

card("ORI-02b", "Ch 2 circularization (l.468)", "quick", [1, 3],
     "How the coast ends is described nowhere",
     "Chapter 2 says that circularizing from the coast ellipse takes one impulse at the apoapsis. Chapter 5 Table cap_guidance lists &ldquo;coast ends after a set duration or on reaching the target altitude&rdquo;.",
     _WHY + " The coast-end rule is that of the reference-tracking flight: the coast ends where the flight climbs through the target altitude, or at its apoapsis if that is lower. The sentence goes in the paragraph that already treats the apoapsis impulse.",
     "Add the sentence after the one on the circularizing impulse.",
     [("ASC", "\\cite{Edberg, curtis2020orbital}. When the orbit must instead be changed",
       "\\cite{Edberg, curtis2020orbital}. The coast that precedes it can be ended in two ways: after a chosen duration, or when the vehicle reaches the target altitude, at the latest at its apoapsis, where the circularizing impulse is applied. When the orbit must instead be changed")],
     "Thesis_Ascent_Background.tex, l.468", ["Chapter 5 Table cap_guidance then has a place to point to (ORI-02d)."])

card("ORI-02c", "Ch 4 §4.2 (l.81)", "quick", [1, 3],
     "That a closed-loop law can end its own burn is implied, never stated",
     "Chapter 4 uses t<sub>go</sub> = t<sub>f</sub> &minus; t as the time to cut-off but never says that it ends the burn. Chapter 5 Table cap_guidance lists &ldquo;burn ends at an optimized time or on the law&rsquo;s own time-to-go&rdquo;.",
     _WHY + " Two flights depend on it: the segmented ascent case and the reference-tracking flight let a closed-loop law end its burn.",
     "Add two sentences at the end of the paragraph on closed-loop guidance.",
     [("GUID", "relative to purely precomputed profiles~\\cite{SongZhaoTheil2023AutonomousTrajectory}.",
       "relative to purely precomputed profiles~\\cite{SongZhaoTheil2023AutonomousTrajectory}. A closed-loop law can also decide when its burn ends: its time-to-go reaches zero at the cut-off it plans, so the burn duration is a result of the law and not a choice. In the other case the duration is chosen from outside, for instance by an optimizer.")],
     "Thesis_Guidance.tex, l.81", ["Chapter 5 then points to Chapter 4 for the burn end (ORI-02d)."])

card("ORI-02d", "Ch 5 §5.1, after Figure ascent_variables (l.38)", "quick", [1, 3],
     "Chapter 5 lists options without saying where each is explained",
     "&ldquo;Figure~\\ref{fig:ascent_variables} places the guidance variables on the phases of the ascent.&rdquo;; the tables that follow name options that Chapter 5 does not describe.",
     "A reader who meets &ldquo;pressure engine model&rdquo; or &ldquo;burn ends on the law&rsquo;s own time-to-go&rdquo; in a table has no pointer to where it is explained (principle 3). This is the original ORI-02 proposal, now naming three places.",
     "Add the sentence. If you reject ORI-02a to ORI-02c, keep only the part that names Appendix A.",
     [("IMPL", "Figure~\\ref{fig:ascent_variables} places the guidance variables on the phases of the ascent.",
       "Figure~\\ref{fig:ascent_variables} places the guidance variables on the phases of the ascent. The tables name options that this chapter does not describe. The engine models and the end of the coast are set out in Chapter~\\ref{chapter:ascent_background} and the end of the burn in Chapter~\\ref{chapter:guidance}; Appendix~\\ref{chapter:appendix_guidance_options} gives the remaining options and the setting that selects each.")],
     "Thesis_Implementation.tex, l.38", ["Depends on ORI-02a, ORI-02b and ORI-02c."])
