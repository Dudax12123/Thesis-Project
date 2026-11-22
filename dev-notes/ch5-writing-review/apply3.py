import pathlib, re
R = pathlib.Path(r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf")


def rd(n):
    return (R / n).read_text(encoding="utf8")


def sub(t, old, new):
    assert t.count(old) == 1, old[:70]
    return t.replace(old, new, 1)


# Draft 1 and 2: Chapter 2
a = rd("Thesis_Ascent_Background.tex")
i = a.index("impulse therefore follows the thrust, $I_{sp}(h) = T(h)/(\\dot m_p\\,g_0)$")
j = a.index("\\cite{sutton2017rocket}.", i) + len("\\cite{sutton2017rocket}.")
d1 = ("\n\nThe altitude dependence of the first stage can also be modeled in less detail. The thrust and the specific impulse can be held at their sea-level values, "
      "at their vacuum values or at the mean of the two, or they can rise from the sea-level to the vacuum values with flight time, in steps of a set interval. "
      "These models do not need the ambient pressure, but they do not follow the altitude at which the thrust changes. "
      "The thrust and the specific impulse must follow the same model, since a mixed pair implies a mass flow that the engine does not have.")
a = a[:j] + d1 + a[j:]
k = a.index("Circularizing from the coast ellipse takes a single impulse at its apoapsis")
m = a.index("\n", k)
assert a[m - 1] != " " or True
d2 = (" The coast that precedes it can be ended in two ways: after a chosen duration, or when the vehicle reaches the target altitude, "
      "at the latest at its apoapsis, where the circularizing impulse is applied.")
# insert after the sentence that ends with its \cite{...}. before 'When the orbit must instead'
anchor = "\\cite{Edberg, curtis2020orbital}. When the orbit must instead be changed"
a = sub(a, anchor, "\\cite{Edberg, curtis2020orbital}." + d2 + " When the orbit must instead be changed")

# Draft 3: Chapter 4
g = rd("Thesis_Guidance.tex")
anchor = "relative to purely precomputed profiles~\\cite{SongZhaoTheil2023AutonomousTrajectory}."
d3 = (" A closed-loop law can also decide when its burn ends: its time-to-go reaches zero at the cut-off it plans, so the burn duration is a result of the law "
      "and not a choice. In the other case the duration is chosen from outside, for instance by an optimizer.")
g = sub(g, anchor, anchor + d3)

# Chapter 5: ORI-02 pointer
p = rd("Thesis_Implementation.tex")
old = "Figure~\\ref{fig:ascent_variables} places the guidance variables on the phases of the ascent."
new = (old + " The tables name options that this chapter does not describe. The engine models and the end of the coast are set out in "
       "Chapter~\\ref{chapter:ascent_background} and the end of the burn in Chapter~\\ref{chapter:guidance}; "
       "Appendix~\\ref{chapter:appendix_guidance_options} gives the remaining options and the setting that selects each.")
p = sub(p, old, new)

for n, t in (("Thesis_Ascent_Background.tex", a), ("Thesis_Guidance.tex", g), ("Thesis_Implementation.tex", p)):
    (R / n).write_text(t, encoding="utf8", newline="")
print("written")
