import io, re, sys
R = r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf"
def rd(n): return io.open(R + "\\" + n, encoding="utf-8", newline="").read()
def wr(n, s): io.open(R + "\\" + n, "w", encoding="utf-8", newline="").write(s)

meth = rd("Thesis_Methodology.tex")
L = meth.split("\n")
# lines 12..96 (1-indexed) are the 5.1 text before the subsections
body = "\n".join(L[11:96])
assert body.startswith("The simulator is configured") and body.rstrip().endswith("\\end{table}"), body[-80:]

def sub(s, old, new):
    assert s.count(old) == 1, (s.count(old), old[:60])
    return s.replace(old, new)

# nozzle-area paragraph: second sentence cut
body = sub(body, """The last entry is derived rather than sourced: Eq.~\\eqref{eq:nozzle_exit_area} fixes it from
the two first-stage thrust figures, at some \\qty{0.68}{\\m\\squared} for each of the nine engines.
It is listed only because the pressure-dependent engine model of
Section~\\ref{ssec:engine_model} reads it, and editing either thrust figure moves it.""",
"""The nozzle exit area is derived rather than sourced: Eq.~\\eqref{eq:nozzle_exit_area} fixes it from
the two first-stage thrust figures, at some \\qty{0.68}{\\m\\squared} for each of the nine engines.""")

# lift paragraph: the fact only
i = body.index("The lift is approximated as zero")
j = body.index("No path constraint bounds")
body = body[:i] + "The lift is approximated as zero and is not included in the simulation.\n\n" + body[j:]

# path-constraint paragraph: future-work clause cut
body = sub(body, """so a trajectory can be checked after the flight against the limits a
user sets for the vehicle. Imposing them in the search is left as future work
(Chapter~\\ref{section:future}).""", """so a trajectory can be checked after the flight against the limits a
user sets for the vehicle.""")

section = ("\\section{Experimental Setup}\n\\label{sec:vehicle}\n\n" + body.rstrip() + "\n\n"
           "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n")

# --- Results: opening paragraph and the new section
res = rd("Thesis_Results.tex")
old_open = res[res.index("Chapter~\\ref{chapter:implementation} fixed"):res.index("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n\\section{The Optimal Reference}")]
new_open = ("This chapter reports seventeen cases. Section~\\ref{sec:vehicle} fixes the vehicle and the mission they\n"
            "fly, and the cases, the conditions held across them and the conventions under which they are reported\n"
            "are set out in Chapter~\\ref{sec:experimental_design}.\n"
            "This chapter presents and discusses their results. Each section first presents its figures and tables and then, in its own\n"
            "discussion, compares the results with what the preceding chapters lead one to expect.\n\n")
bar = "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n"
anchor = bar + "\\section{The Optimal Reference}"
assert res.count(anchor) == 1
res = res.replace(old_open + anchor, new_open + bar + section + "\\section{The Optimal Reference}") if (old_open + anchor) in res else None
assert res is not None
wr("Thesis_Results.tex", res)

# --- Methodology: drop lines 12..96, and the label that moved
new_meth = "\n".join(L[:11] + L[96:])
new_meth = sub(new_meth, "\\section{Vehicle Configuration and Baseline Mission}\n\\label{sec:vehicle}\n", "\\section{Vehicle Configuration and Baseline Mission}\n")
wr("Thesis_Methodology.tex", new_meth)

# --- Chapter 2 pointer
asc = rd("Thesis_Ascent_Background.tex")
asc = sub(asc, "In this work $C_D$ is held constant and the lift is neglected, for the reasons given in Chapter~\\ref{sec:vehicle}.",
          "In this work $C_D$ is held constant, for the reasons given in Chapter~\\ref{sec:vehicle}, and the lift is neglected.")
wr("Thesis_Ascent_Background.tex", asc)
print("ok")
