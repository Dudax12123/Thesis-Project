"""Applies the decisions the user gave on the Chapter 5 review page (2026-11-22).
python apply.py            -> dry run, prints what would change
python apply.py --write    -> writes the thesis files"""
import json, re, sys, pathlib
import content as C
from content import I, S, rn, text, ROOT, FILES

ANS = pathlib.Path(r"C:\Users\eduar\AppData\Local\Temp\claude\8d0fbe531235107b\44ee588a-64d7-4296-b7f7-53b730e2c7ad\scratchpad\answers\reviews")
cards = {c["id"]: c for c in C.CARDS}

APPLY = ["CLAIM-02", "CLAIM-03", "COH-01", "COH-02", "MECH-01", "MECH-02", "MECH-03", "ORG-01", "ORG-02", "ORG-04",
         "ORI-01", "ORI-03", "PROG-01", "PROG-03", "PROG-04", "REC-01", "REC-02", "RED-1", "RED-2", "RED-3", "RED-4",
         "REP-02", "REP-04", "TERM-02", "TERM-04", "TERM-06", "TERM-07"]
# held for a question: PROG-02, REP-01, REC-03, HW-01 (bounds), ORI-02 (pointer), TERM-05 (name)
# rejected: CLAIM-01 HW-03 HW-04 ORG-03 RED-5 REP-03; TERM-03 (kick angle kept)

edits = {c: list(cards[c]["edits"]) for c in APPLY}

# --- user overrides -------------------------------------------------------------------------------
# COH-01: no first-stage claim at all
fk, old, new = edits["COH-01"][0]
edits["COH-01"] = [(fk, old, r"In this work the second stage flies a burn--coast--burn sequence steered by the angle of attack $\alpha(t)$.")]

# MECH-01: keep the local refinement after the swarm
fk, old, new = edits["MECH-01"][1]
new = new.replace("its cost grows exponentially with the number of variables and the particle swarm is used instead (Section~\\ref{sec:optimization_algorithms}).",
                  "its cost grows exponentially with the number of variables and the particle swarm is used instead, followed by a local refinement of the results found (Section~\\ref{sec:optimization_algorithms}).")
assert "local refinement" in new
edits["MECH-01"][1] = (fk, old, new)

# ORI-01: the user's own text
ans = json.load(open(ANS / "ORI-01.json", encoding="utf8"))
ans = ans.get("data", ans)
edits["ORI-01"][0] = (I, edits["ORI-01"][0][1], ans["edits"]["0"])

# PROG-01: red coda after the summary (the summary stays at the end, ORG-03 rejected)
last = S(I, "The cases also differ in when a guidance law acts.", "it shows when each choice takes effect.")
last_new = rn(last).replace("each Ascent Case chooses", "each ascent case chooses")
last_new = last_new.replace("shows what each case chooses", "shows what each ascent case chooses").replace("The cases also differ", "The ascent cases also differ")
last_new += (r"""

\textcolor{red}{The four ascent cases are what Chapter~\ref{chapter:results_and_discussion} compares. It flies the guidance laws of Chapter~\ref{chapter:guidance} under each of them, on the vehicle and the mission it defines, and compares the results through the propellant left at insertion.}""")
edits["PROG-01"] = [(I, last, last_new)]

# PROG-04: the swarm's result is refined (Section 3.3.5)
anchor = S(I, "in the objective function of Section~\\ref{ssec:cost_function}, Eq.~\\eqref{eq:indirect_augmented}.")
edits["PROG-04"] = [(I, anchor, anchor + r" The best vector found by the swarm is then refined by a local refinement algorithm, which solves the optimality conditions from that starting point (Section~\ref{ssec:pmp_polish}).")]

# TERM-01b: the summary block stays (ORG-03 rejected), so it is renamed here
blk = S(I, r"\subsection{Summary of the Architectures}", "\\label{fig:arch_waterfall}\n\\end{figure}")
blk2 = S(I, "\\begin{figure}[h]\n\\centering\n\\includegraphics[width=\\linewidth]{Figures/architecture_arc_structures.png}", "\\label{fig:arc_structures}\n\\end{figure}")
full = S(I, r"\subsection{Summary of the Architectures}", "\\label{fig:arc_structures}\n\\end{figure}")
nb = (full.replace("Summary of the Architectures", "Summary of the Ascent Cases")
      .replace("The four cases above", "The four ascent cases above")
      .replace("in every case, and each case then", "in every ascent case, and each ascent case then")
      .replace("defined in all cases, those defined by each case", "defined in all ascent cases, those defined by each ascent case")
      .replace("the different architectures", "the different ascent cases"))
assert "case" not in re.sub(r"ascent cases?|Ascent Cases?", "", nb), nb
edits["TERM-01b"] = [(I, full, nb)]

# --- TERM-01: regenerate against the applied spans only ----------------------------------------------
T = text("IMPL")
spans = []
for cid, L in edits.items():
    for f, o, n in L:
        if f == "IMPL":
            i = T.index(o)
            spans.append((i, i + len(o)))
pat = re.compile(r"(?<![\w/_:\\])(?:Architectures?|architectures?|Case [A-D])\b")
occ = []
for m in pat.finditer(T):
    ls = T.rfind("\n", 0, m.start()) + 1
    pre = T[ls:m.start()]
    if pre.lstrip().startswith("%") or re.search(r"(?<!\\)%", pre):
        continue
    if any(a <= m.start() < b for a, b in spans):
        continue
    occ.append((m.start(), m.end()))
cl = []
for o in occ:
    if cl and o[0] - cl[-1][1] < 90:
        cl[-1][1] = o[1]
    else:
        cl.append([o[0], o[1]])
wins = []
for a, b in cl:
    s0, e0 = max(0, a - 30), min(len(T), b + 30)
    for x, y in spans:
        if x < e0 <= y or (x >= b and x < e0):
            e0 = x
        if x < s0 < y or (y <= a and y > s0):
            s0 = y
    if wins and s0 < wins[-1][1]:
        wins[-1][1] = max(wins[-1][1], e0)
    else:
        wins.append([s0, e0, a])
fin = []
for s0, e0, a in wins:
    old = T[s0:e0]
    while T.count(old) != 1:
        s0 = max(0, s0 - 15)
        for x, y in spans:
            if x < s0 < y:
                s0 = y
        old = T[s0:e0]
    if fin and s0 < fin[-1][1]:
        fin[-1][1] = max(fin[-1][1], e0)
    else:
        fin.append([s0, e0, a])
edits["TERM-01"] = [(I, T[s0:e0], rn(T[s0:e0])) for s0, e0, a in fin]

# --- verify, then apply ----------------------------------------------------------------------------
cur = {k: text(k) for k in FILES}
allspans = {}
bad = 0
for cid, L in edits.items():
    for fk, o, n in L:
        if cur[fk].count(o) != 1:
            print("NOT UNIQUE", cid, fk, repr(o[:70])); bad += 1; continue
        i = cur[fk].index(o)
        allspans.setdefault(fk, []).append((i, i + len(o), cid))
for fk, L in allspans.items():
    L.sort()
    for a, b in zip(L, L[1:]):
        if b[0] < a[1]:
            print("OVERLAP", fk, a, b); bad += 1
if bad:
    sys.exit("problems: %d" % bad)
n_edits = 0
for cid, L in edits.items():
    for fk, o, n in L:
        assert cur[fk].count(o) == 1, (cid, o[:60])
        cur[fk] = cur[fk].replace(o, n, 1)
        n_edits += 1
print("cards", len(edits), "edits", n_edits)
for fk in ("IMPL", "GUID"):
    for i, l in enumerate(cur[fk].split("\n"), 1):
        if l.lstrip().startswith("%"):
            continue
        for m in re.finditer(r"(?<![\w/_:\\])(?:[Aa]rchitectures?|Cases? [A-D])\b", l):
            print("LEFT", fk, i, l[max(0, m.start() - 40):m.end() + 30])
if "--write" in sys.argv:
    for fk in FILES:
        if cur[fk] != text(fk):
            (ROOT / FILES[fk]).write_text(cur[fk], encoding="utf8", newline="")
            print("wrote", FILES[fk])
