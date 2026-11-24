"""Cards for Section 5.2 of Chapter 5 (free variables, ascent cases and the optimization loop).
Edits are (file key, old, new); every old string is extracted from the current Overleaf source."""
import pathlib

ROOT = pathlib.Path(r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf")
FILES = {"IMPL": "Thesis_Implementation.tex", "BIB": "Thesis_Bibliography_DB.bib"}
_cache = {}


def text(fk):
    if fk not in _cache:
        _cache[fk] = (ROOT / FILES[fk]).read_text(encoding="utf-8")
    return _cache[fk]


def S(fk, start, end=None):
    t = text(fk)
    assert t.count(start) == 1, (fk, start, t.count(start))
    i = t.index(start)
    if end is None:
        return start
    j = t.index(end, i) + len(end)
    return t[i:j]


I = "IMPL"
CARDS = []
DEPS = [
    ("FV-02", "FV-01", "the first paragraph no longer cites the cascade figure, so that figure has to go with it."),
    ("FV-04", "FV-01", "the loop paragraph becomes the place that cites the single, merged figure."),
    ("FV-05", "FV-04", "both edit the same stretch of the section; the order of paragraphs assumes FV-04."),
]


def card(id, loc, size, tags, title, refers, reading, rec, edits=(), where=None, affected=("Nothing else.",), note=None):
    CARDS.append(dict(id=id, tab="s52", loc=loc, size=size, tags=tags, title=title, refers=refers,
                      reading=reading, rec=rec, edits=list(edits), where=where, affected=list(affected), note=note))


# ---------------------------------------------------------------- FV-01 the figures
card("FV-01", "Ch 5 §5.2, Figures free_variables_cascade and optimization_loop (l.288&ndash;308)", "structural", [11, 7],
     "Two figures say one thing: merge them into one",
     "Figure 5.x (<code>free_variables_cascade.png</code>) has three boxes and a dashed &ldquo;Fixed variables&rdquo; box: variables of the ascent, then the ascent case fixes some, then the guidance law adds its own, giving the design vector <b>x</b>. Figure 5.y (<code>optimization_loop.png</code>) shows the optimizer sending <b>x</b> to two boxes, &ldquo;Free variables of the architecture&rdquo; and &ldquo;Variables added by the guidance law&rdquo;, which feed &ldquo;Ascent flown&rdquo;, whose final state gives the cost <i>J</i>(<b>x</b>) back to the optimizer.",
     "The second figure already contains the second half of the first: the two sources of <b>x</b>. What the first adds is the narrowing, and the fixed variables, which the loop figure leaves out although they enter every flight. The caption and the text of the first figure say what its boxes say, so it carries no information of its own (principle 11). Both PNGs also still read &ldquo;architecture&rdquo;, against the Ascent Case naming.",
     "One figure, read left to right: <i>Variables of the ascent</i> &rarr; <i>Ascent case fixes some</i> (dashed box &ldquo;Fixed variables&rdquo;, with an arrow straight into &ldquo;Ascent flown&rdquo;) &rarr; <i>Variables left free</i> + <i>added by the guidance law</i> = <b>x</b> &rarr; Optimizer (outer loop) &harr; inner loop (&ldquo;Ascent flown&rdquo;) &rarr; <i>J</i>(<b>x</b>). It keeps the label <code>fig:optimization_loop</code> and the new file name <code>ascent_optimization_loop.png</code>; the cascade figure and its label are removed. The caption below says what each stage is.",
     [(I, S(I, "\\begin{figure}[h]\n  \\centering\n  \\includegraphics[width=\\linewidth]{Figures/free_variables_cascade.png}", "\\end{figure}\n\n"),
      ""),
      (I, S(I, "\\includegraphics[width=\\linewidth]{Figures/optimization_loop.png}", "the cost function $J$.}"),
       r"""\includegraphics[width=\linewidth]{Figures/ascent_optimization_loop.png}
\caption{From the variables of the ascent to the cost. The ascent case fixes some variables, which enter every flight unchanged; the guidance law adds its own, and what is still free is the design vector $\mathbf{x}$. In the outer loop the optimizer proposes $\mathbf{x}$; in the inner loop one ascent is flown with it, and the cost $J$ computed from the final state is returned to the optimizer.}""")],
     where="<code>Thesis_Implementation.tex</code>, §5.2: the cascade figure is removed and the loop figure is replaced. The new PNG is built with <code>dev-notes/figures/fig_optimization_loop.py</code> in the code repo and copied to <code>Figures/</code> when you approve.",
     affected=["Ascent Case wording in the figure is fixed by the rebuild (&ldquo;ascent case&rdquo; in place of &ldquo;architecture&rdquo;).",
               "The list of figures loses one entry.",
               "FV-02 and FV-04 carry the text that goes with this change."],
     note="Alternative if you prefer two figures: keep both and only rebuild them with the new wording. I do not recommend it; the second would still need the fixed variables added.")

# ---------------------------------------------------------------- FV-02 define first
card("FV-02", "Ch 5 §5.2, first two paragraphs (l.284&ndash;286)", "moderate", [5, 2],
     "The figure is cited before &ldquo;ascent case&rdquo; is defined, and the reduction is told in the wrong order",
     "&ldquo;&hellip; any combination of their values and options defines an ascent. Figure&nbsp;5.x shows how this space is reduced, step by step, to the <i>design vector</i> x&hellip;&rdquo; The next paragraph then says the best trajectory is not sought, the problem is reduced, and a set of such choices is an <i>ascent case</i>.",
     "The reader is shown the end of the reduction (the design vector) in a figure before being told that a reduction is made, why, and what an ascent case is. The two terms are the ones the whole chapter rests on, and Chapters 1 and 4 point here for both (principles 5 and 2: define before use, proposition first).",
     "Join the two paragraphs: free variables, then why they cannot all be searched, then the reduction, then the definition of an ascent case. The figure moves to the loop paragraph (FV-04), where the design vector is used.",
     [(I, S(I, "The variables of Section~\\ref{sec:capabilities} that the code does not fix are \\emph{free}", "is an \\emph{ascent case}."),
       r"""The variables of Section~\ref{sec:capabilities} that the code does not fix are \emph{free}: any combination of their values and options defines an ascent. Searching all of them together is beyond the computational budget of this work, so the best possible trajectory over the whole space is not sought. The problem is reduced instead: some variables are fixed, either arbitrarily or from the literature, and the rest are left to be optimized. A set of such choices is an \emph{ascent case}.""")],
     where="<code>Thesis_Implementation.tex</code>, the first two paragraphs of §5.2, merged into one.",
     affected=["The text of the paragraph that introduces the design vector (&ldquo;What remains free&hellip;&rdquo;) is unchanged and now follows directly."])

# ---------------------------------------------------------------- FV-03 starting point and target
card("FV-03", "Ch 5 §5.2, paragraph on the guidance law (l.295)", "quick", [5, 4],
     "&ldquo;A starting point and a target&rdquo; are called variables to decide, but not whose",
     "&ldquo;Every law needs a starting point and a target to guide the vehicle to, and both are variables to decide. A closed-loop law determines its remaining variables by itself&hellip;&rdquo;",
     "The reader cannot tell if these two are variables that the law adds to the design vector, like the constants of an open-loop law in the next sentences, or ascent variables of Section 5.1 (the target orbit is listed there among the mission variables). If they are the second, the sentence does not belong to the list of what the law adds, and the word &ldquo;remaining&rdquo; after it has nothing to be the remainder of (principle 5).",
     "Say that both are ascent variables already defined in Section 5.1, which the ascent case fixes or leaves free like any other; the law adds only what follows.",
     [(I, "Every law needs a starting point and a target to guide the vehicle to, and both are variables to decide.",
       r"Every law needs a starting point and a target to guide the vehicle to; both are ascent variables of Section~\ref{ssec:cap_guidance}, and the ascent case fixes them or leaves them free like any other.")],
     where="<code>Thesis_Implementation.tex</code>, one sentence in §5.2.",
     affected=["I assumed that the starting point of a law is the hand-off or ignition variable of Section 5.1; please confirm, or tell me what you meant, before approving."],
     note="This card needs one fact from you: what &ldquo;starting point&rdquo; stands for. If it is something the law itself adds, the sentence should move to the list that follows instead.")

# ---------------------------------------------------------------- FV-04 fixed the design vector
card("FV-04", "Ch 5 §5.2, first sentence of the loop paragraph (l.301)", "quick", [9, 5],
     "&ldquo;Once an ascent case has <i>fixed</i> the design vector&rdquo; says the opposite of what an ascent case does",
     "&ldquo;Once an ascent case has fixed the design vector <b>x</b>, the computation of Figure&nbsp;5.y chooses its value, in two nested loops.&rdquo;",
     "An ascent case fixes variables and leaves <b>x</b> free; if <b>x</b> were fixed, there would be nothing to choose. The sentence also carries the first citation of the (merged) figure, which the loop paragraph should open with now that the cascade is gone.",
     "Use &ldquo;defined&rdquo; and open the paragraph with the figure, which now shows the whole path from the variables to the cost.",
     [(I, "Once an ascent case has fixed the design vector $\\mathbf{x}$, the computation of Figure~\\ref{fig:optimization_loop} chooses its value, in two nested loops.",
       r"Figure~\ref{fig:optimization_loop} summarizes the reduction and the search. Once an ascent case has defined the design vector $\mathbf{x}$, its value is chosen in two nested loops.")],
     where="<code>Thesis_Implementation.tex</code>, one sentence in §5.2.",
     affected=["Depends on the merged figure of FV-01 for the citation to make sense."])

# ---------------------------------------------------------------- FV-05 search paragraph and modularity
card("FV-05", "Ch 5 §5.2, paragraph on the number of variables (l.299) and last sentence (l.310)", "moderate", [7, 8],
     "How <b>x</b> is searched is told before the loop that does the searching; the modularity sentence is loose",
     "The paragraph &ldquo;The number of variables in <b>x</b> decides how it is searched. With one or two, the grid search with Brent refinement is used; with more&hellip; the particle swarm&hellip;&rdquo; sits between the definition of <b>x</b> and the description of the two loops. The section ends: &ldquo;The separation is deliberate to ensure the modularity of this simulator.&rdquo;",
     "The choice of search method is a property of the optimizer, the outer loop, which has not been introduced when the paragraph is read. Placed after the loops, the section goes from definition, to computation, to its consequence, and ends by pointing at the ascent cases, which is the closing it needs (principle 10). The last sentence has an unfinished purpose (&ldquo;to ensure&rdquo; what the separation does) and &ldquo;this simulator&rdquo; is vaguer than the context allows (principles 8 and 11).",
     "Move the paragraph to the end of the section, unchanged, and reword the modularity sentence so that it says what the separation allows.",
     [(I, S(I, "The number of variables in $\\mathbf{x}$ decides how it is searched.", "from another.\n\n"), ""),
      (I, "The separation is deliberate to ensure the modularity of this simulator.",
       r"""The separation is deliberate: it keeps the simulator modular, so that the optimizer or the guidance law can be changed without touching the other.

The number of variables in $\mathbf{x}$ decides how it is searched. With one or two, the grid search with Brent refinement is used; with more, its cost grows exponentially with the number of variables and the particle swarm is used instead, followed by a local refinement of the results found (Section~\ref{sec:optimization_algorithms}). Fixing variables is therefore what makes the optimization possible, and the choice of what to fix is what distinguishes one ascent case from another.""")],
     where="<code>Thesis_Implementation.tex</code>, §5.2: the paragraph moves two paragraphs down, the last sentence is reworded.",
     affected=["The commented-out cost sentence below the modularity sentence is handled in FV-06."])

# ---------------------------------------------------------------- FV-06 title and leftovers
card("FV-06", "Ch 5 §5.2 heading and two commented blocks (l.278&ndash;282, l.311)", "quick", [4, 1],
     "The title repeats the name of §5.3, and two commented-out blocks remain",
     "&ldquo;5.2 Free Variables, Ascent Cases and the Optimization Loop&rdquo; and &ldquo;5.3 Ascent Cases&rdquo;. Under the heading, a commented introduction (&ldquo;The guidance laws of Chapter 4 leave some quantities free&hellip;&rdquo;); under the last paragraph, a commented sentence on what the cost scores.",
     "Two sections of one chapter carry the same words in their titles, while 5.2 is about the free variables and the loop and 5.3 about the cases themselves (principle 4). The commented blocks are leftovers that a later reader would take for text to be restored; the cost sentence is already said by the loop paragraph (&ldquo;the cost $J$ is computed from the final state&rdquo;), and its form is given with each optimizer in Chapter 3.",
     "Retitle 5.2 &ldquo;Free Variables and the Optimization Loop&rdquo; and delete both commented blocks. The two labels on the heading stay, since the Appendix, Chapter 4 and Chapter 6 use <code>sec:optimization</code>.",
     [(I, S(I, "\\section{Free Variables, Ascent Cases and the Optimization Loop}", "how they are applied.\n\n"),
       "\\section{Free Variables and the Optimization Loop}\n\\label{sec:free_variables}\n\\label{sec:optimization}\n\n"),
      (I, S(I, "\n%and a comparison between results changes one of them at a time.", "ssec:cost_function}."), "")],
     where="<code>Thesis_Implementation.tex</code>, the heading of §5.2 and the end of the section.",
     affected=["The chapter introduction (&ldquo;Section 5.2 reduces them to a design vector&hellip;&rdquo;) already fits the new title."])
