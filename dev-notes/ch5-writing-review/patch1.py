import re, pathlib
p = pathlib.Path(r"C:\Users\eduar\Desktop\Tese\Code\Thesis-Project\dev-notes\ch5-writing-review\content.py")
s = p.read_text(encoding="utf8")

def cut(s, a, b, new):
    i = s.index(a); j = s.index(b)
    return s[:i] + new + s[j:]

ORG01 = '''card("ORG-01", "org", "Ch 5 §5.3 heading (l.309–311)", "quick", [1, 4],
     "Section 5.3 carries a dead label, <code>ssec:integration</code>",
     "<code>\\\\section{Ascent Cases}</code> opens with &ldquo;This section sets out the four architectures compared in this work.&rdquo; The heading carries two labels, <code>sec:trajectory_framework</code> and <code>ssec:integration</code>. The second names a subsection about numerical integration that no longer exists, and nothing refers to it.",
     "The title &ldquo;Ascent Cases&rdquo; is the name you chose for the four (TERM-01) and stays. The stale label is a leftover (check 7); a later reader would take it for a section about integration.",
     "Drop the dead label and keep <code>sec:trajectory_framework</code>, which the Appendix still uses.",
     [(I, S(I, r"\\section{Ascent Cases}", r"\\label{ssec:integration}"),
       r"""\\section{Ascent Cases}
\\label{sec:trajectory_framework}""")],
     "Thesis_Implementation.tex, l.309–311", ["Label <code>ssec:integration</code> is referenced nowhere (checked in all chapters and appendices)."])

'''
s = cut(s, 'card("ORG-01"', 'card("ORG-02"', ORG01)

TERM01 = '''card("TERM-01", "term", "Ch 5, every place that says &ldquo;architecture&rdquo; or &ldquo;Case&rdquo; (about 30)", "moderate", [4],
     "The four are named &ldquo;Ascent Cases&rdquo; (your decision); the chapter still says &ldquo;architecture&rdquo; and a bare &ldquo;Case&rdquo;",
     "Section 5.3 is titled &ldquo;Ascent Cases&rdquo;, the subsections &ldquo;Architecture A&rdquo; to &ldquo;Architecture D&rdquo;, and the text &ldquo;Case A&rdquo;, &ldquo;each case&rdquo;, &ldquo;the four cases above&rdquo; as well as &ldquo;each architecture&rdquo;, &ldquo;the segmented architecture&rdquo;. The waterfall figure prints &ldquo;Case A Defines&rdquo;.",
     "Three names for one object (principle 4). You chose <b>Ascent Case</b>: &ldquo;Ascent Case A&rdquo; as the name and &ldquo;ascent case&rdquo; in running text. Every other card of this page already writes its proposed text with that name.",
     "Rename in Chapter 5 only. The edits below are generated from the source: one per run of occurrences, outside comments and outside the passages that other cards rewrite (those cards already use the new name).",
     TERM01_EDITS,
     "Thesis_Implementation.tex, the lines listed in the edits", TERM01_AFFECTED)

'''
s = cut(s, 'card("TERM-01"', 'card("TERM-02"', TERM01)
p.write_text(s, encoding="utf8")
print("ok")
