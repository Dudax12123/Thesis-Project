import sys, io, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent))
import ch5

STATUS = {
 "CH5-00": ("Approved", "Partly implemented", "Opening paragraph of Chapter 6 and the 5.1 text (thesis 69cea40), baseline mission (d96ce1e). The rest follows the other cards."),
 "CH5-01": ("Approved", "Implemented", "Thesis 69cea40 (pushed to Overleaf)."),
 "CH5-02": ("Approved", "Implemented", "Thesis d96ce1e (pushed to Overleaf)."),
 "CH5-03": ("Rejected", "Nothing to do", "The mission stays a 500 km circular orbit. The 280 km re-flight is not done."),
 "CH5-04": ("Not answered", "To implement once approved", "Audit of the engine-model and fairing subsections with six insertions (I1 to I6)."),
 "CH5-05": ("Not answered", "To implement once approved", "Dissolve 5.2 into Chapter 4."),
 "CH5-06": ("Approved", "Nothing to implement", "5.3 Experimental Design is parked. The card only records what it depends on."),
 "CH5-07": ("Approved", "Approved, not yet implemented", "5.4 Numerical Integration and Event Detection to the appendix, plus one sentence in Chapter 4."),
 "CH5-08": ("Not answered", "To implement once approved", "Delete the commented-out blocks of Thesis_Methodology.tex, keep a copy in dev-notes."),
}
ORDER = [c["id"] for c in ch5.CARDS]
by = {c["id"]: c for c in ch5.CARDS}

out = []
out.append("# Chapter 5 restructuring: review cards and status\n")
out.append("Exported 2026-11-01 from the artifact page *Supervisor Annotation Review*, tab **Chapter 5 restructuring**. "
           "The page is https://claude.ai/artifact/Mw2HC2yUsWpezUBANTWUdt (private, owned by the account that published it). "
           "This file holds the whole tab so the work can continue from another account without the page.\n")
out.append("## Where things stand\n")
out.append("Thesis repo: `C:\\Users\\eduar\\Desktop\\Tese\\Thesis_Overleaf` (Overleaf git bridge), head **d96ce1e** on `main`, nothing uncommitted. "
           "Two commits came from this work: **69cea40** (Chapter 6 opening and the 5.1 text) and **d96ce1e** (the baseline mission). "
           "Both are pushed. No compile was run, only a label sweep (`sweep.py`), which found no new missing or duplicate labels.\n")
out.append("Chapter 6 now opens with a section *Experimental Setup* (label `sec:vehicle`) holding the stage table, the environment table, the lift, drag and path-constraint paragraphs, and the baseline mission as its last subsection. "
           "Chapter 5 (`Thesis_Methodology.tex`) still holds the engine-model and fairing subsections under a section heading, 5.2 Guidance Laws as Implemented, 5.3 Experimental Design and 5.4 Numerical Integration and Event Detection.\n")
out.append("| Card | Subject | Your decision | Implementation status | Note |\n| --- | --- | --- | --- | --- |")
for cid in ORDER:
    d, s, n = STATUS[cid]
    out.append(f"| [{cid}](#{cid.lower()}) | {by[cid]['title']} | {d} | {s} | {n} |")
out.append("")
out.append("**Resolved (answered):** CH5-00, 01, 02, 03, 06, 07. **Implemented:** CH5-01, CH5-02, and the parts of CH5-00 that they cover. "
           "**Still to implement:** CH5-07 (approved), and CH5-04, CH5-05, CH5-08 once you answer them. **Nothing to do:** CH5-03, CH5-06.\n")
out.append("## Rules from the earlier work that apply\n")
out.append("- Fetch Overleaf before editing (`git fetch`, read the incoming diff). Never handle the Overleaf password or token; mask it with `sed -E 's#//[^@]*@#//***@#'`.\n"
           "- No LaTeX toolchain on this machine: verify with a `\\ref` / `\\label` / `\\cite` sweep.\n"
           "- The working copies are CRLF while the index is LF. Edit with scripts that detect the line ending, and put LaTeX in raw strings in a script file, never in a heredoc.\n"
           "- An Implement request means: apply, commit and push in the same turn. Commit messages end with the Co-Authored-By line.\n"
           "- Review cards show the whole affected text with additions green (`<ins>`) and removals red (`~~`).\n"
           "- Do not republish the page over unsaved answers without asking.\n")
out.append("## Cards\n")
out.append(ch5.INTRO.replace("<ins>Green</ins> is added", "Green (`<ins>`) is added").replace("~~red~~ is removed", "red (`~~`) is removed") + "\n")
for cid in ORDER:
    c = by[cid]
    d, s, n = STATUS[cid]
    out.append(f"<a id=\"{cid.lower()}\"></a>\n")
    out.append(f"### {cid}. {c['title']}\n")
    out.append(f"*{c['where']} · {c['size']} · **Decision:** {d} · **Status:** {s}*\n")
    out.append(c["md"].replace("\n#### ", "\n##### ").replace("#### ", "##### ", 1) + "\n")
txt = "\n".join(out)
for target in (r"C:\Users\eduar\Desktop\Chapter 5 Restructuring.md", r"C:\Users\eduar\Desktop\Tese\Code\Thesis-Project\dev-notes\chapter5-restructuring-2026-11-01.md"):
    io.open(target, "w", encoding="utf-8", newline="\n").write(txt)
print(len(txt))
