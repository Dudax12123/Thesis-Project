import re, os, sys
R = r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf"
os.chdir(R)
APPLY = "--apply" in sys.argv
LB = r"(?<![\w/_:\\])(?<![Aa]scent )"
# file -> lines (1-based) NOT to touch
EXCL = {
    "Thesis_Results.tex": {473},
    "Thesis_Appendix_Capabilities.tex": {154, 161},
    "Thesis_Conclusions.tex": {153},
}
FILES = ["Thesis_Abstract.tex", "Thesis_Appendix_Capabilities.tex", "Thesis_Appendix_Supporting.tex",
         "Thesis_Conclusions.tex", "Thesis_Introduction.tex", "Thesis_Results.tex"]
SPEC = {"Thesis_Introduction.tex": {119}, "Thesis_Abstract.tex": {26},
        "Thesis_Appendix_Supporting.tex": {4, 11}}
for f in FILES:
    lines = open(f, encoding="utf8", newline="").read().split("\n")
    out = []
    for i, l in enumerate(lines, 1):
        n = l
        if not l.lstrip().startswith("%") and i not in EXCL.get(f, set()):
            n = re.sub(LB + r"Cases\b", "Runs", n)
            n = re.sub(LB + r"Case\b", "Run", n)
            n = re.sub(LB + r"case(s?)\b", lambda m: "run" + m.group(1), n)
            n = n.replace("an run", "a run")
        if n != l:
            print(f"{f}:{i}\n  - {l.strip()[:230]}\n  + {n.strip()[:230]}")
        out.append(n)
    if APPLY:
        open(f, "w", encoding="utf8", newline="").write("\n".join(out))
