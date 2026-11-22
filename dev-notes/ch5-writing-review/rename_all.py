import re, glob, os, sys
R = r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf"
os.chdir(R)
APPLY = "--apply" in sys.argv
LB = r"(?<![\w/_:\\])"

RULES = [
    (r"Optimization Architecture(s?)\b", lambda m: "Ascent Case" + m.group(1)),
    (r"[Oo]ptimization architecture(s?)\b", lambda m: "ascent case" + m.group(1)),
    (r"Cost of the Architectures\b", lambda m: "Cost of the Ascent Cases"),
    (r"Architectures that supply", lambda m: "Ascent cases that supply"),
    (r"Architecture(\s+&)", lambda m: "Ascent case" + m.group(1)),
    (r"[Aa]rchitectures ([A-D])\b", lambda m: "ascent cases " + m.group(1)),
    (r"[Aa]rchitecture ([A-D])\b", lambda m: "ascent case " + m.group(1)),
    (r"architecture(s?)\b", lambda m: "ascent case" + m.group(1)),
]
SKIP = {"Thesis_Implementation.tex"}
for f in sorted(glob.glob("Thesis_*.tex")):
    if f in SKIP:
        continue
    lines = open(f, encoding="utf8", newline="").read().split("\n")
    out, ch = [], 0
    for i, l in enumerate(lines, 1):
        if l.lstrip().startswith("%") or "staging architecture" in l:
            out.append(l)
            continue
        n = l
        for pat, fn in RULES:
            n = re.sub(LB + pat, fn, n)
        if n != l:
            ch += 1
            if not APPLY:
                print(f"{f}:{i}\n  - {l.strip()[:200]}\n  + {n.strip()[:200]}")
        out.append(n)
    if APPLY and ch:
        open(f, "w", encoding="utf8", newline="").write("\n".join(out))
    print(f, ch)
