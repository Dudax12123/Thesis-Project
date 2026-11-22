import re, glob, os, sys
os.chdir(r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf")
pat = sys.argv[1] if len(sys.argv) > 1 else r"[Aa]rchitectures?"
files = sorted(glob.glob("Thesis_*.tex")) + sorted(glob.glob("ExtendedAbstract/*.tex"))
tot = 0
for f in files:
    if f == "Thesis_Implementation.tex":
        continue
    for i, l in enumerate(open(f, encoding="utf8").read().split("\n"), 1):
        if l.lstrip().startswith("%"):
            continue
        for m in re.finditer(r"(?<![\w/_:\\])(" + pat + r")\b", l):
            tot += 1
            print(f"{f}:{i}: ...{l[max(0, m.start() - 45):m.end() + 45]}")
print(tot)
