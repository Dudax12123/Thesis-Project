import re, glob
R = r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf"
t = ""
for f in glob.glob(R + r"\*.tex") + glob.glob(R + r"\ExtendedAbstract\*.tex"):
    t += "\n".join(l for l in open(f, encoding="utf-8").read().split("\n") if not l.lstrip().startswith("%")) + "\n"
# drop comment environments
t = re.sub(r"\\begin\{comment\}.*?\\end\{comment\}", "", t, flags=re.S)
labs = re.findall(r"\\label\{([^}]+)\}", t)
refs = set(re.findall(r"\\(?:ref|eqref|pageref)\{([^}]+)\}", t))
print("duplicate labels:", sorted({l for l in labs if labs.count(l) > 1}))
print("missing labels:", sorted(r for r in refs if r not in set(labs)))
