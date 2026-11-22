import re, content as C
for c in C.CARDS:
    for f in ("title","reading","rec","note"):
        v=c.get(f) or ""
        if re.search(r"[Aa]scent [Cc]ase",v): print(c["id"],f,"::",v[:400]);print()
    for a in c["affected"]:
        if re.search(r"[Aa]scent [Cc]ase",a): print(c["id"],"affected ::",a[:300]);print()
