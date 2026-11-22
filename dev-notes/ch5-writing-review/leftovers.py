import re
import content as C

for fk in ("IMPL", "GUID"):
    t = C.text(fk)
    for c in C.CARDS:
        for f, o, n in c["edits"]:
            if f == fk:
                t = t.replace(o, n, 1)
    print("==", fk)
    for i, l in enumerate(t.split("\n"), 1):
        if l.lstrip().startswith("%"):
            continue
        for m in re.finditer(r"(?<![\w/_:\\])([Aa]rchitectures?|Cases? [A-D]|[Cc]ases?)\b", l):
            print(i, m.group(), "|", l[max(0, m.start() - 50):m.end() + 40])
t = [c for c in C.CARDS if c["id"] == "TERM-01"][0]
print(len(t["edits"]))
for f, o, n in t["edits"][:6]:
    print(repr(o[:110]), "->", repr(n[:110]))
