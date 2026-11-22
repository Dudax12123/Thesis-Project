import content_open as C
ids = {"MCIT-01", "MCIT-02", "MCIT-03", "BIB-01", "TERM-05"}
for c in C.CARDS:
    if c["id"] not in ids:
        continue
    for fk, old, new in c["edits"]:
        p = C.ROOT / C.FILES[fk]
        t = p.read_text(encoding="utf8")
        assert t.count(old) == 1, (c["id"], old[:60])
        p.write_text(t.replace(old, new), encoding="utf8", newline="")
        print("ok", c["id"], fk)
