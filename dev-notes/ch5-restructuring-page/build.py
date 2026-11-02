import re, json, pathlib, sys
SRC = pathlib.Path(r"C:\Users\eduar\Desktop\Supervisor Annotation Review.md")
OUT = pathlib.Path(__file__).parent / "supervisor-annotation-review.html"
TPL = (pathlib.Path(__file__).parent / "template.html").read_text(encoding="utf-8")

L = SRC.read_text(encoding="utf-8").split("\n")
ID = r"(X\d-\d+|X\d|C\d-\d+A?|F-\d+|RN-\d+|FG-\d+|OF-\d+|TF-\d+|R-\d+)"
HDR = re.compile("^" + ID + r"(.*?)(Quick edit|Moderate|Structural)(Open|Done|Skipped|Applied)$")

# section headings (##) in file order
sec_lines = [(i, l[3:].strip()) for i, l in enumerate(L) if l.startswith("## ")]
SECTION_IDS = {
    "Cross-cutting": "x", "Front matter": "front", "Chapter 1": "ch1", "Chapter 2": "ch2",
    "Chapter 4 (was 3)": "ch3", "Architecture renaming": "rn", "Figures": "fg",
}
def sec_id(title):
    if title in SECTION_IDS: return SECTION_IDS[title]
    if title.startswith("Ch. 4") and "4.2" in title: return "of"
    if title.startswith("Ch. 4") and "4.3" in title: return "tf"
    if title.startswith("Ch. 3"): return "re"
    if title == "All annotations": return "list"
    return None

hdr_idx = [i for i in range(2, len(L)) if L[i].startswith("### ") and L[i-1].strip() == "" and L[i-2].strip()]
hdr_idx = [i - 2 for i in hdr_idx]
assert len(hdr_idx) == 147, len(hdr_idx)

def clean(lines):
    out = [l for l in lines if l.strip() != "Your comment"]
    while out and not out[-1].strip(): out.pop()
    while out and not out[0].strip(): out.pop(0)
    return "\n".join(out)

sections = []
for k, (i, title) in enumerate(sec_lines):
    sid = sec_id(title)
    if not sid: continue
    end = sec_lines[k + 1][0] if k + 1 < len(sec_lines) else len(L)
    cards_here = [h for h in hdr_idx if i < h < end]
    intro_end = cards_here[0] if cards_here else end
    intro = clean(L[i + 1:intro_end])
    cards = []
    for n, h in enumerate(cards_here):
        stop = cards_here[n + 1] if n + 1 < len(cards_here) else end
        m = HDR.match(L[h])
        assert m, L[h]
        cid, where, size, status = m.groups()
        ttl = L[h + 2][4:].strip()
        body = clean(L[h + 3:stop])
        cards.append(dict(id=cid, where=where.strip(), size=size, status=status, title=ttl, md=body))
    sections.append(dict(id=sid, title=title, intro=intro, cards=cards))

import ch5
sections.insert(0, dict(id='c5', title='Chapter 5 restructuring', intro=ch5.INTRO, cards=ch5.CARDS))
overview = clean(L[8:82])
data = dict(overview=overview, sections=sections)
total = sum(len(s["cards"]) for s in sections)
assert total == 147 + len(ch5.CARDS), total
blob = json.dumps(data, ensure_ascii=False).replace("</", "<\\/").replace("\u2028", "\\u2028").replace("\u2029", "\\u2029")
html = TPL.replace("/*__DATA__*/null", blob)
OUT.write_text(html, encoding="utf-8")
print(OUT, len(html), total, [(s["id"], len(s["cards"])) for s in sections])
