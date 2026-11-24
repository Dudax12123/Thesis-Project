"""Builds the Chapter 5 writing-review page from content.py and the Ch 1-4 page template.
Usage: python build.py [--check]   (--check only verifies the edits)"""
import re, sys, html, difflib, pathlib, json

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
import content_52 as C
from overview_52 import OVERVIEW

CARDS = C.CARDS
ROOT = C.ROOT

TABS = [("s52", "Section 5.2", "Figures and text of Section 5.2, Free Variables and the Optimization Loop.")]
SIZE = {"quick": "Quick", "moderate": "Moderate", "structural": "Structural"}
PNAME = {1: "Orient the reader first", 2: "State the main proposition early", 3: "Keep related information close",
         4: "One stable name per concept", 5: "Define before use, and say how it is measured",
         6: "Recall details retrieved from far away", 7: "Progress argumentatively", 8: "Vary rhythm by function, not terminology",
         9: "Match the strength of claims to the evidence", 10: "Close a subject by its role", 11: "Be concise without losing precision"}


def esc(s):
    return html.escape(s, quote=False)


# ------------------------------------------------------------------ verification
def verify():
    bad = 0
    spans = {}
    for c in CARDS:
        for k, (fk, old, new) in enumerate(c["edits"]):
            t = C.text(fk)
            n = t.count(old)
            if n != 1:
                print("EDIT NOT UNIQUE", c["id"], k, fk, n, repr(old[:80]))
                bad += 1
                continue
            i = t.index(old)
            spans.setdefault(fk, []).append((i, i + len(old), c["id"], k))
    for fk, L in spans.items():
        L.sort()
        for a, b in zip(L, L[1:]):
            if b[0] < a[1]:
                print("OVERLAP", fk, a, b)
                bad += 1
    # cites and refs in new text
    bib = (ROOT / C.FILES["BIB"]).read_text(encoding="utf8")
    keys = set(re.findall(r"@\w+\{([^,]+),", bib))
    labels = set()
    for p in ROOT.glob("Thesis_*.tex"):
        tx = p.read_text(encoding="utf8")
        labels |= set(re.findall(r"\\label\{([^}]*)\}", tx))
    newlabels = set()
    for c in CARDS:
        for fk, old, new in c["edits"]:
            newlabels |= set(re.findall(r"\\label\{([^}]*)\}", new))
    for c in CARDS:
        for fk, old, new in c["edits"]:
            if fk == "BIB":
                continue
            for m in re.finditer(r"\\cite\w*\{([^}]*)\}", new):
                for k in m.group(1).split(","):
                    if k.strip() not in keys:
                        print("MISSING KEY", c["id"], k.strip())
                        bad += 1
            for m in re.finditer(r"\\(?:ref|eqref)\{([^}]*)\}", new):
                if m.group(1) not in labels | newlabels:
                    print("MISSING LABEL", c["id"], m.group(1))
                    bad += 1
    ids = [c["id"] for c in CARDS]
    assert len(ids) == len(set(ids)), "duplicate ids"
    return bad


# ------------------------------------------------------------------ diff rendering
def worddiff(old, new):
    a = re.findall(r"\s+|\S+", old)
    b = re.findall(r"\s+|\S+", new)
    sm = difflib.SequenceMatcher(None, a, b, autojunk=False)
    out = []
    for op, i1, i2, j1, j2 in sm.get_opcodes():
        if op == "equal":
            out.append(esc("".join(a[i1:i2])))
        else:
            if i2 > i1:
                out.append("<del>" + esc("".join(a[i1:i2])) + "</del>")
            if j2 > j1:
                out.append("<ins>" + esc("".join(b[j1:j2])) + "</ins>")
    return "".join(out)


def card_html(c):
    id_ = c["id"]
    h = []
    h.append(f'<article class="card" id="{id_}" data-id="{id_}" data-size="{c["size"]}"><header><div class="idrow"><span class="idchip">{id_}</span><span class="pg">{c["loc"]}</span><span class="size s-{c["size"]}">{SIZE[c["size"]]}</span><span class="status" data-status>Open</span></div><h3>{c["title"]}</h3></header>')
    h.append('<section class="prof guide"><h4>Writing guide</h4><ul class="plist">' + "".join(
        f'<li><span class="kind">Principle {p}</span><div class="pt">{PNAME[p]}</div></li>' for p in c["tags"]) + "</ul></section>")
    h.append(f'<section class="blk"><h4>What it refers to</h4><p>{c["refers"]}</p></section>')
    h.append(f'<section class="blk"><h4>Reading</h4><p>{c["reading"]}</p></section>')
    h.append(f'<section class="blk rec"><h4>Recommendation</h4><p>{c["rec"]}</p></section>')
    E = c["edits"]
    if c.get("note"):
        h.append(f'<section class="blk"><h4>Note</h4><p>{c["note"]}</p></section>')
    if E:
        n = len(E)
        h.append('<section class="blk"><h4>Proposed change in context</h4><p class="legend"><del>Red</del> is removed from the current text, <ins>green</ins> is added. Every removed passage was checked to occur exactly once in the current source.</p>')
        for k, (fk, old, new) in enumerate(E):
            h.append(f'<details class="diffbox" open><summary><span class="dl">Edit {k+1} of {n}</span><span class="df">{C.FILES[fk]}</span></summary><div class="diff">{worddiff(old, new)}</div>'
                     f'<div class="ibtns" data-item="e{k}" role="group" aria-label="Decision for edit {k+1}"><span>This edit:</span><button type="button" data-d="approve">Approve</button><button type="button" data-d="change">Change</button><button type="button" data-d="reject">Reject</button></div></details>')
        h.append("</section>")
        h.append('<section class="blk edit"><h4>Edit the proposed text</h4><p class="legend">Change the LaTeX to what you want; it is saved with your answer and is what I apply. The tracked-change view above does not update.</p>')
        for k, (fk, old, new) in enumerate(E):
            rows = max(3, min(18, new.count("\n") + 2))
            h.append(f'<details class="edbox"><summary><span class="dl">Edit {k+1} ({C.FILES[fk]})</span><span class="edflag" data-edflag="{k}" hidden>edited</span></summary><textarea class="edta" data-i="{k}" rows="{rows}" spellcheck="false" aria-label="Edit {k+1}">{esc(new)}</textarea><button type="button" class="edreset" data-i="{k}">Reset to the proposal</button></details>')
        h.append("</section>")
    else:
        h.append('<section class="blk"><h4>Proposed change in context</h4><p>No LaTeX is proposed for this card; it needs your answer or is a finding only. Use Change and the comment box to tell me what you want.</p></section>')
    where = c["where"] or "&mdash;"
    dep = "".join(
        f'<li><b>Depends on <a href="#{o}" data-goto="{o}">{o}</a>:</b> {r}</li>' if a == id_ else
        f'<li><b>Linked to <a href="#{a}" data-goto="{a}">{a}</a>:</b> {r}</li>'
        for a, o, r in C.DEPS if id_ in (a, o) for a, o in [(a, o if a == id_ else o)] if True)
    aff = dep + "".join(f"<li>{a}</li>" for a in c["affected"])
    h.append(f'<section class="meta"><div><h4>Where the change is made</h4><p>{where}</p></div><div><h4>What else is affected</h4><ul>{aff}</ul></div></section>')
    h.append(f'<footer class="decide"><div class="btns" role="group" aria-label="Decision for {id_}"><button type="button" data-d="approve">Approve</button><button type="button" data-d="change">Change</button><button type="button" data-d="reject">Reject</button></div>'
             f'<div class="impl"><button type="button" class="implbtn" data-impl aria-pressed="false">Implement this card</button><span class="implnote" data-implnote>Sends only this card to Claude, who implements it at once.</span></div>'
             f'<label class="cmt" for="cm-{id_}"><span>Your comment</span><textarea id="cm-{id_}" rows="2" placeholder="Optional: your version, a condition, or why"></textarea></label><div class="saved" data-saved aria-live="polite"></div></footer></article>')
    return "".join(h)


def build():
    t = (HERE / "ch14_page_template.html").read_text(encoding="utf8")
    # strip the publishing skeleton
    t = t[t.index("</head><body>") + len("</head><body>"):]
    t = t[: t.rindex("</body></html>")].rstrip()
    while t.endswith("</body></html>"):
        t = t[: -len("</body></html>")].rstrip()
    t = t.replace("Writing Review Ch 1–4", "Chapter 5 Section 5.2 Review")
    pre = t[: t.index('<div class="wrap">')]
    script = t[t.index("<script>"):]
    N = len(CARDS)
    ids = [c["id"] for c in CARDS]
    secof = {c["id"]: c["tab"] for c in CARDS}
    nedit = sum(len(c["edits"]) for c in CARDS)
    script = re.sub(r"var IDS = \[.*?\];", "var IDS = " + json.dumps(ids) + ";", script, count=1, flags=re.S)
    script = re.sub(r"var SECOF = \{.*?\};", "var SECOF = " + json.dumps(secof) + ";", script, count=1, flags=re.S)
    script = re.sub(r"var TOTAL = \d+;", f"var TOTAL = {N};", script, count=1)
    script = script.replace('"wr14-state-v1"', '"wr52-state-v1"')
    nav = '<a class="tab" href="#overview" data-tab="overview">Overview</a>' + "".join(
        f'<a class="tab" href="#{k}" data-tab="{k}">{name} <span class="n">{sum(1 for c in CARDS if c["tab"] == k)}</span></a>' for k, name, _ in TABS) + '<a class="tab" href="#list" data-tab="list">All cards</a>'
    head = f'''<div class="wrap">
  <header class="top">
    <div>
      <h1>Chapter 5, Section 5.2</h1>
      <p class="sub">{N} cards on the figures and the text of Section 5.2. Each carries a recommendation and the proposed LaTeX as tracked changes.</p>
    </div>
    <div class="progress" aria-label="Progress">
      <div class="bar"><i class="a" id="bar-a"></i><i class="c" id="bar-c"></i><i class="r" id="bar-r"></i></div>
      <div class="txt" id="prog-txt">0 of {N} answered</div>
    </div>
  </header>
  <div class="shell">
    <nav class="tabs" aria-label="Sections">{nav}</nav>
    <main>
'''
    toolbar = '''      <div class="toolbar" id="toolbar" hidden>
        <button type="button" data-f="all" aria-pressed="true">All</button>
        <button type="button" data-f="open" aria-pressed="false">Open</button>
        <button type="button" data-f="approve" aria-pressed="false">Approved</button>
        <button type="button" data-f="change" aria-pressed="false">Change</button>
        <button type="button" data-f="reject" aria-pressed="false">Rejected</button>
        <label for="sizef" class="sr" style="position:absolute;left:-9999px">Size</label>
        <select id="sizef"><option value="all">Any size</option><option value="quick">Quick edits</option><option value="moderate">Moderate</option><option value="structural">Structural</option></select>
      </div>
'''
    ov = OVERVIEW.format(N=N, NEDIT=nedit, ROWS="".join(
        f'<tr><td><a href="#{k}" data-tab="{k}">{name}</a></td><td>{sum(1 for c in CARDS if c["tab"] == k)}</td><td>{desc}</td></tr>' for k, name, desc in TABS),
        FILES=files_table(), DEPS="".join(f'<li><a href="#{a}" data-goto="{a}">{a}</a> and <a href="#{o}" data-goto="{o}">{o}</a>: {r}</li>' for a, o, r in C.DEPS))
    panels = [f'<section class="panel" id="p-overview" data-panel="overview" hidden>{ov}</section>']
    for k, name, desc in TABS:
        cs = [c for c in CARDS if c["tab"] == k]
        panels.append(f'<section class="panel" id="p-{k}" data-panel="{k}" hidden><div class="phead"><h2>{name}</h2><p>{desc}</p></div>' + "".join(card_html(c) for c in cs) + "</section>")
    rows = ""
    for k, name, _ in TABS:
        rows += f'<tr><th colspan="4">{name}</th></tr>'
        for c in CARDS:
            if c["tab"] == k:
                rows += f'<tr><td><a href="#{c["id"]}" data-goto="{c["id"]}">{c["id"]}</a></td><td>{SIZE[c["size"]]}</td><td>{c["loc"]}</td><td>{c["title"]}</td></tr>'
    panels.append(f'<section class="panel" id="p-list" data-panel="list" hidden><div class="phead"><h2>All cards</h2><p>Every card in one list, by tab.</p></div><div class="tablewrap"><table class="mini"><tr><th>Card</th><th>Size</th><th>Where</th><th>Problem</th></tr>{rows}</table></div></section>')
    tail = "\n    </main>\n  </div>\n</div>\n"
    out = pre + head + toolbar + "\n".join(panels) + tail + script + "\n"
    (HERE / "page_52.html").write_text(out, encoding="utf8")
    print("cards", N, "edits", nedit, "bytes", len(out))


def files_table():
    byfile = {}
    for c in CARDS:
        for fk in {e[0] for e in c["edits"]}:
            byfile.setdefault(fk, []).append(c["id"])
    rows = ""
    for fk, L in byfile.items():
        rows += f'<tr><td><code>{C.FILES[fk]}</code></td><td>' + " ".join(f'<a href="#{i}" data-goto="{i}">{i}</a>' for i in L) + "</td></tr>"
    return rows


if __name__ == "__main__":
    bad = verify()
    print("problems:", bad)
    if "--check" not in sys.argv and bad == 0:
        build()
