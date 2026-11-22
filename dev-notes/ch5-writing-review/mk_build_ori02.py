import re, pathlib
h = pathlib.Path(__file__).parent
s = (h / "build.py").read_text(encoding="utf8")
s = s.replace("import content as C", "import content_ori02 as C")
s = s.replace("from overview import OVERVIEW, QUESTIONS_NOTE  # noqa", "from overview_ori02 import OVERVIEW")
a = s.index("TABS = [")
b = s.index("SIZE = {")
s = s[:a] + 'TABS = [("ori", "ORI-02 additions", "Four short additions so that the option tables of Chapter 5 have a place to point to.")]\n' + s[b:]
s = s.replace("Writing Review Ch 1–4\", \"Writing Review Ch 5\"", "Writing Review Ch 1–4\", \"ORI-02 Additions\"")
s = s.replace('"wr5-state-v1"', '"wr5ori02-state-v1"')
s = s.replace("page.html", "page_ori02.html")
s = s.replace("<h1>Writing review, Chapter 5</h1>", "<h1>ORI-02 additions</h1>")
s = re.sub(r'<p class="sub">.*?</p>', '<p class="sub">Four proposed additions to Chapters 2, 4 and 5, each with its tracked-change edit. Nothing is written in the thesis until you approve.</p>', s, count=1)
(h / "build_ori02.py").write_text(s, encoding="utf8")
