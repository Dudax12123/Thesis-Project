import re, pathlib
h = pathlib.Path(__file__).parent
s = (h / "build.py").read_text(encoding="utf8")
s = s.replace("import content as C", "import content_open as C")
s = s.replace("from overview import OVERVIEW, QUESTIONS_NOTE  # noqa", "from overview_open import OVERVIEW")
a = s.index("TABS = [")
b = s.index("SIZE = {")
s = s[:a] + 'TABS = [("open", "Open items", "The two cards still without an answer and the two drafts from the renaming.")]\n' + s[b:]
s = s.replace("Writing Review Ch 1–4\", \"Writing Review Ch 5\"", "Writing Review Ch 1–4\", \"Chapter 5 Open Items\"")
s = s.replace('"wr5-state-v1"', '"wr5open-state-v1"')
s = s.replace("page.html", "page_open.html")
s = s.replace("<h1>Writing review, Chapter 5</h1>", "<h1>Chapter 5, open items</h1>")
s = re.sub(r'<p class="sub">.*?</p>', '<p class="sub">{N} cards in one section: two still unanswered and two drafts. Each carries a recommendation and the proposed LaTeX as tracked changes.</p>', s, count=1)
(h / "build_open.py").write_text(s, encoding="utf8")
