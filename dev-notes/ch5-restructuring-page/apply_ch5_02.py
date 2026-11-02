import io
R = r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf"
EOL = {}
def rd(n):
    s = io.open(R + "\\" + n, encoding="utf-8", newline="").read()
    EOL[n] = "\r\n" if "\r\n" in s else "\n"
    return s.replace("\r\n", "\n")
def wr(n, s): io.open(R + "\\" + n, "w", encoding="utf-8", newline="").write(s.replace("\n", EOL[n]))

m = rd("Thesis_Methodology.tex")
i = m.index("\\subsection{Baseline Mission}")
k = m.index("\\begin{comment}", i)
j = m.rindex("\n", 0, k - 1) + 1
block = m[i:j]
assert block.rstrip().endswith("specific mission is substituted.")
m = m[:i] + m[j:]
wr("Thesis_Methodology.tex", m)

old = "every case of\nChapter~\\ref{chapter:results_and_discussion} flies the baseline mission."
assert block.count(old) == 1
block = block.replace(old, "every case in this\nchapter flies the baseline mission.")

r = rd("Thesis_Results.tex")
bar = "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n\\section{The Optimal Reference}"
assert r.count(bar) == 1
r = r.replace(bar, block.rstrip() + "\n\n" + bar)
wr("Thesis_Results.tex", r)
print("ok")
