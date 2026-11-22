import pathlib
p=pathlib.Path(r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf\Thesis_Ascent_Background.tex")
t=p.read_text(encoding="utf8")
i=t.index("\n\nThe altitude dependence of the first stage can also be modeled in less detail.")
j=t.index("\n\\begin{equation}",i)
d1=t[i:j]
assert d1.count("\n")==2
t=t[:i]+"\n"+t[j:]
old="The simulator also offers a simpler model, with thrust and specific impulse held at their sea-level values."
assert t.count(old)==1
new=d1.strip().replace("The altitude dependence of the first stage can also be modeled in less detail.","The altitude dependence of the first stage can also be modeled in less detail.")
t=t.replace(old,new)
p.write_text(t,encoding="utf8",newline="")
