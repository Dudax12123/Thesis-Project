p='overview.py'
s=open(p,encoding='utf8').read()
for a,b in [("what each architecture fixes comes after all four architectures","what each ascent case fixes comes after all four ascent cases"),
            ("but Architecture A lets","but Ascent Case A lets"),
            ("These support Architecture A&rsquo;s vector and Architecture C","These support Ascent Case A&rsquo;s vector and Ascent Case C"),
            ("edits to Architecture C&rsquo;s paragraph","edits to Ascent Case C&rsquo;s paragraph")]:
    assert a in s, a
    s=s.replace(a,b)
open(p,'w',encoding='utf8').write(s)
