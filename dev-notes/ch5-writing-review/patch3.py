p='content.py'
s=open(p,encoding='utf8').read()
s=s.replace('''    assert _T.count(old) == 1, old
''','''    while _T.count(old) != 1:
        s0 = max(0, s0 - 15)
        for x, y in _spans:
            if x < s0 < y:
                s0 = y
        old = _T[s0:e0]
''')
open(p,'w',encoding='utf8').write(s)
