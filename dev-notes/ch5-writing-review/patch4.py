p='content.py'
s=open(p,encoding='utf8').read()
a=s.index('for s0, e0, a in _wins:\n    old = _T[s0:e0]')
b=s.index('TERM01_AFFECTED.extend([')
new='''_fin = []
for s0, e0, a in _wins:
    old = _T[s0:e0]
    while _T.count(old) != 1:
        s0 = max(0, s0 - 15)
        for x, y in _spans:
            if x < s0 < y:
                s0 = y
        old = _T[s0:e0]
    if _fin and s0 < _fin[-1][1]:
        _fin[-1][1] = max(_fin[-1][1], e0)
    else:
        _fin.append([s0, e0, a])
for s0, e0, a in _fin:
    old = _T[s0:e0]
    assert _T.count(old) == 1
    _term01["edits"].append(("IMPL", old, rn(old)))
    _lines.append(_TL(a))
'''
s=s[:a]+new+s[b:]
open(p,'w',encoding='utf8').write(s)
