p='content.py'
s=open(p,encoding='utf8').read()
a=s.index('_lines = []\nfor a, b in _clusters:')
b=s.index('TERM01_AFFECTED.extend([')
new='''_lines = []
_wins = []
for a, b in _clusters:
    k = 30
    s0, e0 = max(0, a - k), min(len(_T), b + k)
    for x, y in _spans:                      # stay clear of the other cards' passages
        if x < e0 <= y or (x >= b and x < e0):
            e0 = x
        if x < s0 < y or (y <= a and y > s0):
            s0 = y
    if _wins and s0 < _wins[-1][1]:
        _wins[-1][1] = max(_wins[-1][1], e0)
    else:
        _wins.append([s0, e0, a])
for s0, e0, a in _wins:
    old = _T[s0:e0]
    assert _T.count(old) == 1, old
    _term01["edits"].append(("IMPL", old, rn(old)))
    _lines.append(_TL(a))
'''
s=s[:a]+new+s[b:]
open(p,'w',encoding='utf8').write(s)
