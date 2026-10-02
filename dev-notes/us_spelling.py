"""Convert UK spellings to US in the prose of one .tex file.

Only the non-comment part of each line is touched, and the arguments of
\\label, \\ref, \\eqref, \\cite, \\texttt, \\input, \\includegraphics, \\resultstable,
\\url and \\href are masked so no key or path changes. Usage:
    python us_spelling.py FILE [--apply]
"""
import re
import sys
from pathlib import Path

STEM_MAP = {  # UK stem -> US stem, applied to whole words that start with the stem
    "optimis": "optimiz", "minimis": "minimiz", "maximis": "maximiz", "normalis": "normaliz",
    "initialis": "initializ", "specialis": "specializ", "penalis": "penaliz", "realis": "realiz",
    "parametris": "parametriz", "discretis": "discretiz", "stabilis": "stabiliz",
    "linearis": "lineariz", "summaris": "summariz", "generalis": "generaliz", "organis": "organiz",
    "recognis": "recogniz", "utilis": "utiliz", "regularis": "regulariz",
}
WORD_MAP = {
    "neighbourhood": "neighborhood", "neighbourhoods": "neighborhoods", "neighbouring": "neighboring",
    "behaviour": "behavior", "behaviours": "behaviors",
    "manoeuvre": "maneuver", "manoeuvres": "maneuvers",
    "labelled": "labeled", "labelling": "labeling", "modelled": "modeled", "modelling": "modeling",
    "colour": "color", "colours": "colors", "centre": "center", "centres": "centers",
}
MASK = re.compile(r"\\(label|ref|eqref|cite|texttt|input|includegraphics|resultstable|url|href)"
                  r"(\[[^\]]*\])?\{[^}]*\}")
WORD = re.compile(r"[A-Za-z]+")
# words that merely contain a stem (e.g. "realistic", "characteristic") must not change
STEM_TAILS = re.compile(r"^(e|ed|es|er|ers|ing|ation|ations|able)$")


def convert_word(w):
    lw = w.lower()
    new = None
    if lw in WORD_MAP:
        new = WORD_MAP[lw]
    else:
        for uk, us in STEM_MAP.items():
            if lw.startswith(uk) and STEM_TAILS.match(lw[len(uk):]):
                new = us + lw[len(uk):]
                break
    if new is None:
        return w
    if w.isupper():
        return new.upper()
    if w[0].isupper():
        return new[0].upper() + new[1:]
    return new


def convert_prose(seg, changes):
    # mask protected spans
    spans = [(m.start(), m.end()) for m in MASK.finditer(seg)]
    out, pos = [], 0
    for a, b in spans + [(len(seg), len(seg))]:
        chunk = seg[pos:a]

        def sub(m):
            new = convert_word(m.group(0))
            if new != m.group(0):
                changes.append((m.group(0), new))
            return new
        out.append(WORD.sub(sub, chunk))
        out.append(seg[a:b])
        pos = b
    return "".join(out)


def main():
    path = Path(sys.argv[1])
    apply = "--apply" in sys.argv
    raw = path.read_bytes()
    assert b"\r" not in raw, "file has CR; expected LF"
    lines = raw.decode("utf-8").split("\n")
    total = []
    for i, line in enumerate(lines):
        m = re.search(r"(?<!\\)%", line)
        code, comment = (line[:m.start()], line[m.start():]) if m else (line, "")
        changes = []
        new_code = convert_prose(code, changes)
        if changes:
            total.extend((i + 1, a, b) for a, b in changes)
            lines[i] = new_code + comment
    for ln, a, b in total:
        print(f"l.{ln}: {a} -> {b}")
    print(f"{len(total)} replacements")
    if apply:
        path.write_bytes("\n".join(lines).encode("utf-8"))
        print("written")


main()
