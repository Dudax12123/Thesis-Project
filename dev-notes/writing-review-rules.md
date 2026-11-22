# Writing review rules (thesis)

Rules used for the Chapter 1–4 writing review of 2026-11-13. That review is published as the
"Writing Review Ch 1–4" page: https://claude.ai/artifact/LwJSN79eNsF1J7PMo6DDRc.
Feed this file to a session that reviews another chapter (e.g. Chapter 5, `Thesis_Implementation.tex`).

## 0. Setup and standing rules

- **Where the thesis lives.** The thesis is a separate repo, `C:\Users\eduar\Desktop\Tese\Thesis_Overleaf`
  (Overleaf git bridge). Add it to the session if it is not there.
- **Fetch first.** Run `git fetch` and read the incoming diff before reviewing or editing.
- **Never handle the Overleaf token.** Mask remote URLs in any output with `sed -E 's#//[^@]*@#//***@#'`.
- **No LaTeX toolchain.** Verify with `\ref` / `\label` / `\cite` sweeps, not a build.
- **Review first, edit later.** Implement nothing in the thesis until the user answers the review page.
  - An Implement-button press means: apply that one card, commit and push in the same turn.
- **Paths.** Use absolute paths only; never `cd` into the thesis repo.
- **Chapter roles.**
  - Ch 2 is theory only.
  - **Ch 5 is how AND why** (implementation choices with their reasons).
  - Ch 6 is results only and points back to Ch 5.
  - Implementation detail found in Ch 2–4 belongs in Ch 5; results found in Ch 5 belong in Ch 6.
- **Paragraphs** are small to medium, about 5 lines at most.
- **Units** always through siunitx: `\qty`, `\qtyrange`, `\ang`, `\unit`.
- **No model history.** Never write "new/old model", "re-flown", "re-run", "previously". The current
  implementation is simply the implementation.
- **Register.** Concise, graduate-level English. No metaphors ("three doors"). No colon-reveal
  sentences ("One assumption is made, and it is deliberate: …").

## 1. The coordinator's writing guide (11 principles)

Source: "Clareza na escrita científica", the Kepler-equation example (`escrita_cientifica_mecanica_orbital.pdf`).
The guide's lesson is that a bad text is usually not wrong; it makes the reader work. Fix the
**order** before the style.

| # | Aspect (Tabela 1) | Rule | What to look for |
|---|---|---|---|
| 1 | Orientação inicial | Orient the reader before the details: what the subject is and why it is developed. One concrete sentence is enough; no generic introduction. | Sections opening with generic text ("This section presents the adopted modeling approach…"); details before purpose. |
| 2 | Proposição adiada | State the main proposition early (theorem, then proof). | Decisions given after their reasons; the model used shown after a model that is abandoned; an approach assembled across several paragraphs. |
| 3 | Associação das hipóteses | Keep related information close. Distance matters, not sentence length. | An assumption stated far from the equation it conditions; an omission hidden in a subordinate clause. |
| 4 | Coerência terminológica | One stable name per concept, method, quantity and symbol. Never vary for style. | Synonyms (kick / pitch-over / pitch program); two symbols for one quantity (T vs F_T, I_sp0 vs I_sp,vac, ω vs χ); two notations for the same time variables. |
| 5 | Definição dos conceitos | Define before use, including how a quantity is measured. | Terms used before their definition ("architecture", "design vector"); undefined symbols; weights or thresholds given without their meaning. |
| 6 | Recuperação de informação | Recall the detail when it is retrieved from far away. | "the choice referred to earlier", "the shortfall", "it" pointing two paragraphs back, **"above"/"below"** (the coordinator said never: name the section). Forward references where a recall would do. |
| 7 | Ordem / progressão do argumento | Progress argumentatively: purpose → equation → conditions → results → interpretation → consequence. Each passage prepares the next. | Definitions never applied afterwards; a section placed after the sections that use it; laws or variants that do not end with the same information (e.g. their free constants). |
| 8 | Ritmo | Vary rhythm by sentence function. Avoid repeated passive cadences. Never vary terminology for rhythm. | Strings of "is computed… is set… is used…"; the same citation closing every sentence of a paragraph. |
| 9 | Âmbito das conclusões | Match the strength of claims to the evidence. | Universal claims from one or two citations; "guarantees", "the most accurate", "defeats", "requires"; claims of verification with no location; conditions of validity left out. |
| 10 | Fecho do assunto | Close a subject by saying its role in the work. Chapters need a coda (the coordinator asked for it). Not every paragraph needs a closing sentence. | A chapter ending on a technical detail; a literature review ending on a pointer instead of the gap. |
| 11 | Concisão sem perda de precisão | Concision without losing precision: "the Newton–Raphson method", not the bare name. | Wordiness; bare names that lose what kind of thing they are; informal phrasing ("starts becoming too expensive"). |

## 2. The checks (one tab each on the review page)

1. **Organization.** For every chapter, section, subsection and subsubsection:
   - Is it justified, and is it introduced?
   - Is it in the place the reader needs it (defined before use)?
   - Is it closed?
   - Does the title match the content? (e.g. "Forces and Moments" that treats no moments)
   - Is any division level lone (a single subsection)?
   - Does the chapter introduction's roadmap match the actual headings?
   - Is each section in the right chapter (chapter roles)?
   - Does any section's content sit in the wrong section (an event inside a forces section)?
   - Is something announced but missing (an eighth law with no subsection; a commented-out block)?
2. **Orientation first** (principles 1–2).
3. **Terms and definitions** (principles 4–5). Keep a running table of every concept and symbol
   with all the names it takes in the chapter, and check them against Ch 1–4. Ch 5 must use the
   names Ch 1–4 now use.
4. **Distance and recall** (principles 3, 6).
5. **Progression and closings** (principles 7, 10). Check the chapter coda and the bridge to the next chapter.
6. **Strength of claims** (principle 9).
7. **Concision and mechanics** (principles 8, 11).
   - `Chapter~\ref{<section label>}` prints "Chapter 5.2". Use `Section~` for section labels.
   - Missing spaces after commas (`conditions,Eq.`).
   - Commas between subject and verb.
   - Mixed curly and straight apostrophes.
   - Stale file headers and comments, empty comment blocks, "VERIFY" notes.
8. **Missing citations.** Statements of fact, history, figures or literature that carry no source.
   - Suggest only sources already in `Thesis_Bibliography_DB.bib` or ones you have actually opened.
   - **Never invent** a source or a page number. If none is available, ask the user.
9. **Wrong citations.** Check that each source supports its sentence.
   - Open the book or PDF where possible (`C:\Users\eduar\Desktop\Tese\References\`, books in `Livros\`).
   - Extract text with `/mingw64/bin/pdftotext`; pypdf/fitz are not installed.
   - Typical faults:
     - a source unrelated to the claim;
     - a source predating what it is cited for;
     - a general survey or textbook cited for this thesis's specific equations;
     - figures that do not match the source;
     - a claim wider than the source (three launchers cited to a paper on one).
   - Mark each one "Wrong", "Probably wrong", "Partly unsupported" or "Not verified". State what was
     actually found in the source.
10. **Bibliography entries.** Look for:
    - duplicates (the same paper under two keys);
    - malformed fields (authors inside the title, the foreword writer as author, bad years);
    - the wrong entry type (`@CONFERENCE` with a `journal` field);
    - a missing venue;
    - an edition that differs from the PDF actually used.

## 3. Mechanical sweeps to run

- **Cite and label sweep** on the active text, with `\begin{comment}` blocks and `%` lines stripped:
  - every `\cite` key exists in the bib;
  - every `\ref`/`\eqref` label exists in a file that is `\input` by `Thesis.tex`.
  - Results and Conclusions are currently commented out in `Thesis.tex`, so their labels print "??".
- **Cite counts per key.** A key cited in every sentence of a paragraph should appear once.
- **Proposed edits:**
  - every `old` string occurs **exactly once** in the current file;
  - every key and label in the `new` text exists.
  - Keep the cards in a `content.py` of `(file, old, new)` tuples written as **raw strings in a
    file**; never put LaTeX-bearing Python in a bash heredoc.

## 4. Card format (review page)

Each card has:
- an id;
- the location (file, line, section);
- a size: quick, moderate or structural;
- the principle tags;
- a title stating the problem;
- **What it refers to**, quoting the current text;
- **Reading**, saying why it is a problem and which principle;
- **Recommendation**, the concrete fix;
- **Proposed change in context**, as a word diff with additions in green and removals in red, showing
  the whole affected passage;
- an editable box with the proposed LaTeX;
- **Where the change is made** and **What else is affected**, including other chapters;
- Approve / Change / Reject buttons, a comment field and an Implement button.

Page layout and saving:
- The first page is the Overview. It holds:
  - the principles;
  - the key take-aways for this chapter;
  - what to decide first;
  - questions for the user;
  - what was checked in the sources;
  - a files table.
- Answers save to the artifact db, collection `reviews`, one document per card id.
- Reuse the JS of the Writing Review Ch 1–4 page with a new localStorage key.

## 5. Recurring faults found in Ch 1–4 (check that Ch 5 does not repeat or contradict them)

**Names and symbols to keep consistent with the Ch 1–4 decisions** (subject to what the user approved
on that page):
- pitch-over angle γ_p, not "kick angle";
- the arc durations Δt₁, Δt_c, Δt₃ and the objective J = Δt₁ + Δt₃;
- the thrust symbol F_T;
- I_sp,vac;
- the PSO factor χ (constriction, Clerc–Kennedy), not ω;
- "endo-/exo-atmospheric arc";
- "vertical plane", not "2D"/"planar"/"3DOF" mixed;
- "Apollo guidance" and the named PEG variant.

**"Architecture" and "design vector"** are defined in Ch 5. Their definition must be explicit and
early, because Ch 1 and Ch 4 point to it.

**Faults that also appeared in Ch 1–4:**
- Claims of verification ("verified by finite differences") with no pointer to where it is shown.
- Weights and thresholds given without their meaning or their source. In Ch 2 this was the 1000 Pa
  fairing threshold, which has no source.
- Laws not ending with their free constants. Ch 5 should state, per law, what the optimizer chooses.
- Implementation options mentioned in theory chapters ("the simulator also offers…"). These belong in
  Ch 5 or Appendix A.
- Typical-value statements that must match the cited figures. Edberg gives drag 40–200 m/s, steering
  30–400 m/s and a design speed of about 9.3 km/s.
- Refinement: per the user rule of 2026-10-30, only the PMP references are refined. Ask the user before
  describing refinement of any law; an optional open-loop `DIRECT_REFINE` exists, off by default.
