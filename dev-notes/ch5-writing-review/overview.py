QUESTIONS_NOTE = ""

OVERVIEW = """<section class="ov"><h2>What this page is</h2>
<p>A review of Chapter 5 (Implementation, <code>Thesis_Implementation.tex</code>) against the same writing guide as the Chapter 1 to 4 page, with the names and symbols you approved there. Each tab is one kind of check, with one card per problem: what the text says, why it is a problem, what I recommend, and the proposed LaTeX as tracked changes. Chapter 5 is the <em>how and why</em> chapter, so two tabs are specific to it: <b>How and why</b> (every choice with its reason; nothing that belongs in Chapter 6) and <b>Draft notes</b> (the five red passages).</p>
<p><b>Nothing has been edited in the thesis.</b> {N} cards, {NEDIT} proposed edits, all checked to occur exactly once in the current Overleaf source (<code>main</code> at b75f495), with no two edits overlapping. Answer each card with Approve, Change or Reject; answers save as you type. The Implement button sends that one card to me. Several cards are questions without LaTeX, because the answer is yours (a reason, a source, a decision); they say so.</p>

<h2>The guide, in short</h2>
<p>The guide teaches that a bad text is usually not wrong, it makes the reader work, so the order is fixed before the style. Its eleven aspects, which each card cites by number:</p>
<ol class="principles">
<li><b>Orient the reader first.</b> <span class="ptag">Orienta&ccedil;&atilde;o inicial</span></li>
<li><b>State the main proposition early.</b> <span class="ptag">Proposi&ccedil;&atilde;o adiada</span></li>
<li><b>Keep related information close.</b> <span class="ptag">Associa&ccedil;&atilde;o das hip&oacute;teses</span></li>
<li><b>One stable name per concept.</b> <span class="ptag">Coer&ecirc;ncia terminol&oacute;gica</span></li>
<li><b>Define before use, and say how it is measured.</b> <span class="ptag">Defini&ccedil;&atilde;o dos conceitos</span></li>
<li><b>Recall details retrieved from far away.</b> <span class="ptag">Recupera&ccedil;&atilde;o de informa&ccedil;&atilde;o</span></li>
<li><b>Progress argumentatively.</b> <span class="ptag">Ordem / progress&atilde;o do argumento</span></li>
<li><b>Vary rhythm by function, not terminology.</b> <span class="ptag">Ritmo</span></li>
<li><b>Match the strength of claims to the evidence.</b> <span class="ptag">&Acirc;mbito das conclus&otilde;es</span></li>
<li><b>Close a subject by its role.</b> <span class="ptag">Fecho do assunto</span></li>
<li><b>Be concise without losing precision.</b> <span class="ptag">Concis&atilde;o sem perda de precis&atilde;o</span></li>
</ol>

<h2>Key take-aways for Chapter 5</h2>
<ul class="notes">
<li><b>The chapter is sound in content but the order of Section 5.3 hides its main proposition.</b> The section runs ninety lines, three tables and three figures before its first subsection, and the figure that shows what each ascent case fixes comes after all four ascent cases (<a href="#ORG-02" data-goto="ORG-02">ORG-02</a>, <a href="#ORG-03" data-goto="ORG-03">ORG-03</a>). The chapter opening never says what the chapter does (<a href="#ORI-01" data-goto="ORI-01">ORI-01</a>).</li>
<li><b>&ldquo;Ascent case&rdquo; (your name for the four) and &ldquo;design vector&rdquo; are the terms Chapters 1, 3 and 4 point to, and their definitions are two passing sentences</b> (<a href="#ORI-03" data-goto="ORI-03">ORI-03</a>). Chapter 5 still says &ldquo;architecture&rdquo; in about 30 places and a bare &ldquo;Case&rdquo; in others; <a href="#TERM-01" data-goto="TERM-01">TERM-01</a> renames them all, and every other card is already written with the new name.</li>
<li><b>The design vector uses symbols the rest of the thesis does not.</b> Chapter 5 searches &Delta;t<sub>r</sub>% and a coast-start %, while Chapters 3 and 4 use &Delta;t<sub>1</sub>, &Delta;t<sub>c</sub>, &Delta;t<sub>3</sub> and J = &Delta;t<sub>1</sub> + &Delta;t<sub>3</sub>. I read the link from the code and state it in the table (<a href="#TERM-02" data-goto="TERM-02">TERM-02</a>).</li>
<li><b>Several choices have no reason, and the bounds live in Chapter 6.</b> The bounds of the design variables, the fixed event times, the instantaneous kick, the hand-off by altitude and the weight w<sub>&gamma;</sub> = 3 are stated without why, or the why is in a commented line. Morgado et al. give ranges I could use as a source (<a href="#HW-01" data-goto="HW-01">HW-01</a> to <a href="#HW-04" data-goto="HW-04">HW-04</a>, <a href="#RED-5" data-goto="RED-5">RED-5</a>).</li>
<li><b>No result sits in Chapter 5 that belongs in Chapter 6.</b> The only candidates are two measured misses in a code comment behind the w<sub>&gamma;</sub> choice (190 and 502 km); I kept them out of the proposed text (<a href="#RED-5" data-goto="RED-5">RED-5</a>).</li>
<li><b>Repetition is moderate:</b> the Limitations list repeats the tables, what a law adds to the design vector is said three times, and the direct-insertion argument twice (<a href="#ORG-04" data-goto="ORG-04">ORG-04</a>, <a href="#REP-01" data-goto="REP-01">REP-01</a>, <a href="#REP-02" data-goto="REP-02">REP-02</a>, <a href="#REP-03" data-goto="REP-03">REP-03</a>).</li>
<li><b>One contradiction with Chapter 4:</b> Chapter 4 says the first stage is steered only through &gamma;<sub>p</sub>, but Ascent Case A lets a law steer it (<a href="#COH-01" data-goto="COH-01">COH-01</a>).</li>
<li><b>Citations are mostly sound.</b> The one concrete fault is the Morgado bibliography entry (a wrong URL and no volume), and &ldquo;the standard way&rdquo; for the gravity turn goes beyond what Edberg and Walter say (<a href="#BIB-01" data-goto="BIB-01">BIB-01</a>, <a href="#REC-02" data-goto="REC-02">REC-02</a>).</li>
</ul>

<h2>Sections</h2><div class="tablewrap"><table class="mini"><tr><th>Tab</th><th>Cards</th><th>What is checked</th></tr>{ROWS}</table></div>

<h2>What I would decide first</h2>
<ol class="order">
<li><a href="#TERM-04" data-goto="TERM-04">TERM-04</a> and <a href="#TERM-05" data-goto="TERM-05">TERM-05</a>: the two remaining naming choices (free or open, name of the optimal-control law). The name of the four, &ldquo;Ascent Case&rdquo;, is settled and applied by <a href="#TERM-01" data-goto="TERM-01">TERM-01</a>.</li>
<li><a href="#ORG-02" data-goto="ORG-02">ORG-02</a>, <a href="#ORG-03" data-goto="ORG-03">ORG-03</a> and <a href="#REP-03" data-goto="REP-03">REP-03</a>: the structure of Section 5.3 (two new subsections, the summary moved to the front, whether to keep the four per-architecture figures).</li>
<li><a href="#HW-01" data-goto="HW-01">HW-01</a> and <a href="#HW-02" data-goto="HW-02">HW-02</a>: the bounds and fixed values need reasons that only you can give.</li>
<li><a href="#PROG-04" data-goto="PROG-04">PROG-04</a>: whether Chapter 5 mentions the refinement of the references.</li>
</ol>

<h2>Questions for you</h2>
<ul class="notes">
<li><b>Rename &ldquo;architecture&rdquo; in the other chapters too?</b> It appears about 83 times outside Chapter 5 (Abstract 3, Introduction 4, Chapter 3 9, Chapter 4 1, Chapter 6 39, Conclusions 7, Appendix A 20), and in the Resumo and Extended Abstract. I recommend renaming it everywhere in one pass after this page is answered.</li>
<li><b>Chapter 6 uses &ldquo;case&rdquo; for a flown run</b> (&ldquo;seventeen cases&rdquo;, the results matrix). With &ldquo;ascent case&rdquo; now meaning A to D, I recommend calling the runs &ldquo;runs&rdquo; there; confirm, or give the word you prefer.</li>
<li><b>Free or open?</b> I recommend &ldquo;free&rdquo;, which Chapters 3 and 4 use (<a href="#TERM-04" data-goto="TERM-04">TERM-04</a>).</li>
<li><b>Name of the seventh law.</b> Chapter 5 uses three forms; I recommend &ldquo;optimal-control law&rdquo; (<a href="#TERM-05" data-goto="TERM-05">TERM-05</a>).</li>
<li><b>Reasons for three bounds that differ from Morgado et al.</b> The coast duration is searched up to 2000 s (they use 3000 s), &gamma;<sub>p</sub> down to 1.50 rad (they use 1.54 rad) and the burn fraction from 0 % (they use 70 %). I propose no reason (<a href="#HW-01" data-goto="HW-01">HW-01</a>).</li>
<li><b>Sources or reasons for fixed values:</b> 7.5 s vertical rise, 3 s separation, 8 s ignition delay, the 280 km orbit, the 500-point and 21-point grids (<a href="#HW-02" data-goto="HW-02">HW-02</a>), and the reason for the instantaneous kick, which I read from the Appendix (<a href="#HW-03" data-goto="HW-03">HW-03</a>).</li>
<li><b>Does Chapter 5 mention the refinement of the PMP references?</b> Your rule of 30 October limits refinement to them, so I removed &ldquo;followed by a local refinement&rdquo; from the general sentence about the swarm (<a href="#MECH-01" data-goto="MECH-01">MECH-01</a>) and ask here (<a href="#PROG-04" data-goto="PROG-04">PROG-04</a>).</li>
<li><b>Can Architecture A fly the optimal-control law as its second law?</b> The Limitations list allows it, the waterfall figure shows no costates for A (<a href="#CLAIM-03" data-goto="CLAIM-03">CLAIM-03</a>).</li>
<li><b>Are the gravity turn and the constant pitch rate really flown in the second stage in the results?</b> Chapter 4 places them in the atmospheric arc (<a href="#REP-04" data-goto="REP-04">REP-04</a>).</li>
<li><b>Source for the empirical atmosphere model and the wind options</b> (<a href="#MCIT-01" data-goto="MCIT-01">MCIT-01</a>). Without one I propose the cut.</li>
<li><b>Results is commented out in Thesis.tex.</b> The proposed text refers to Chapter 6 by label, which prints &ldquo;??&rdquo; until it returns; I assume that is deliberate.</li>
<li><b>Outside this chapter</b> (no card edits them): Chapter 6 says &ldquo;all nine&rdquo; laws and &ldquo;seventeen cases&rdquo;, Appendix A says &ldquo;nine laws&rdquo;, and Chapter 3 writes the burn time as t<sub>f</sub> &minus; t<sub>cf</sub> (<a href="#COH-02" data-goto="COH-02">COH-02</a>, <a href="#TERM-02" data-goto="TERM-02">TERM-02</a>). Tell me if you want cards for them.</li>
</ul>

<h2>What I checked in the sources</h2>
<ul class="notes">
<li><b>Morgado et al. (2022), opened:</b> the design vector and the PyGMO swarm (coast duration, coast start, last-stage duration, pitch-over angle, three costates), the search ranges of their Table 9, and the costate range [&minus;1, 1] that the thesis adopts. These support Ascent Case A&rsquo;s vector and Ascent Case C. The two-law schedule of A is not theirs.</li>
<li><b>Edberg and Walter, opened:</b> both call the gravity turn a common pitch program, not the standard one; Walter adds that an optimized ascent is close to it. Edberg says low Earth orbits may be reached by a direct ascent, which supports one unsourced clause of Architecture D.</li>
<li><b>Not verified:</b> Pontani (2014), Teofilatto (damaged text), Vallado, Kluever. The cards that depend on them say so.</li>
<li><b>Code:</b> the bounds of the law constants, the burn-variable mapping (&Delta;t<sub>1</sub> + &Delta;t<sub>3</sub> = &Delta;t<sub>r</sub>%/100 &middot; T<sub>max,2</sub>), the event times and the w<sub>&gamma;</sub> comment were read in <code>simulation_parameters.py</code>, <code>rocket_specs.py</code> and <code>pso_coast_solver.py</code>.</li>
<li><b>Mechanical sweep:</b> every <code>\\cite</code> key and every <code>\\ref</code> label of Chapter 5, with comments stripped, exists; every key and label in the proposed text exists; the label <code>ssec:integration</code> is referenced nowhere.</li>
</ul>

<h2>Cards that depend on each other</h2><p>No two edits touch the same text: I checked every pair of edits by position in the source, and none overlap. Ten pairs still depend on each other in meaning, and each is shown on both cards. Sentences that sit side by side (for example the three edits to Ascent Case C&rsquo;s paragraph) are separate sentences and can be applied in any order.</p><ul class="notes">{DEPS}</ul>

<h2>Which files change</h2><div class="tablewrap"><table class="mini"><tr><th>File</th><th>Cards</th></tr>{FILES}</table></div>
</section>"""
