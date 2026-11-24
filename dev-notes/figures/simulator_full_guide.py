"""Second page of simulator_full_blocks.pdf: how to read the full block diagram.

Called by fig_simulator_dynamics_blocks.py after the full sheet is saved. The text flows
through three columns on a page the size of the diagram page, and the two PDFs are joined
with pypdf, so the diagram stays page 1 and this guide is page 2.
"""
import io
import os
import re

import matplotlib.pyplot as plt
from pypdf import PdfReader, PdfWriter

from figstyle import INK

BODY, H2, H1 = 12.5, 15.0, 19.0       # pt; nothing below the 10 pt thesis rule

# (kind, text): h1 title, h2 section, p paragraph, i numbered item heading, b bullet
GUIDE = [
    ("h1", "How to read the full simulator block diagram"),
    ("h2", "Layout"),
    ("p", "The page has three columns. Read them left to right, and each column top to bottom."),
    ("b", "Left: Optimizer, Phase schedules and insertion, Cost."),
    ("b", "Middle: Instant State Changes, Guidance, Defined variables."),
    ("b", "Right: Environment and aerodynamics, Engine and mass flow, Dynamics, Kinematics, "
          "State and termination, Legend."),
    ("h2", "One pass through the loop"),
    ("i", "1. Optimizer (top left)"),
    ("p", r"A particle swarm of $N_p$ particles runs for $N_g$ generations. Each particle is one "
          r"design vector $\mathbf{x}$, kept between the lower and upper bounds "
          r"$\mathbf{x}_{lb}$ and $\mathbf{x}_{ub}$. A Demux splits $\mathbf{x}$ into:"),
    ("b", r"$\gamma_p$: the flight-path angle the vehicle is kicked to after the vertical rise;"),
    ("b", r"$\Delta t_c$: the Stage 2 coast duration;"),
    ("b", r"$\Delta t_r\%$: the share of the maximum Stage 2 burn time $T_{\max,2}$ that is used;"),
    ("b", r"coast-start$\%$: the share of that burn time flown before the coast;"),
    ("b", r"$\mathbf{c}$: the constants of an open-loop law;"),
    ("b", r"$\boldsymbol{\lambda}_0$: the initial costates of the optimal-control law."),
    ("p", r"The optimizer keeps the best design and its cost, $\mathbf{x}^*$ and $J^*$."),
    ("i", "2. Phase schedules (left)"),
    ("p", r"Each schedule outputs $k$ (which guidance port is active), law (law1 in Stage 1, "
          r"law2 in Stage 2), $\delta$ (engine on = 1, off = 0) and $j$ (the stage, 1 or 2). "
          r"A Merge passes on the active schedule's outputs."),
    ("b", r"Stage 1: vertical rise until $t_v$, then a kick, then law1 steers. MECO (main "
          r"engine cutoff) comes when the Stage 1 propellant $m_{p1}$ is spent. $m$ is the "
          r"current mass."),
    ("b", r"Interstage: the vehicle coasts with $\theta = \gamma$. Separation comes "
          r"$\Delta t_{sep}$ after MECO, and Stage 2 ignition $\Delta t_{ign}$ after MECO."),
    ("b", r"Stage 2: burn 1 (duration $\Delta t_1$), a coast of $\Delta t_c$, then burn 2 "
          r"(duration $\Delta t_3$). The two burns together last "
          r"$\Delta t_r\% \cdot T_{\max,2}$, and law2 steers both."),
    ("i", "3. Instant State Changes (middle top)"),
    ("p", r"Two wires from the phase schedules trigger these jumps. The kick wire leaves the "
          r"Stage 1 schedule and fires at $t_v$, when the vertical rise ends. The sep wire "
          r"leaves the interstage schedule and fires at stage separation, $\Delta t_{sep}$ "
          r"after MECO. The superscript $^+$ marks the value just after a jump."),
    ("b", r"Start: $\gamma^+ = \gamma_0$ and $m^+ = m_0$, the launch flight-path angle and mass."),
    ("b", r"Kick: $\gamma^+ = \gamma_p$."),
    ("b", r"Fairing: $m^+ = m - m_{fair}$ (the fairing mass), when the altitude $h$ crosses "
          r"$h_{atm}$ (the atmosphere-exit altitude)."),
    ("b", r"Separation: $m^+ = m - m_{s1}$, the mass Stage 1 drops."),
    ("b", r"The $\gamma$ reset and $m$ reset tags tell the integrators when to take these values."),
    ("i", "4. Guidance (middle)"),
    ("p", r"The multiport switch, set by $k$, picks the pitch angle $\theta$ (the thrust "
          r"direction measured from the local horizontal) from one of five sources."),
    ("b", r"Vertical rise: $\theta = 90^\circ$."),
    ("b", r"Thrust along velocity: $\theta = \gamma$, where $\gamma$ is the flight-path angle."),
    ("b", r"Closed-loop law (Apollo, PEG). It uses the state $\mathbf{y}$, the thrust $F_T$, the "
          r"specific impulse $I_{sp}$ and the target $r_f,\ v_f,\ \gamma_f$ (radius, speed and "
          r"flight-path angle). On a clock it recomputes its time-to-go $t_{go}$ and its "
          r"coefficients."),
    ("b", r"Open-loop law: $\theta(t;\ \mathbf{c})$."),
    ("b", r"Optimal-control law: starts at Stage 2 ignition $t_{ign}$ from costates "
          r"$\boldsymbol{\lambda} = \boldsymbol{\lambda}_0$, and computes $\alpha$ from "
          r"$\boldsymbol{\lambda}$."),
    ("b", r"A summer outputs $\alpha = \theta - \gamma$, the angle from the velocity to the thrust."),
    ("i", "5. Environment and aerodynamics (right)"),
    ("b", r"Altitude: $h = r - R_E$, where $r$ is the geocentric radius and $R_E$ the Earth radius."),
    ("b", r"Density $\rho$ and ambient pressure $p_a$ are exponential in $h$. They use the "
          r"sea-level values $\rho_0$ and $p_0$ and the scale height $H$. $\kappa$ is the "
          r"atmosphere switch (1 = on, 0 = off)."),
    ("b", r"Dynamic pressure: $q = \rho v^2/2$, where $v$ is the speed."),
    ("b", r"Drag: $F_D = q\, C_D A$, with drag coefficient $C_D$ and reference area $A$."),
    ("b", r"Gravity: $g = \mu/r^2$, where $\mu$ is Earth's gravitational parameter."),
    ("i", "6. Engine and mass flow (right)"),
    ("b", r"Thrust: $F_T = \delta\,(F_{vac,j} - p_a A_{e,j})$. $F_{vac,j}$ is stage $j$'s vacuum "
          r"thrust and $A_{e,j}$ its nozzle exit area."),
    ("b", r"$I_{sp}$ follows from $p_a$, with vacuum value $I_{sp,vac,j}$."),
    ("b", r"Mass flow: $\dot m = -F_T/(I_{sp}\, g_0)$, with standard gravity $g_0$. "
          r"Integrating $\dot m$ gives $m$."),
    ("i", "7. Dynamics (right)"),
    ("b", r"$\dot v$ and $\dot\gamma$ come from $F_T,\ F_D,\ g,\ \alpha,\ m,\ v,\ \gamma$ and $r$."),
    ("b", r"The pseudo-forces add $\Delta\dot v$ and $\Delta\dot\gamma$, using "
          r"$\Omega = \varepsilon\,\omega\cos\varphi_0$. $\varepsilon$ is the rotation switch, "
          r"$\omega$ Earth's rotation rate and $\varphi_0$ the launch latitude."),
    ("b", r"Integrators (1/s) give $v$ and $\gamma$, starting from $v_0$ and $\gamma_0$."),
    ("i", "8. Kinematics (right)"),
    ("p", r"$\dot r$ and $\dot s$ are integrated into $r$ and the downrange distance $s$, "
          r"starting from $r_0$ and $s_0$."),
    ("i", "9. State and termination (right)"),
    ("b", r"A Mux builds $\mathbf{y} = [s,\ r,\ v,\ \gamma,\ m]$."),
    ("b", r"The Crash check raises the crash flag if $h < 0$, $v < 0$ or an integration step fails."),
    ("i", "10. Insertion (bottom of the left block)"),
    ("p", r"At the end of burn 2 ($\delta = 0$) it does four things, then the Stop block ends the run."),
    ("b", r"Inertial frame: $v_h$ is the inertial horizontal speed. $v_I$ and $\gamma_I$ are the "
          r"inertial speed and flight-path angle, and $\mathbf{y}_{f,I}$ is the final inertial state."),
    ("b", r"Final orbit: semi-major axis $a$, eccentricity $e$, apoapsis altitude $h_a$ and "
          r"periapsis altitude $h_p$."),
    ("b", r"Propellant left: $m_{prop} = m - m_{dry,2}$, where $m_{dry,2}$ is Stage 2's "
          r"structure plus payload."),
    ("b", r"Misses from the target: $\Delta r,\ \Delta v$ and $\Delta\gamma$."),
    ("i", "11. Cost (bottom left)"),
    ("p", r"$J$ combines four terms:"),
    ("b", r"Unused propellant, as the fraction $m_{prop}/m_{p2}$ of the Stage 2 propellant "
          r"load $m_{p2}$."),
    ("b", r"The misses, normalised by the target altitude $h_f$, the target speed $v_f$ and "
          r"the reference angle $\gamma_{ref}$."),
    ("b", r"The weights $w_J,\ w_r,\ w_v$ and $w_\gamma$ on each term."),
    ("b", r"A penalty $J_{crash}$, added if crash is set."),
    ("b", r"The $J$ wire runs round the left edge back into the optimizer."),
    ("h2", "Reading conventions"),
    ("b", r"Solid wires connect blocks directly, also across columns: kick, sep, law, $k$, "
          r"$\alpha$ and $J$."),
    ("b", "Tags (From/Goto) carry a named signal without a wire. A tag connects to every other "
          "tag with the same label."),
    ("b", "Clocks are time inputs. 1/s blocks are integrators, with their initial or reset "
          "values written underneath."),
    ("b", r"Defined variables (middle bottom) list every constant above, grouped as Earth, "
          r"atmosphere, vehicle, model switches, Ascent Case, launch state $\mathbf{y}_0$ and timing."),
    ("b", "Colours show each block's role (legend, bottom right): environment, vehicle, "
          "equations of motion, guidance, schedule and event logic, optimizer and cost."),
]


def _words(s):
    """Split on spaces outside $...$, so a formula is never broken across lines."""
    return [w for w in re.split(r" (?=(?:[^$]*\$[^$]*\$)*[^$]*$)", s) if w]


def _render(path, width, height, ncol=3):
    fig = plt.figure(figsize=(width, height))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.axis("off")
    R = fig.canvas.get_renderer()
    inv = ax.transData.inverted()

    def tw(s, size, weight):
        t = ax.text(0, 0, s, fontsize=size, fontweight=weight)
        bb = t.get_window_extent(R)
        t.remove()
        return inv.transform((bb.x1, 0))[0] - inv.transform((bb.x0, 0))[0]

    mx, top, bot, gap = 0.45, height - 0.45, 0.4, 0.5
    cw = (width - 2 * mx - (ncol - 1) * gap) / ncol
    col, y = 0, top
    style = {"h1": (H1, "bold", 0.0, 0.16), "h2": (H2, "bold", 0.0, 0.14),
             "i": (BODY, "bold", 0.0, 0.07), "p": (BODY, "normal", 0.0, 0.05),
             "b": (BODY, "normal", 0.22, 0.04)}
    for kind, text in GUIDE:
        size, weight, ind, after = style[kind]
        lh = size * 1.38 / 72
        avail = cw - ind
        lines, cur = [], ""
        for w in _words(text):
            trial = w if not cur else cur + " " + w
            if cur and tw(trial, size, weight) > avail:
                lines.append(cur)
                cur = w
            else:
                cur = trial
        lines.append(cur)
        pre = 0.14 if kind in ("h2", "i") and y < top else 0.0
        keep = 2 * BODY * 1.38 / 72 if kind in ("h2", "i") else 0.0   # heading keeps 2 lines
        if y - pre - lh * len(lines) - keep < bot and col < ncol - 1 and kind != "h1":
            col, y, pre = col + 1, top, 0.0
        y -= pre
        x0 = mx + col * (cw + gap)
        if kind == "b":
            ax.text(x0 + 0.06, y, "–", fontsize=size, va="top", color=INK)
        for ln in lines:
            ax.text(x0 + ind, y, ln, fontsize=size, fontweight=weight, va="top", color=INK)
            y -= lh
        y -= after
    if y < bot:
        print("  [guide] text runs past the bottom of the page")
    fig.savefig(path)
    plt.close(fig)


def append_guide(diagram_pdf):
    """Make the guide page 2 of diagram_pdf, at the diagram page's size."""
    with open(diagram_pdf, "rb") as f:
        page = PdfReader(io.BytesIO(f.read())).pages[0]
    w, h = float(page.mediabox.width) / 72, float(page.mediabox.height) / 72
    guide_pdf = os.path.splitext(diagram_pdf)[0] + "_guide.tmp.pdf"
    _render(guide_pdf, w, h)
    out = PdfWriter()
    out.add_page(page)
    out.add_page(PdfReader(guide_pdf).pages[0])
    with open(diagram_pdf, "wb") as f:
        out.write(f)
    os.remove(guide_pdf)
    print("added the guide as page 2 of " + diagram_pdf)
