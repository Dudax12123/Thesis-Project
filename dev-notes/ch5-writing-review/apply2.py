"""HW-01 / PROG-02 / REP-01 / REC-03 as decided 2026-11-22: Chapter 5 keeps the constants of all eight laws,
the bounds go to Chapter 6 (Table pso_settings + a paragraph)."""
import sys, pathlib
R = pathlib.Path(r"C:\Users\eduar\Desktop\Tese\Thesis_Overleaf")
imp = (R / "Thesis_Implementation.tex").read_text(encoding="utf8")
res = (R / "Thesis_Results.tex").read_text(encoding="utf8")


def cut(t, a, b):
    i = t.index(a)
    assert t.count(a) == 1, a[:60]
    j = t.index(b, i) + len(b)
    return t[i:j]


def sub(t, old, new):
    assert t.count(old) == 1, old[:70]
    return t.replace(old, new, 1)


# ---- Chapter 5
p_old = cut(imp, "An open-loop law prescribes the pitch as a function of time, and the constants", "with their bounds.")
p_new = (r"Table~\ref{tab:law_constants} lists, for each guidance law, the constants that the optimizer appends to the design vector $\mathbf{x}$; "
         r"their bounds are given with the optimizer settings (Section~\ref{ssec:optimizer_settings}). "
         r"An open-loop law prescribes the pitch as a function of time but not the constants of that function; a closed-loop law determines its own and appends none.")
imp = sub(imp, p_old, p_new)

t_old = cut(imp, "\\begin{table}[htbp]\n  \\centering\n  \\caption{Constants appended to the design vector", "\\end{table}")
t_new = r"""\begin{table}[htbp]
  \centering
  \caption{Constants appended to the design vector $\mathbf{x}$ by each guidance law.}
  \label{tab:law_constants}
  \begin{tabular}{l l}
    \toprule
    \textbf{Law} & \textbf{Constants} \\
    \midrule
    Gravity turn              & none \\
    Constant pitch rate       & $\dot\theta$ \\
    Linear tangent            & $\theta_i,\ \theta_f$ \\
    Bilinear tangent          & $\theta_i,\ \theta_f,\ \eta$ \\
    Exponential pitch         & $a,\ b$ \\
    Apollo guidance           & none \\
    Powered explicit guidance & none \\
    Optimal-control law       & $\lambda_{0r},\ \lambda_{0v},\ \lambda_{0\gamma}$ \\
    \bottomrule
  \end{tabular}
\end{table}"""
imp = sub(imp, t_old, t_new)

b_old = cut(imp, "The bound on $a$ keeps the initial commanded pitch", "of each burn arc.")
b_new = (r"The time of the exponential law restarts at the beginning of each burn, so that the pitch returns to $a$ at the start of the final burn; "
         r"the tangent laws instead run continuously through the coast (Section~\ref{ssec:tangent}).")
imp = sub(imp, b_old, b_new)

# ---- Chapter 6
r_old = "Initial costates $\\lambda_0$\n      & --- & --- & $[-1,1]$ & --- \\\\\n    \\midrule"
r_new = (r"""Initial costates $\lambda_0$
      & --- & --- & $[-1,1]$ & --- \\
    \midrule
    \multicolumn{5}{l}{\emph{Law-constant bounds (open-loop laws)}}\\
    Pitch rate $\dot\theta$ [\unit{\degree\per\s}]
      & $[0.02,1.0]$ & $[0.02,1.0]$ & --- & $[0.02,1.0]$ \\
    Tangent pitch $\theta_i$ [\unit{\degree}]
      & $[-20,70]$ & $[-20,70]$ & --- & $[-20,70]$ \\
    Tangent pitch $\theta_f$ [\unit{\degree}]
      & $[-40,30]$ & $[-40,30]$ & --- & $[-40,30]$ \\
    Bilinear fraction $\eta$
      & $[0.1,0.9]$ & $[0.1,0.9]$ & --- & $[0.1,0.9]$ \\
    Exponential $a$ [rad]
      & $[0,1.6]$ & $[0,1.6]$ & --- & $[0,1.6]$ \\
    Exponential $b$ [\unit[per-mode=power]{\per\s}]
      & $[-0.05,0.005]$ & $[-0.05,0.005]$ & --- & $[-0.05,0.005]$ \\
    \midrule""")
res = sub(res, r_old, r_new)
after_old = "\\end{table}\n\n\n\\subsection{Reported Quantities}"
after_new = ("\\end{table}\n\n"
             r"The bound on $a$ keeps the initial commanded pitch in the physically meaningful quadrant, and the one on $b$ admits the decaying programs the exponential law is intended to produce while leaving a narrow margin for a slowly rising one."
             "\n\n\n\\subsection{Reported Quantities}")
res = sub(res, after_old, after_new)

if "--write" in sys.argv:
    (R / "Thesis_Implementation.tex").write_text(imp, encoding="utf8", newline="")
    (R / "Thesis_Results.tex").write_text(res, encoding="utf8", newline="")
    print("written")
else:
    print("dry run ok")
