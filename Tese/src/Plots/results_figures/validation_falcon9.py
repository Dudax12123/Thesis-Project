"""Validation figure -- the simulator's ascent against Falcon 9 flight data.

Three expendable Falcon 9 flights (Reference_Data/falcon9_webcast, telemetry read
off the SpaceX webcasts) against the reference and two laws of the results
matrix, from liftoff to the end of the first Stage-2 burn. No flight shares the
simulator's configuration -- these carried 5-7 t to a transfer orbit, throttled
through max-Q, and their Stage-1 propellant load is not published -- so the
comparison is of ranges and shape, not a reproduction.

The webcast shows speed and altitude only; every other flight channel is derived
from those two by the dataset's author. Dynamic pressure is therefore recomputed
here, for flights and cases alike, with the simulator's own atmosphere: a gap in
panel (d) is then a gap in speed and altitude, not in the atmosphere model.

The section's two tables (tables.validation_timeline and
tables.validation_states) read the same flights and cases through
summary_rows, and the timeline adds the sample LEO mission of the SpaceX user's
guide, GUIDE_LEO. make_all draws the figure; run on its own, this module also
prints the table values::

    python Plots/results_figures/validation_falcon9.py --root <results dir>

Outputs
-------
results_validation_flights.png   fig:validation_flights
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_SRC = Path(__file__).resolve().parent.parent.parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from Plots import plot_state_utils as psu
from Plots.results_figures import _data
from Plots.results_figures import _panels as pn
from Plots.results_figures import _style as st

DATA_DIR = _SRC / "Reference_Data" / "falcon9_webcast"

FLIGHTS = [("intelsat_35e", "Intelsat 35e", "-"),
           ("inmarsat_5_f4", "Inmarsat-5 F4", "--"),
           ("echostar_23", "EchoStar 23", ":")]

SIM_CASES = [("pmp_baseline", st.REFERENCE),
             ("gt_baseline", st.BASELINE),
             ("peg_baseline", st.VARIANT)]

# Falcon User's Guide (SpaceX, May 2025), Table 10-4, "Falcon 9 sample flight
# timeline -- LEO mission" [s]. The guide states no orbit, payload or recovery
# mode for it, and notes that every flight profile differs.
GUIDE_LEO = {"maxq": 67.0, "meco": 145.0, "ses1": 156.0, "fairing": 195.0,
             "seco1": 514.0, "ses2": 3086.0, "seco2": 3090.0}

T_END = 560.0      # s; the webcasts stop showing Stage 2 near SECO-1, ~515 s
Q_WINDOW = 200.0   # s; dynamic pressure is negligible after MECO


class Flight:
    """One webcast record, exposing the channels a Case exposes."""

    def __init__(self, name, label, linestyle, analysed, events):
        self.name = name
        self.label = label
        self.linestyle = linestyle
        self.events = events
        self.time = np.asarray(analysed["time"], dtype=float)
        self.v = np.asarray(analysed["velocity"], dtype=float)
        self.alt_km = np.asarray(analysed["altitude"], dtype=float)
        self.gamma_deg = np.asarray(analysed["angle"], dtype=float)
        self.downrange_km = np.asarray(analysed["downrange_distance"], dtype=float)
        self.q_dataset = np.asarray(analysed["q"], dtype=float)

    def event(self, key):
        t_evt = self.events.get(key)
        return np.nan if t_evt is None else float(t_evt)


def load_flights(data_dir=None):
    data_dir = Path(data_dir) if data_dir is not None else DATA_DIR
    flights = []
    for name, label, linestyle in FLIGHTS:
        folder = data_dir / name
        if not (folder / "analysed.json").exists():
            print("  [skip] flight %s -- no %s" % (name, folder / "analysed.json"))
            continue
        with open(folder / "analysed.json", encoding="utf-8") as fh:
            analysed = json.load(fh)
        with open(folder / "events.json", encoding="utf-8") as fh:
            events = json.load(fh)
        flights.append(Flight(name, label, linestyle, analysed, events))
    return flights


def _first_burn(case):
    """SES-1 and the end of the first Stage-2 burn, from the thrust trace."""
    spans = pn.burn_intervals(case, stage2_only=True)
    return spans[0] if spans else (np.nan, float(case.t_seco))


def _first_burn_end(case):
    return min(_first_burn(case)[1], T_END)


def fairing_time(case):
    """When the fairing left [s]: the 65 km crossing, or the arc boundary after it."""
    return _data._scalar(case._z, "t_atmosphere_exit")


def _q(time, v, alt_km):
    """Dynamic pressure [kPa] over the first Q_WINDOW seconds, simulator atmosphere."""
    sel = time <= Q_WINDOW
    return time[sel], psu.compute_dynamic_pressure(v[sel], alt_km[sel] * 1e3) / 1e3


def _at(time, values, t_evt):
    return float(np.interp(t_evt, time, values)) if np.isfinite(t_evt) else np.nan


def summary_rows(cases, flights):
    """Event values per flight and per case, for the console table."""
    rows = []
    for flight in flights:
        t_q, q = _q(flight.time, flight.v, flight.alt_km)
        i_q = int(np.nanargmax(q))
        t_meco = flight.event("meco")
        rows.append({
            "name": flight.label, "t_maxq": t_q[i_q], "maxq": q[i_q],
            "maxq_dataset": np.nanmax(flight.q_dataset[flight.time <= Q_WINDOW]) / 1e3,
            "t_meco": t_meco,
            "h_meco": _at(flight.time, flight.alt_km, t_meco),
            "v_meco": _at(flight.time, flight.v, t_meco),
            "g_meco": _at(flight.time, flight.gamma_deg, t_meco),
            "s_meco": _at(flight.time, flight.downrange_km, t_meco),
            "t_ses1": flight.event("ses1"), "t_burn_end": flight.event("seco1"),
        })
    for name, _colour in SIM_CASES:
        case = cases.get(name)
        if case is None:
            continue
        t_q, q = _q(case.time, case.v, case.alt_km)
        i_q = int(np.argmax(q))
        rows.append({
            "name": st.case_label(name), "t_maxq": t_q[i_q], "maxq": q[i_q],
            "maxq_dataset": np.nan, "t_meco": case.t_meco,
            "h_meco": _at(case.time, case.alt_km, case.t_meco),
            "v_meco": _at(case.time, case.v, case.t_meco),
            "g_meco": _at(case.time, case.gamma_deg, case.t_meco),
            "s_meco": _at(case.time, case.downrange_km, case.t_meco),
            "t_ses1": _first_burn(case)[0], "t_burn_end": _first_burn(case)[1],
        })
    return rows


def print_summary(rows):
    head = ("%-26s %6s %6s %6s %6s %6s %6s %6s %6s %6s %7s"
            % ("", "t_maxQ", "maxQ", "(data)", "t_MECO", "h", "v", "gamma",
               "s", "SES-1", "burn end"))
    print(head)
    print("%-26s %6s %6s %6s %6s %6s %6s %6s %6s %6s %7s"
          % ("", "[s]", "[kPa]", "[kPa]", "[s]", "[km]", "[km/s]", "[deg]",
             "[km]", "[s]", "[s]"))
    for r in rows:
        print("%-26s %6.0f %6.1f %6.1f %6.1f %6.1f %6.2f %6.1f %6.0f %6.0f %7.0f"
              % (r["name"], r["t_maxq"], r["maxq"], r["maxq_dataset"], r["t_meco"],
                 r["h_meco"], r["v_meco"] / 1e3, r["g_meco"], r["s_meco"],
                 r["t_ses1"], r["t_burn_end"]))


def ascent_against_flight_data(cases, flights=None):
    """Altitude, speed, flight-path angle and dynamic pressure, flights against cases.

    Flights in grey, one line style each; cases in the chapter's colours, cut at
    the end of their first Stage-2 burn. A dot marks each MECO. Panel (d) shades
    each flight's throttle-down, which the simulator does not model.
    """
    flights = load_flights() if flights is None else flights
    present = [(cases[n], colour) for n, colour in SIM_CASES if n in cases]
    if not flights or not present:
        print("  [skip] validation figure -- no flights or no cases")
        return None

    fig, axes = plt.subplots(2, 2, figsize=st.WIDE_4)
    ax_h, ax_v, ax_g, ax_q = axes.ravel()

    for flight in flights:
        style = dict(color=st.GREY, linestyle=flight.linestyle, linewidth=1.1)
        sel = flight.time <= T_END
        t = flight.time[sel]
        ax_h.plot(t, flight.alt_km[sel], label=flight.label, **style)
        ax_v.plot(t, flight.v[sel] / 1e3, **style)
        ax_g.plot(t, flight.gamma_deg[sel], **style)
        t_q, q = _q(flight.time, flight.v, flight.alt_km)
        ax_q.plot(t_q, q, **style)
        t0, t1 = flight.event("throttle_down_start"), flight.event("throttle_down_end")
        if np.isfinite(t0) and np.isfinite(t1):
            ax_q.axvspan(t0, t1, color=st.FAINT, alpha=0.35, linewidth=0)
        t_meco = flight.event("meco")
        for ax, values in ((ax_h, flight.alt_km), (ax_v, flight.v / 1e3),
                           (ax_g, flight.gamma_deg)):
            ax.plot(t_meco, _at(flight.time, values, t_meco), "o", ms=3,
                    color=st.GREY, zorder=4)

    for case, colour in present:
        sel = case.time <= _first_burn_end(case)
        t, h, v, g = st.thin(case.time[sel], case.alt_km[sel], case.v[sel] / 1e3,
                             case.gamma_deg[sel])
        ax_h.plot(t, h, color=colour, label=st.case_label(case.name))
        ax_v.plot(t, v, color=colour)
        ax_g.plot(t, g, color=colour)
        t_q, q = _q(case.time, case.v, case.alt_km)
        ax_q.plot(*st.thin(t_q, q), color=colour)
        for ax, values in ((ax_h, case.alt_km), (ax_v, case.v / 1e3),
                           (ax_g, case.gamma_deg)):
            ax.plot(case.t_meco, _at(case.time, values, case.t_meco), "o", ms=3,
                    color=colour, zorder=4)

    ax_h.set_ylabel("Altitude [km]")
    ax_v.set_ylabel("Speed, Earth-relative [km/s]")
    ax_g.set_ylabel("Flight-path angle [deg]")
    ax_g.set_ylim(-5.0, 95.0)
    ax_q.set_ylabel("Dynamic pressure [kPa]")
    ax_q.set_xlim(0.0, Q_WINDOW)
    buckets = [(f.event("throttle_down_start"), f.event("throttle_down_end"))
               for f in flights]
    buckets = [b for b in buckets if np.all(np.isfinite(b))]
    if buckets:
        mid = 0.5 * (min(b[0] for b in buckets) + max(b[1] for b in buckets))
        ax_q.annotate("flights'\nthrottle-down", xy=(mid, 0.03),
                      xycoords=("data", "axes fraction"), ha="center", va="bottom",
                      fontsize=6.5, color=st.GREY)
    ax_h.plot([], [], "o", ms=3, color=st.INK, label="MECO")
    for ax, letter in zip((ax_h, ax_v, ax_g, ax_q), "abcd"):
        if ax is not ax_q:
            ax.set_xlim(0.0, T_END)
        ax.set_xlabel("Time from liftoff [s]")
        st.panel_tag(ax, letter)
        st.tidy(ax, legend=False)
    pn.figure_legend(fig, ax_h, ncol=3)
    return st.save(fig, "results_validation_flights.png")


FIGURES = [ascent_against_flight_data]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", help="results directory "
                                       "(default: Output/results_matrix)")
    parser.add_argument("--data", help="webcast telemetry directory "
                                       "(default: Reference_Data/falcon9_webcast)")
    args = parser.parse_args()

    st.use_thesis_style()
    cases = _data.load_many([n for n, _c in SIM_CASES], root=args.root)
    _data.check_one_rotation_model(cases)
    flights = load_flights(args.data)
    print_summary(summary_rows(cases, flights))
    ascent_against_flight_data(cases, flights)


if __name__ == "__main__":
    main()
