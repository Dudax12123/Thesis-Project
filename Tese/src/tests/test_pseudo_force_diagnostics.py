"""The pseudo-force diagnostics are those of the flight on the launch azimuth, while
the terms flown are those of a due-east flight (decision 2026-10-08)."""
import sys
from pathlib import Path

import numpy as np

# Allow src-relative imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import earth_rotation as er
from Simulation import rocket_ascent as ra


def _set_site(monkeypatch, azimuth_deg=44.98, latitude_deg=28.5):
    monkeypatch.setattr(ra, "LAUNCH_AZIMUTH", np.deg2rad(azimuth_deg))
    monkeypatch.setattr(ra, "LAUNCH_LATITUDE_RAD", np.deg2rad(latitude_deg))


def test_diagnostics_are_evaluated_on_the_launch_azimuth(monkeypatch):
    _set_site(monkeypatch)
    v, gamma, r = 6500.0, np.deg2rad(3.0), 6.378e6 + 150e3
    cross, cor, cen = ra.pseudo_force_diagnostics(v, gamma, r)
    expected = er.rotating_frame_pseudoforce_rates(v, gamma, ra.LAUNCH_AZIMUTH,
                                                   ra.LAUNCH_LATITUDE_RAD, r)
    assert (cross, cor, cen) == expected[3:]


def test_flown_terms_stay_due_east(monkeypatch):
    _set_site(monkeypatch)
    v, gamma, r = 6500.0, np.deg2rad(3.0), 6.378e6 + 150e3
    flown = er.planar_pseudoforce_rates(v, gamma, ra.LAUNCH_LATITUDE_RAD, r)
    due_east = er.rotating_frame_pseudoforce_rates(v, gamma, np.pi / 2.0,
                                                   ra.LAUNCH_LATITUDE_RAD, r)
    assert flown == due_east
    # Off due east the Coriolis magnitude differs, so the diagnostic is not the flown term.
    assert abs(ra.pseudo_force_diagnostics(v, gamma, r)[1] - flown[4]) > 0.05
