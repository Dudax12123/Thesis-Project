"""Regression tests for the two guidance-law fixes of 2026-09-10.

Classical PEG (peg_guidance):
    the steering command must carry the gravity/centrifugal term C of the
    Orbiter-wiki reference, sin(pitch) = A + B t + C, and the estimate step
    must use f_r = A + C.

Apollo (apollo_guidance):
    the non-thrust accelerations subtracted from the polynomial's commands
    must be the local-frame kinematics of (vx, vy) = (v cos g, v sin g) as
    given by the simulator's own equations of motion, and when the available
    thrust acceleration is supplied the vertical channel is served first and
    the downrange channel takes the remainder (Apollo P12).

The ignition state used below is the archived show_peg / show_apollo
second-stage ignition of the 2026-08 results batch, written out as numbers so
the test does not depend on the (gitignored) archive.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from Auxiliary import constants as c            # noqa: E402
from Auxiliary import rocket_specs as r         # noqa: E402
from Auxiliary import gravity as grav           # noqa: E402
import Guidance.peg_guidance as peg             # noqa: E402
import Guidance.apollo_guidance as ap           # noqa: E402
from Simulation import rocket_ascent as ra      # noqa: E402

R_T   = c.R_EARTH + 500e3
VE    = r.ISP_2 * c.G_0
F_T   = r.F_THRUST_2
V_TH_T_INERTIAL = np.sqrt(c.MU_EARTH / R_T)

# show_peg second-stage ignition: h 136.2 km, v 2989 m/s, gamma 49.3 deg, m 96786 kg
STATE_STEEP = np.array([91e3, c.R_EARTH + 136.2e3, 2989.0, np.deg2rad(49.3), 96786.0])


# ---------------------------------------------------------------------------
# Classical PEG
# ---------------------------------------------------------------------------

class TestPegGravityTerm:

    def test_gravity_term_matches_reference_definition(self):
        s, rr, v, g, m = STATE_STEEP
        v_theta = v * np.cos(g)
        expected = (c.MU_EARTH / rr**2 - (v_theta / rr)**2 * rr) / (F_T / m)
        assert peg.compute_gravity_term(STATE_STEEP, F_T, c.MU_EARTH) == pytest.approx(expected)
        # This stage cannot quite hold altitude thrusting vertically at ignition.
        assert 0.85 < expected < 0.95

    def test_gravity_term_vanishes_on_circular_orbit(self):
        st = np.array([0.0, R_T, V_TH_T_INERTIAL, 0.0, 20000.0])
        assert peg.compute_gravity_term(st, F_T, c.MU_EARTH) == pytest.approx(0.0, abs=1e-12)

    def test_steering_carries_C(self):
        A, B, C, g = -0.62, 1e-3, 0.91, np.deg2rad(49.3)
        alpha = peg.peg_alpha(10.0, A, B, g, C)
        expected = np.arcsin(A + B * 10.0 + C) - g
        assert alpha == pytest.approx(expected)
        # Without C the old (wrong) command comes back — the argument is not optional.
        assert peg.peg_alpha(10.0, A, B, g, 0.0) == pytest.approx(np.arcsin(A + B * 10.0) - g)
        with pytest.raises(TypeError):
            peg.peg_alpha(10.0, A, B, g)

    def test_converged_command_at_steep_ignition(self):
        """At the archived show_peg ignition the reference gives a thrust pitched
        ~17 deg above the horizon; the pre-fix law gave -48 deg (alpha -97 deg)."""
        s, rr, v, g, m = STATE_STEEP
        T_seed = (m - r.M_STRUCTURE_2) / (F_T / VE)
        A, B, T = peg.converge_peg(STATE_STEEP, T_seed, VE, F_T, R_T, c.MU_EARTH,
                                   max_iter=30, tol=0.5, damping=0.5,
                                   v_theta_T=V_TH_T_INERTIAL)
        C = peg.compute_gravity_term(STATE_STEEP, F_T, c.MU_EARTH)
        pitch = np.arcsin(np.clip(A + C, -1.0, 1.0))
        assert np.rad2deg(pitch) == pytest.approx(17.0, abs=4.0)
        alpha0 = peg.peg_alpha(0.0, A, B, g, C)
        assert -40.0 < np.rad2deg(alpha0) < -25.0
        # The thrust must never have a rearward component at ignition.
        assert np.cos(alpha0) > 0.0
        assert 200.0 < T < 340.0

    def test_estimate_collapses_burn_time_near_target(self):
        """Just short of the target circular orbit with A = B = 0 there is almost
        no angular momentum to gain, so the estimate must return a burn time of
        well under a second rather than the seed it was given."""
        st = np.array([0.0, R_T, 0.999 * V_TH_T_INERTIAL, 0.0, 20000.0])
        T = 50.0
        T_new = peg.estimate_peg_T(0.0, 0.0, T, st, VE, F_T, R_T, c.MU_EARTH)
        assert 0.1 <= T_new < 1.0

    def test_estimate_uses_f_r_equal_A_plus_C(self):
        """Reproduce the reference estimate step by hand for one iterate and
        compare with the module (this is the algebra that was wrong before)."""
        s, rr, v, g, m = STATE_STEEP
        A, B, T = -0.62, 1.2e-3, 284.0
        a0 = F_T / m
        tau = VE / a0
        v_theta = v * np.cos(g)
        h, h_T = rr * v_theta, R_T * V_TH_T_INERTIAL
        r_bar = 0.5 * (rr + R_T)
        C = (c.MU_EARTH / rr**2 - (v_theta / rr)**2 * rr) / a0
        a_T = VE / (tau - T)
        C_T = (c.MU_EARTH / R_T**2 - (V_TH_T_INERTIAL / R_T)**2 * R_T) / a_T
        f_r = A + C
        f_rT = A + B * T + C_T
        f_rd = (f_rT - f_r) / T
        f_th = 1 - f_r**2 / 2
        f_thd = -f_r * f_rd
        f_thdd = -f_rd**2 / 2
        num = (h_T - h) / r_bar + VE * T * (f_thd + f_thdd * tau) + f_thdd * VE * T**2 / 2
        den = f_th + f_thd * tau + f_thdd * tau**2
        T_expected = tau * (1 - np.exp(-(num / den) / VE))
        T_new = peg.estimate_peg_T(A, B, T, STATE_STEEP, VE, F_T, R_T, c.MU_EARTH)
        assert T_new == pytest.approx(T_expected, rel=1e-9)


# ---------------------------------------------------------------------------
# Apollo
# ---------------------------------------------------------------------------

def _eom_rates_no_thrust(state):
    """d(vx)/dt, d(vy)/dt from the simulator's own EOM with thrust, lift, drag = 0."""
    s, rr, v, g, m = state
    a_grav = grav.gravitational_acceleration(rr)
    d = ra.diff_eom_base(s, rr, v, g, m, 0.0, 0.0, 0.0, a_grav, 0.0, r.ISP_2)
    dv, dg = d[2], d[3]
    dvx = dv * np.cos(g) - v * np.sin(g) * dg
    dvy = dv * np.sin(g) + v * np.cos(g) * dg
    return dvx, dvy


class TestApolloFrame:

    @pytest.mark.parametrize("state", [
        STATE_STEEP,
        np.array([1400e3, c.R_EARTH + 480e3, 7000.0, np.deg2rad(2.0), 25000.0]),
        np.array([3000e3, R_T, V_TH_T_INERTIAL, 0.0, 20000.0]),
    ])
    def test_frame_terms_equal_the_simulator_eom(self, state):
        """The accelerations the law subtracts must be exactly what the EOM does
        to (vx, vy) with the engine off — otherwise the polynomial is corrected
        for a model the vehicle does not fly."""
        ax_f, ay_f = ap.local_frame_accelerations(state)
        dvx, dvy = _eom_rates_no_thrust(state)
        assert ax_f == pytest.approx(dvx, rel=1e-9, abs=1e-9)
        assert ay_f == pytest.approx(dvy, rel=1e-9, abs=1e-9)

    def test_no_vertical_thrust_needed_in_circular_orbit(self):
        """A vehicle already on the target circular orbit with zero commanded
        acceleration must be told to thrust along the horizontal. The old
        gravity-only correction commanded +g upward here."""
        st = np.array([3000e3, R_T, V_TH_T_INERTIAL, 0.0, 20000.0])
        alpha, _ = ap.apollo_guidance(0.0, 0.0, st, [0.0, 0.0, 0.0, 0.0],
                                      a_thrust_available=F_T / 20000.0)
        assert abs(np.rad2deg(alpha)) < 0.05

    def test_no_downrange_dependence(self):
        """The old frame rotated gravity by s/R_E; the local-frame terms must not
        depend on downrange at all."""
        a = STATE_STEEP.copy()
        b = STATE_STEEP.copy(); b[0] += 2000e3
        assert ap.local_frame_accelerations(a) == pytest.approx(ap.local_frame_accelerations(b))


class TestApolloMagnitude:

    def test_direction_uses_all_available_thrust(self):
        """With the available acceleration given, the commanded (ax, ay) must have
        exactly that magnitude and the vertical channel must be the polynomial's."""
        st = STATE_STEEP
        a_T = F_T / st[4]
        k = [0.0, 18.6, 0.0, -5.49]      # the archived show_apollo ignition demand
        ax_f, ay_f = ap.local_frame_accelerations(st)
        alpha, demanded = ap.apollo_guidance(0.0, 0.0, st, k, a_thrust_available=a_T)
        ay_thrust = k[3] - ay_f
        assert abs(ay_thrust) < a_T
        ax_cmd = np.sqrt(a_T**2 - ay_thrust**2)
        expected_alpha = np.arctan2(ay_thrust, ax_cmd) - st[3]
        assert alpha == pytest.approx(expected_alpha, abs=1e-12)
        assert demanded > a_T                        # the polynomial asked for more

    def test_vertical_priority_when_demand_exceeds_thrust(self):
        st = STATE_STEEP
        a_T = F_T / st[4]
        ax_f, ay_f = ap.local_frame_accelerations(st)
        k = [0.0, 5.0, 0.0, 3 * a_T + ay_f]          # vertical demand = 3 a_T
        alpha, _ = ap.apollo_guidance(0.0, 0.0, st, k, a_thrust_available=a_T)
        # Clipped to +a_T vertical, zero horizontal: thrust straight up
        assert np.rad2deg(alpha + st[3]) == pytest.approx(90.0, abs=1e-9)

    def test_direction_only_fallback_kept(self):
        st = STATE_STEEP
        k = [0.0, 18.6, 0.0, -5.49]
        ax_f, ay_f = ap.local_frame_accelerations(st)
        alpha, mag = ap.apollo_guidance(0.0, 0.0, st, k)
        assert alpha == pytest.approx(np.arctan2(k[3] - ay_f, k[1] - ax_f) - st[3])
        assert mag == pytest.approx(np.hypot(k[1] - ax_f, k[3] - ay_f))
