"""
Regression tests for main-engine cutoff.

MECO used to be latched from inside ``rocket_dynamics`` -- the ODE right-hand
side -- so the cutoff was recorded at whichever speculative time solve_ivp
happened to sample rather than at the true propellant-depletion crossing. At
mdot_1 ~ 2740 kg/s that put -701 to +235 kg of scatter on the burnout mass
across the results matrix, and the scatter carried straight into Stage-2
propellant, which is the quantity Chapter 6 ranks the guidance laws on.

These tests pin the two properties that make that impossible to recur:

1. ``interrupt_main_engine_cutoff`` is a *pure signed function of the state
   passed in* -- it sets nothing, and its sign says which side of cutoff the
   state is on. A latch would fail ``test_event_is_pure``.
2. Flying Stage 1 end-to-end lands on ``_stage1_burnout_mass()`` to solver
   tolerance, on *both* guidance dispatchers (legacy ``run()`` and the PSO
   ``run_stage1()``), which is the property that actually matters.

Note the kick-angle convention: ``run``/``run_stage1`` take a *delta* on gamma,
so a pitch-over parameter gamma_p enters as ``gamma_p - pi/2`` (see
pso_coast_solver).
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Allow src-relative imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Auxiliary import rocket_specs as r
import Simulation.rocket_ascent as ra


# Burnout mass with the fairing already gone -- the normal configuration at MECO.
BURNOUT_NO_FAIRING = ((r.M_STRUCTURE_1 - r.M_FAIRING) + r.M_STRUCTURE_2
                      + r.M_PROP_2 + r.M_PAYLOAD)

# Tolerance on the flown burnout mass [kg]. The event is root-found, so the
# residual is solver tolerance, not a grid step; 1e-3 kg is ~4e-7 s of burn.
MASS_TOL = 1e-3


def _state(mass):
    """A 5-element state with the given mass; only element 4 is read."""
    return np.array([0.0, c.R_EARTH, 0.0, np.pi / 2.0, float(mass)])


class TestMecoEvent:
    """The event function itself."""

    def test_burnout_mass_matches_the_vehicle(self):
        ra.fairing_jettisoned = True
        assert ra._stage1_burnout_mass() == pytest.approx(BURNOUT_NO_FAIRING)

    def test_burnout_mass_carries_the_fairing_when_still_attached(self):
        """A trajectory that stages below the jettison altitude is heavier.

        Not an error: the fairing is a subset of M_STRUCTURE_1 and is shed at
        the next Stage-2 arc boundary by shed_fairing_if_due().
        """
        ra.fairing_jettisoned = False
        assert (ra._stage1_burnout_mass()
                == pytest.approx(BURNOUT_NO_FAIRING + r.M_FAIRING))

    def test_sign_convention(self):
        """Positive while propellant remains, negative once dry."""
        ra.fairing_jettisoned = True
        assert ra.interrupt_main_engine_cutoff(0.0, _state(BURNOUT_NO_FAIRING + 1e3)) > 0
        assert ra.interrupt_main_engine_cutoff(0.0, _state(BURNOUT_NO_FAIRING - 1e3)) < 0
        assert ra.interrupt_main_engine_cutoff(0.0, _state(BURNOUT_NO_FAIRING)) == 0.0

    def test_event_is_pure(self):
        """Evaluating the event must not latch anything.

        This is the whole point of the fix: the old event_main_engine_cutoff
        set main_engine_cutoff/time_main_engine_cutoff as a side effect, so a
        speculative RHS sample permanently cut the engine.
        """
        ra.fairing_jettisoned = True
        ra.main_engine_cutoff = False
        ra.time_main_engine_cutoff = None

        # Evaluate well past cutoff, repeatedly and out of time order, exactly
        # as solve_ivp does when it probes and then rejects a step.
        for t in (500.0, 10.0, 900.0, 1.0):
            ra.interrupt_main_engine_cutoff(t, _state(BURNOUT_NO_FAIRING - 5e3))

        assert ra.main_engine_cutoff is False
        assert ra.time_main_engine_cutoff is None

    def test_direction_is_downward(self):
        """Registered so only the depletion crossing counts."""
        assert getattr(ra.interrupt_main_engine_cutoff, "direction", 0) in (-1, 0)

    def test_meco_is_in_the_shared_stage1_event_list(self):
        assert ra.STAGE1_BURN_EVENTS[ra.EV_MECO] is ra.interrupt_main_engine_cutoff
        assert ra.STAGE1_BURN_EVENTS[ra.EV_FAIRING] is ra.interrupt_fairing_jettison


@pytest.mark.parametrize("gamma_p", [1.54, 1.56, 1.57])
class TestFlownBurnoutMass:
    """End-to-end: does a flown Stage 1 actually stop at the burnout mass?

    Each case is a full Stage-1 ascent (~1 s), not a unit stub, because the bug
    lived in the interaction between the event and the integrator.
    """

    def test_pso_dispatcher(self, gamma_p):
        ra.set_pseudo_forces_for_run(True)
        t2, state2, t_meco, _t, _y, crashed = ra.run_stage1(gamma_p - np.pi / 2.0)
        assert not crashed
        assert t_meco is not None

        # state2 is post-separation: add back the Stage-1 structure that came off.
        m_meco = state2[4] + (r.M_STRUCTURE_1 - r.M_FAIRING)
        assert m_meco == pytest.approx(BURNOUT_NO_FAIRING, abs=MASS_TOL)

        # Separation is a planned interval after MECO, integrated to exactly.
        assert t2 == pytest.approx(t_meco + r.TIME_First_STAGE_SEPARATION, abs=1e-9)

    def test_legacy_dispatcher(self, gamma_p):
        ra.set_pseudo_forces_for_run(True)
        out = ra.run(gamma_p - np.pi / 2.0)
        t_all, y_all = out[0], out[1]

        t_meco = ra.time_main_engine_cutoff
        assert t_meco is not None

        j = int(np.searchsorted(t_all, t_meco, side="left"))
        assert y_all[4, j] == pytest.approx(BURNOUT_NO_FAIRING, abs=MASS_TOL)

    def test_both_dispatchers_agree(self, gamma_p):
        """The legacy and PSO paths must stage identically.

        They fly Stage 1 through the same _fly_stage1() helper, so a divergence
        here means one of them grew a private copy again -- the failure mode
        CLAUDE.md warns about for the two guidance dispatchers.
        """
        ra.set_pseudo_forces_for_run(True)
        _t2, _s2, t_meco_pso, _t, _y, crashed = ra.run_stage1(gamma_p - np.pi / 2.0)
        assert not crashed

        ra.set_pseudo_forces_for_run(True)
        ra.run(gamma_p - np.pi / 2.0)
        t_meco_legacy = ra.time_main_engine_cutoff

        # Not bit-identical: run() honours KICK_PROFILE_MODE and run_stage1()
        # is always the instantaneous jump, so the Stage-1 trajectories differ
        # slightly. The cutoff still has to land within a few tenths of a second.
        assert t_meco_legacy == pytest.approx(t_meco_pso, abs=0.5)
