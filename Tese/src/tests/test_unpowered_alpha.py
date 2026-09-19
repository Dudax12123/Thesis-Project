"""
Tests for the plotting convention that an unpowered sample has alpha = 0
(Plots.plot_state_utils.zero_alpha_when_unpowered / powered_mask).

The archived alpha channel is the guidance log interpolated onto the output grid, so
with the engine off it held the last command flat after cutoff and drew a straight
ramp across every coast. None of that is attitude: a point mass with no thrust has none.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from Plots import plot_state_utils as psu

# burn -> coast -> burn -> post-insertion coast, on one grid, with each switch instant
# written twice (once on each side), exactly as the PSO architectures archive it
T = np.array([0.0, 1.0, 2.0, 2.0, 3.0, 4.0, 4.0, 5.0, 6.0, 6.0, 7.0])
F = np.array([9.0, 9.0, 9.0, 0.0, 0.0, 0.0, 9.0, 9.0, 9.0, 0.0, 0.0])
A = np.array([0.1, 0.2, 0.3, 0.3, 0.1, -0.1, -0.2, -0.1, 0.05, 0.05, 0.05])


def test_same_grid_keeps_both_sides_of_every_switch():
    out = psu.zero_alpha_when_unpowered(T, A, T, F)
    np.testing.assert_array_equal(out[F > 0], A[F > 0])     # powered samples untouched
    assert np.all(out[F == 0] == 0.0)                        # coast and post-insertion zeroed
    assert out[2] == A[2] and out[3] == 0.0                  # cutoff instant: last command, then 0
    assert out[5] == 0.0 and out[6] == A[6]                  # ignition instant: 0, then command


def test_other_grid_holds_the_thrust_trace_from_the_left():
    ta = np.array([0.5, 2.0, 2.5, 3.9, 4.0, 5.5, 6.0, 9.0])
    mask = psu.powered_mask(ta, T, F)
    # at a switch instant the state the engine switched TO wins
    assert mask.tolist() == [True, False, False, False, True, True, False, False]


def test_a_legacy_log_out_of_time_order_is_sorted_first():
    order = np.array([3, 0, 7, 1, 5, 2, 10, 4, 8, 6, 9])
    mask = psu.powered_mask(np.array([1.5, 3.5, 5.5]), T[order], F[order])
    assert mask.tolist() == [True, False, True]


def test_no_thrust_trace_leaves_alpha_alone():
    np.testing.assert_array_equal(psu.zero_alpha_when_unpowered(T, A, [], []), A)
