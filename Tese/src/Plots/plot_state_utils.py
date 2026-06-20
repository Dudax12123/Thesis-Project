"""Utilities for robust plotting across variable state-vector layouts."""

import numpy as np
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent))

from Auxiliary import constants as c
from Auxiliary import atmosphere as atm
from Auxiliary import rocket_specs as r


def reduce_data(time_steps, data, reduction_factor=10):
    """Reduce arrays for plotting while preserving endpoints."""
    if reduction_factor <= 1:
        return np.asarray(time_steps), np.asarray(data)

    time_steps = np.asarray(time_steps)
    data = np.asarray(data)

    time_reduced = time_steps[::reduction_factor]
    data_reduced = data[:, ::reduction_factor]

    if len(time_reduced) == 0 or time_reduced[-1] != time_steps[-1]:
        time_reduced = np.append(time_reduced, time_steps[-1])
        data_reduced = np.concatenate((data_reduced, data[:, -1][:, None]), axis=1)

    return time_reduced, data_reduced


def prepare_monotonic_series(time_array, value_array):
    """Sort by time and keep last sample for duplicate timestamps."""
    t = np.asarray(time_array)
    v = np.asarray(value_array)

    if len(t) == 0:
        return t, v

    order = np.argsort(t, kind="stable")
    t_sorted = t[order]
    v_sorted = v[order]

    unique_t, first_idx = np.unique(t_sorted, return_index=True)
    next_idx = np.r_[first_idx[1:], len(t_sorted)]
    last_idx = next_idx - 1

    return unique_t, v_sorted[last_idx]


def extract_state_channels(data):
    """Extract mandatory and optional state channels safely."""
    data = np.asarray(data)
    if data.ndim != 2 or data.shape[0] < 5:
        raise ValueError("State data must have shape [N_states, N_times] with at least 5 states.")

    channels = {
        "s": data[0],
        "r": data[1],
        "v": data[2],
        "gamma": data[3],
        "m": data[4],
        "lat": data[5] if data.shape[0] > 5 else None,
        "heading": data[6] if data.shape[0] > 6 else None,
    }

    channels["alt"] = channels["r"] - c.R_EARTH
    channels["alt_km"] = channels["alt"] / 1000.0
    channels["s_km"] = channels["s"] / 1000.0
    return channels


def compute_dynamic_pressure(v, alt):
    """Compute dynamic pressure profile."""
    v = np.asarray(v)
    alt = np.asarray(alt)
    return np.array([atm.dynamic_pressure(float(v_i), float(max(0.0, alt_i))) for v_i, alt_i in zip(v, alt)])


def compute_mach(v, alt):
    """Compute Mach number profile."""
    v = np.asarray(v)
    alt = np.asarray(alt)
    a = np.array([atm.speed_of_sound(float(max(0.0, alt_i))) for alt_i in alt])
    a = np.maximum(a, 1e-6)
    return v / a


def cutoff_index(time_array, t_cutoff):
    """Return the index up to which *time_array* should be kept.

    Returns ``len(time_array)`` when *t_cutoff* is ``None`` so callers
    can always slice with ``[:idx]``.
    """
    if t_cutoff is None:
        return len(time_array)
    t = np.asarray(time_array)
    return int(np.searchsorted(t, t_cutoff, side='right'))


def interpolate_to_time(time_source, values_source, time_target):
    """Monotonic interpolation helper."""
    t_src, v_src = prepare_monotonic_series(time_source, values_source)
    if len(t_src) == 0:
        return np.zeros_like(np.asarray(time_target), dtype=float)
    return np.interp(np.asarray(time_target), t_src, v_src)


def compute_propellant_mass(total_mass, time_steps=None):
    """Estimate propellant mass from total mass and dry masses.

    Accounts for fairing jettison (small drop) and Stage-1 structure drop (large drop).
    The fairing can be jettisoned before staging (Stage 1) or after staging (Stage 2).
    """
    total_mass = np.asarray(total_mass, dtype=float)
    M_FAIRING = getattr(r, 'M_FAIRING', 0.0)

    m_dry_pre_fairing       = r.M_STRUCTURE_1 + r.M_STRUCTURE_2 + r.M_PAYLOAD
    m_dry_post_fairing      = (r.M_STRUCTURE_1 - M_FAIRING) + r.M_STRUCTURE_2 + r.M_PAYLOAD
    m_dry_post_staging      = r.M_STRUCTURE_2 + r.M_PAYLOAD
    m_dry_stage2_with_fair  = r.M_STRUCTURE_2 + r.M_PAYLOAD + M_FAIRING

    if len(total_mass) > 1:
        dm = np.diff(total_mass)
        big_drop_idx = np.where(dm < -r.M_STRUCTURE_1 * 0.5)[0]
        if M_FAIRING > 0:
            small_drop_idx = np.where(
                (dm < -M_FAIRING * 0.5) & (dm > -r.M_STRUCTURE_1 * 0.5)
            )[0]
        else:
            small_drop_idx = np.array([], dtype=int)

        m_dry = np.full_like(total_mass, m_dry_post_staging)

        if len(big_drop_idx) > 0:
            sep_idx = big_drop_idx[0] + 1
            m_dry[sep_idx:] = m_dry_post_staging

            if len(small_drop_idx) > 0:
                fair_idx = small_drop_idx[0] + 1
                if fair_idx <= sep_idx:                        # fairing before staging (Stage 1)
                    m_dry[:fair_idx] = m_dry_pre_fairing
                    m_dry[fair_idx:sep_idx] = m_dry_post_fairing
                else:                                          # fairing after staging (Stage 2)
                    m_dry[:sep_idx] = m_dry_pre_fairing
                    m_dry[sep_idx:fair_idx] = m_dry_stage2_with_fair
                    # m_dry[fair_idx:] already = m_dry_post_staging
            else:
                m_dry[:sep_idx] = m_dry_pre_fairing            # no fairing drop detected
        else:
            if len(small_drop_idx) > 0:
                fair_idx = small_drop_idx[0] + 1
                m_dry[:fair_idx] = m_dry_pre_fairing
                m_dry[fair_idx:] = m_dry_post_fairing
            else:
                m_dry[:] = m_dry_pre_fairing
    else:
        m_dry = np.full_like(total_mass, m_dry_pre_fairing)

    return np.maximum(total_mass - m_dry, 0.0)


def compute_acceleration_components(time_steps, channels, thrust_data=None, time_thrust=None,
                                    alpha_data=None, alpha_time_data=None):
    """Compute acceleration diagnostics used by new plots.

    Total acceleration is computed analytically as
        a_total = (F_T/m)*cos(alpha) - F_D/m - g*sin(gamma)
    to avoid unphysical spikes from numerical differentiation at
    thrust discontinuities (staging, circularization impulse).
    """
    time_steps = np.asarray(time_steps)
    v = np.asarray(channels["v"])
    r_arr = np.asarray(channels["r"])
    m_arr = np.asarray(channels["m"])
    gamma = np.asarray(channels["gamma"])
    alt = np.asarray(channels["alt"])

    q = compute_dynamic_pressure(v, alt)
    drag_force = atm.drag_force(q)
    drag_accel = drag_force / np.maximum(m_arr, 1e-6)

    grav_accel = np.array([c.MU_EARTH / max(r_i**2, 1e-6) for r_i in r_arr])
    grav_along = grav_accel * np.sin(gamma)

    if thrust_data is not None and time_thrust is not None:
        thrust_interp = interpolate_to_time(time_thrust, thrust_data, time_steps)
        thrust_accel = thrust_interp / np.maximum(m_arr, 1e-6)
    else:
        thrust_accel = np.zeros_like(v)

    # Steering angle interpolated onto the time grid
    if alpha_data is not None and alpha_time_data is not None:
        alpha_interp = interpolate_to_time(alpha_time_data, alpha_data, time_steps)
    else:
        alpha_interp = np.zeros_like(v)

    # Analytical total acceleration along the velocity direction
    total_accel = thrust_accel * np.cos(alpha_interp) - drag_accel - grav_along

    return {
        "total_accel": total_accel,
        "thrust_accel": thrust_accel,
        "drag_accel": drag_accel,
        "grav_along": grav_along,
    }


def event_times():
    """Get event markers robustly from rocket_ascent globals."""
    from Simulation import rocket_ascent as ra

    return {
        "guidance_start": ra.time_guidance_start,
        "meco": ra.time_main_engine_cutoff,
        "seco": ra.TIME_TO_STOP_BURNING_SINGLE_BURN_FINAL,
    }


def add_event_markers(ax):
    """Draw vertical dotted lines for key flight events on a time-axis plot."""
    events = event_times()
    markers = [
        ("guidance_start", "Guidance Start", "cyan"),
        ("meco", "MECO", "orange"),
        ("seco", "SECO", "black"),
    ]
    for key, label, color in markers:
        t_evt = events.get(key)
        if t_evt is None:
            continue
        ax.axvline(x=t_evt, color=color, linestyle=':', linewidth=1.2,
                   alpha=0.8, label=label)
