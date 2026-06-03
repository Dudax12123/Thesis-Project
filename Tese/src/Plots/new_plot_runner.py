"""Runner for the new one-file-per-metric plotting suite."""

from pathlib import Path
import matplotlib.pyplot as plt

from Plots.new_metrics.altitude_over_time import plot_altitude_over_time
from Plots.new_metrics.dynamic_pressure_over_time import plot_dynamic_pressure_over_time
from Plots.new_metrics.fpa_over_time import plot_fpa_over_time
from Plots.new_metrics.mach_number_over_time import plot_mach_number_over_time
from Plots.new_metrics.propellant_mass_over_time import plot_propellant_mass_over_time
from Plots.new_metrics.rocket_accelerations_over_time import plot_rocket_accelerations_over_time
from Plots.new_metrics.steering_angle_over_time import plot_steering_angle_over_time
from Plots.new_metrics.thrust_over_time import plot_thrust_over_time
from Plots.new_metrics.total_mass_over_time import plot_total_mass_over_time
from Plots.new_metrics.trajectory_xy_fixed import plot_trajectory_xy_fixed
from Plots.new_metrics.pseudo_forces_over_time import plot_pseudo_forces_over_time
from Plots.new_metrics.latitude_over_time import plot_latitude_over_time
from Plots.new_metrics.aero_forces_over_time import plot_aero_forces_over_time
from Plots.new_metrics.trajectory_losses_over_time import plot_trajectory_losses_over_time
from Plots.new_metrics.mass_flow_rate_over_time import plot_mass_flow_rate_over_time
from Plots.new_metrics.apollo_tgo_over_time import plot_apollo_tgo_over_time
from Plots.new_metrics.pitch_angle_over_time import plot_pitch_angle_over_time
from Plots.new_metrics.cross_heading_counter_force_over_time import plot_cross_heading_counter_force_over_time
from Plots.new_metrics.cross_heading_accel_over_time import plot_cross_heading_accel_over_time
from Plots.new_metrics.pso_convergence import plot_best_objective_over_generations


def _make_path(output_dir, filename):
    if output_dir is None:
        return None
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    return out / filename


def run_new_plot_suite(time, data, thrust_data, time_thrust, alpha_data, alpha_time_data,
                       output_dir=None, show=False, close_after=True,
                       coriolis_mag_data=None, centrifugal_mag_data=None,
                       tgo_time_data=None, tgo_data=None, apollo_freeze_threshold=None,
                       theta_data=None, theta_time_data=None,
                       cross_heading_counter_force_data=None,
                       cross_heading_accel_data=None,
                       pso_history=None):
    """Generate all new metric plots for a run."""
    files = {
        "fpa": _make_path(output_dir, "new_01_fpa_over_time.png"),
        "steering": _make_path(output_dir, "new_02_steering_angle_over_time.png"),
        "thrust": _make_path(output_dir, "new_03_thrust_over_time.png"),
        "propellant": _make_path(output_dir, "new_04_propellant_mass_over_time.png"),
        "altitude": _make_path(output_dir, "new_05_altitude_over_time.png"),
        "total_mass": _make_path(output_dir, "new_06_total_mass_over_time.png"),
        "q": _make_path(output_dir, "new_07_dynamic_pressure_over_time.png"),
        "accel": _make_path(output_dir, "new_08_rocket_accelerations_over_time.png"),
        "pseudo": _make_path(output_dir, "new_08b_pseudo_forces_over_time.png"),
        "mach": _make_path(output_dir, "new_09_mach_number_over_time.png"),
        "traj": _make_path(output_dir, "new_10_trajectory_fixed.png"),
        "lat": _make_path(output_dir, "new_11_latitude_over_time.png"),
        "aero": _make_path(output_dir, "new_12_aero_forces_over_time.png"),
        "losses": _make_path(output_dir, "new_13_trajectory_losses_over_time.png"),
        "mdot": _make_path(output_dir, "new_14_mass_flow_rate_over_time.png"),
        "apollo_tgo": _make_path(output_dir, "new_15_apollo_tgo_over_time.png"),
        "pitch": _make_path(output_dir, "new_16_pitch_angle_over_time.png"),
        "cross_heading_force": _make_path(output_dir, "new_17_cross_heading_counter_force_over_time.png"),
        "cross_heading_accel": _make_path(output_dir, "new_18_cross_heading_accel_over_time.png"),
        "pso_obj": _make_path(output_dir, "new_19_pso_best_objective.png"),
    }

    plot_fpa_over_time(time, data, save_path=files["fpa"], show=show)
    plot_steering_angle_over_time(alpha_data, alpha_time_data, save_path=files["steering"], show=show)
    plot_thrust_over_time(time, thrust_data, time_thrust, save_path=files["thrust"], show=show)
    plot_propellant_mass_over_time(time, data, save_path=files["propellant"], show=show)
    plot_altitude_over_time(time, data, save_path=files["altitude"], show=show)
    plot_total_mass_over_time(time, data, save_path=files["total_mass"], show=show)
    plot_dynamic_pressure_over_time(time, data, save_path=files["q"], show=show)
    plot_rocket_accelerations_over_time(time, data, thrust_data, time_thrust,
                                        alpha_data=alpha_data, alpha_time_data=alpha_time_data,
                                        save_path=files["accel"], show=show)
    if coriolis_mag_data is not None and centrifugal_mag_data is not None:
        plot_pseudo_forces_over_time(time, time_thrust,
                                    coriolis_mag_data, centrifugal_mag_data,
                                    save_path=files["pseudo"], show=show)
    plot_mach_number_over_time(time, data, save_path=files["mach"], show=show)
    plot_trajectory_xy_fixed(time, data, save_path=files["traj"], show=show)
    plot_latitude_over_time(time, data, save_path=files["lat"], show=show)
    plot_aero_forces_over_time(time, data, save_path=files["aero"], show=show)
    plot_trajectory_losses_over_time(time, data, thrust_data, time_thrust,
                                     alpha_data, alpha_time_data,
                                     save_path=files["losses"], show=show)
    plot_mass_flow_rate_over_time(time, thrust_data, time_thrust,
                                  save_path=files["mdot"], show=show)

    if tgo_time_data is not None and tgo_data is not None and len(tgo_time_data) > 0:
        plot_apollo_tgo_over_time(
            tgo_time_data, tgo_data,
            freeze_threshold=apollo_freeze_threshold,
            save_path=files["apollo_tgo"], show=show,
        )

    if theta_data is not None and theta_time_data is not None and len(theta_time_data) > 0:
        plot_pitch_angle_over_time(theta_data, theta_time_data,
                                   save_path=files["pitch"], show=show)

    if cross_heading_counter_force_data is not None and len(cross_heading_counter_force_data) > 0:
        plot_cross_heading_counter_force_over_time(
            time_thrust, cross_heading_counter_force_data,
            save_path=files["cross_heading_force"], show=show,
        )

    if cross_heading_accel_data is not None and len(cross_heading_accel_data) > 0:
        plot_cross_heading_accel_over_time(
            time_thrust, cross_heading_accel_data,
            save_path=files["cross_heading_accel"], show=show,
        )

    if pso_history is not None and len(pso_history.get('gen', [])) > 0:
        plot_best_objective_over_generations(
            pso_history['gen'], pso_history['gbest'],
            save_path=files["pso_obj"], show=show,
        )

    if close_after:
        plt.close('all')

    return files
