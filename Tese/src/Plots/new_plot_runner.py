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


def _make_path(output_dir, filename):
    if output_dir is None:
        return None
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    return out / filename


def run_new_plot_suite(time, data, thrust_data, time_thrust, alpha_data, alpha_time_data,
                       output_dir=None, show=False, close_after=True):
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
        "mach": _make_path(output_dir, "new_09_mach_number_over_time.png"),
        "traj": _make_path(output_dir, "new_10_trajectory_fixed.png"),
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
    plot_mach_number_over_time(time, data, save_path=files["mach"], show=show)
    plot_trajectory_xy_fixed(time, data, save_path=files["traj"], show=show)

    if close_after:
        plt.close('all')

    return files
