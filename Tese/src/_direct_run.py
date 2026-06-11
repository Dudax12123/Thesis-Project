import io
import contextlib
import numpy as np
from Input_File import simulation_parameters as sp
from Auxiliary import constants as c

sp.COAST_METHOD = "direct"
sp.RUN_FAST = False
sp.EVENTS_PRINT = False
sp.INTERRUPTS_PRINT = False

import Simulation.rocket_ascent as ra

vt = np.sqrt(c.MU_EARTH / (c.R_EARTH + sp.TARGET_ORBITAL_ALTITUDE))
print(f"direct insertion: v_target_inertial={vt:.1f} m/s; box v={sp.DIRECT_INSERTION_VELOCITY_TOL_MS} "
      f"fpa={sp.DIRECT_INSERTION_FPA_TOL_DEG} alt={sp.DIRECT_INSERTION_ALTITUDE_TOL_KM}km")
for mode in ["peg_new", "apollo", "linear_tangent"]:
    sp.GUIDANCE_MODE = mode
    for kd in [-2.5, -3.5, -4.5]:
        ra.SINGLE_BURN_FULL_SIMULATION = True
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            out = ra.run(np.deg2rad(kd))
        txt = buf.getvalue()
        def grab(key):
            for l in txt.splitlines():
                if key in l:
                    return l.split(key)[1].strip()
            return "?"
        print("%-14s kick=%5.2f | reached=%s | alt=%s | %s"
              % (mode, kd, grab("Reached target box:"),
                 grab("Insertion altitude:"), grab("Achieved orbit:")), flush=True)
print("DONE")
