# Earth Rotation Implementation Notes

## Overview
This update adds optional Earth-rotation support to the ascent simulation.

When enabled, the simulation now:
- Accounts for eastward velocity gain from Earth rotation.
- Computes launch azimuth from launch latitude and target inclination.
- Applies an azimuth correction in the rotating frame (ECEF-like) to compensate Earth rotation deflection.
- Converts velocity from rotating frame to inertial frame (ECI) when evaluating orbital conditions.
- Tracks latitude as an additional state variable during integration.

By default, this feature is disabled to preserve existing behavior.

---

## New User Parameters
Added in `src/Input_File/simulation_parameters.py`:

- `ENABLE_EARTH_ROTATION = False`
- `LAUNCH_LATITUDE = 28.5` (deg)
- `LAUNCH_LONGITUDE = -80.5` (deg)
- `TARGET_ORBIT_INCLINATION = 28.5` (deg)

Notes:
- `LAUNCH_LONGITUDE` is currently stored for future launch-window extensions and does not yet change 2D dynamics.

---

## Main Formulas Used

1. Geometric azimuth from inclination/latitude (ascending node):

\[
\sin(\beta) = \frac{\cos(i)}{\cos(\phi)}
\]

where:
- \(\beta\): inertial azimuth
- \(i\): target inclination
- \(\phi\): launch latitude

2. Surface rotation velocity at latitude:

\[
v_{rot} = \omega_E R_E \cos(\phi)
\]

3. Rotating-frame azimuth correction:

Given target horizontal orbital velocity components in inertial frame,

\[
V_N = v_{orb}\cos(\beta_{inertial}), \quad
V_E^{ecef} = v_{orb}\sin(\beta_{inertial}) - v_{rot}
\]

\[
\beta_{corrected} = \operatorname{atan2}(V_E^{ecef}, V_N)
\]

4. ECEF-to-ECI velocity conversion (local decomposition):

- Horizontal: \(v_h = v\cos(\gamma)\)
- Radial: \(v_r = v\sin(\gamma)\)
- North: \(v_N = v_h\cos(\beta)\)
- East (ECEF): \(v_E = v_h\sin(\beta)\)
- East (ECI): \(v_E^{eci} = v_E + \omega_E r\cos(\phi)\)

Then:

\[
v_{eci} = \sqrt{(v_E^{eci})^2 + v_N^2 + v_r^2},
\quad
\gamma_{eci} = \operatorname{atan2}(v_r, \sqrt{(v_E^{eci})^2 + v_N^2})
\]

5. Approximate latitude propagation in 2D model:

\[
\dot{\phi} = \frac{v\cos(\gamma)\cos(\beta)}{r}
\]

---

## New Module
Created:
- `src/Auxiliary/earth_rotation.py`

Key functions:
- `surface_rotation_velocity(...)`
- `geometric_azimuth(...)`
- `corrected_azimuth(...)`
- `ecef_to_eci_velocity(...)`
- `delta_v_gain(...)`
- `orbit_inclination(...)`

---

## Files Changed

1. `src/Input_File/simulation_parameters.py`
- Added Earth-rotation user inputs.

2. `src/Auxiliary/earth_rotation.py`
- New utility module for azimuth and frame-conversion logic.

3. `src/Simulation/rocket_ascent.py`
- Added Earth-rotation globals (launch azimuth/latitude/rotation speed).
- Added inertial conversion helper.
- Updated interrupt checks to use ECI velocity when rotation is enabled.
- Extended state vector to include latitude when enabled.
- Added latitude derivative in dynamics.
- Computed corrected launch azimuth during run initialization.

4. `src/main.py`
- Added Earth-rotation configuration output block.
- Final orbital element computation now uses ECI-converted velocity when enabled.
- Added inclination output when enabled.

5. `src/Guidance/apollo_guidance.py`
- Updated state unpacking to `state[:5]` in key functions for compatibility with the optional 6-state vector.

---

## Behavior and Compatibility

- `ENABLE_EARTH_ROTATION = False`:
  - Original simulation behavior preserved (5-state vector).

- `ENABLE_EARTH_ROTATION = True`:
  - Latitude is tracked (6-state vector).
  - Azimuth is auto-computed from target inclination and launch latitude.
  - Rotating-frame azimuth correction is applied.
  - Orbital checks and final orbital elements use ECI-converted velocity.

---

## Validation Performed

- Static analysis: no reported errors in modified files.
- Runtime checks:
  - Rotation OFF: simulation runs successfully with 5-state output.
  - Rotation ON: simulation runs successfully with 6-state output.
- Reachability guard:
  - Unreachable case (target inclination below launch latitude in prograde mode) correctly raises `ValueError`.

---

## Current Modeling Scope

Implemented by design:
- Earth rotation effect through azimuth correction and ECI velocity conversion.

Not yet implemented (future extension):
- Coriolis and centrifugal terms directly in equations of motion.
- Longitude/time-of-day launch window coupling in 3D orbital plane targeting.
