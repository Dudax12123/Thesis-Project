# Falcon 9 webcast telemetry

Three expendable Falcon 9 Full Thrust flights to geostationary transfer orbit (2017), the nearest
real flights to the simulator's first stage burned to depletion. Drawn against the simulator by
`Plots/results_figures/validation_falcon9.py`.

| Folder | Mission |
|---|---|
| `intelsat_35e/` | Intelsat 35e |
| `inmarsat_5_f4/` | Inmarsat-5 F4 |
| `echostar_23/` | EchoStar 23 |

**Source.** `analysed.json` and `events.json` from each mission's `JSON/` folder of
https://github.com/shahar603/Telemetry-Data, retrieved 2026-10-09, unmodified. Released into the
public domain (`LICENSE`, the Unlicense). The values were read off the SpaceX webcasts by optical
character recognition (https://github.com/shahar603/SpaceXtract).

**Fields** (`analysed.json`, one sample per second):

- `time` [s] from liftoff.
- `velocity` [m/s] and `altitude` [km]: the two quantities the webcast displays. Measured.
- `velocity_y`, `velocity_x`, `acceleration`, `downrange_distance` [km], `angle` [deg] (flight-path
  angle) and `q` [Pa]: derived by the dataset author from speed and altitude. Not measured.
  `q` uses the author's atmosphere model, not the simulator's.

`events.json` gives event times [s]: max-Q, the throttle-down interval, MECO, SES-1 and, where
shown, SECO-1. `null` means not recorded.

**Frame.** The displayed speed reads 0 m/s on the pad, so it is relative to the Earth's surface:
the simulator's rotating-frame speed `v`.
