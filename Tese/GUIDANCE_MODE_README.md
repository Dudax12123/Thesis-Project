# Guidance Mode Selection Guide

## Overview

The coasting single burn trajectory optimization now supports two different guidance strategies that can be selected by the user.

## Configuration

Edit the file: `src/Input_File/simulation_parameters.py`

Look for the parameter:
```python
ENABLE_POLYNOMIAL_GUIDANCE = True  # or False
```

## Guidance Modes

### Mode 1: Pure Gravity Turn (Traditional)
**Setting:** `ENABLE_POLYNOMIAL_GUIDANCE = False`

**Description:**
- Traditional gravity turn all the way from launch to orbit
- Initial kick maneuver to start the pitchover (7.5 - 52 seconds)
- Zero angle of attack throughout the rest of the flight
- Trajectory shaped primarily by gravity and initial kick angle

**Characteristics:**
- Simpler guidance law
- Relies on ballistic trajectory after initial kick
- Optimal kick angle: ~-3.29°
- Total propellant used: ~76,495 kg
- Payload capacity: ~16,175 kg

### Mode 2: Gravity Turn + Polynomial Guidance (Advanced)
**Setting:** `ENABLE_POLYNOMIAL_GUIDANCE = True`

**Description:**
- Initial kick maneuver (7.5 - 52 seconds)
- Polynomial explicit guidance activates after atmosphere exit (>65 km altitude)
- Actively steers the rocket to optimize trajectory to target orbit
- Guidance coefficients updated every 0.1 seconds based on current state

**Characteristics:**
- More sophisticated guidance law
- Active trajectory shaping in vacuum
- Optimal kick angle: ~-3.66°
- Total propellant used: ~76,017 kg (**saves ~478 kg!**)
- Payload capacity: ~16,653 kg (**+478 kg more payload!**)
- Better orbital accuracy

## Additional Polynomial Guidance Parameters

When `ENABLE_POLYNOMIAL_GUIDANCE = True`, you can also adjust:

```python
POLY_GUIDANCE_ORDER = 3              # Order of polynomial (1, 2, 3, etc.)
GUIDANCE_UPDATE_RATE = 0.1           # How often to update coefficients [s]
```

## Performance Comparison

| Metric | Pure Gravity Turn | Polynomial Guidance | Improvement |
|--------|------------------|---------------------|-------------|
| Optimal Kick Angle | -3.29° | -3.66° | More aggressive |
| Propellant Used | 76,495 kg | 76,017 kg | **-478 kg** |
| Payload Capacity | 16,175 kg | 16,653 kg | **+478 kg** |
| Guidance Complexity | Simple | Advanced | - |
| Orbital Accuracy | Good | Better | Improved |

## When to Use Each Mode

### Use Pure Gravity Turn When:
- You want a simpler, more traditional approach
- You're validating against historical data or classical methods
- Computational resources are limited
- You prefer passive ballistic trajectories

### Use Polynomial Guidance When:
- You want to maximize payload capacity
- You need better orbital insertion accuracy
- You want to implement modern guidance techniques
- You're exploring advanced trajectory optimization

## How It Works

### Pure Gravity Turn:
1. Vertical launch (α = 0°)
2. Initial pitchover kick (α varies linearly)
3. Ballistic flight (α = 0° throughout)
4. Coast to target altitude
5. Circularization burn

### Polynomial Guidance:
1. Vertical launch (α = 0°)
2. Initial pitchover kick (α varies linearly)
3. Atmosphere exit detected at 65 km
4. **Polynomial guidance activates** (α computed from polynomial based on time-to-go)
5. Active steering continues **while engines are burning**
6. **Engine cutoff - guidance stops** (can't steer without thrust)
7. **Coasting phase** (α = 0°, ballistic trajectory)
8. Circularization burn at apoapsis

## Technical Details

The polynomial guidance uses a 3rd-order polynomial:
```
α(τ) = a₀ + a₁·τ + a₂·τ² + a₃·τ³
```

Where:
- α = angle of attack (thrust angle relative to velocity)
- τ = normalized time-to-go
- a₀, a₁, a₂, a₃ = coefficients computed based on current and target states

The coefficients are recomputed periodically to account for:
- Current flight path angle
- Target orbital altitude
- Terminal boundary conditions (horizontal flight for circular orbit)
- Remaining time to target

## Running the Simulation

Simply run:
```bash
python src/main_coasting_single_burn.py
```

The program will automatically:
1. Display which guidance mode is active
2. Optimize the kick angle for that mode
3. Run the full simulation
4. Display results and generate plots

## Example Output

```
============================================================
COASTING SINGLE BURN TRAJECTORY OPTIMIZATION
============================================================
Guidance Mode: Gravity Turn + Polynomial Guidance
  - Initial kick maneuver followed by polynomial guidance
  - Guidance activates after atmosphere exit (>65 km)
============================================================
```

or

```
============================================================
COASTING SINGLE BURN TRAJECTORY OPTIMIZATION
============================================================
Guidance Mode: Pure Gravity Turn
  - Traditional gravity turn all the way
  - Zero angle of attack after initial kick
============================================================
```
