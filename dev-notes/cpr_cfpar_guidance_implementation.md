# Implementation Notes: Constant Pitch Rate (CPR) and Constant Flight-Path-Angle Rate (CFPAR)

> **Status (2026-06-15):** CPR-kinematic (Section 5.1) is implemented as
> `GUIDANCE_MODE = "cpr"` in `Tese/src/Guidance/cpr_guidance.py`. CPR-analytic
> (Section 5.2) and all of CFPAR (Section 6) are **not implemented** — this
> document remains their specification. Note also that `"cpr"` currently has a
> known crash in Stage-1 event handling with Earth rotation enabled (see
> project memory `cpr-stage1-brentq-crash`).

This document specifies how to implement two ascent guidance modes in the simulator:

1. **CPR — Constant Pitch Rate**
2. **CFPAR — Constant Flight-Path-Angle Rate**

The formulas below are the ones provided by the user from the reference material. The goal is to make the simulator expose only the true guidance parameters to the user, while deriving all vehicle/environment quantities from the current simulation state.

---

## 1. Coordinate and Unit Conventions

Use these conventions consistently:

- All angles are in **radians**.
- All angular rates are in **radians per second**.
- Time `t` is measured from the **start of the guidance segment**, not from launch.
- Vehicle mass flow should use the sign convention:

```text
mdot = dm/dt
```

Therefore, during powered flight:

```text
mdot < 0
```

If the engine model stores propellant mass flow as a positive number, for example `mdot_prop > 0`, convert it before using these formulas:

```python
mdot = -mdot_prop
```

Basic angle relation:

```text
alpha = theta - gamma
```

where:

- `alpha` = angle of attack/steering angle
- `theta` = pitch angle / vehicle attitude angle
- `gamma` = flight-path angle

---

## 2. Common Variables

| Symbol | Suggested variable name | Meaning | Source |
|---|---:|---|---|
| \(t\) | `t_segment` | Time since start of CPR/CFPAR segment | Simulator |
| \(\alpha(t)\) | `alpha_cmd` | Commanded angle of attack | Guidance output |
| \(\theta(t)\) | `theta_cmd` | Commanded pitch angle | Guidance output/state |
| \(\gamma(t)\) | `gamma` or `gamma_cmd` | Flight-path angle | State or guidance output |
| \(\dot{\theta}\) | `theta_dot_cmd` | Constant pitch rate | User-defined for CPR |
| \(\dot{\gamma}\) | `gamma_dot_cmd` | Constant flight-path-angle rate | User-defined for CFPAR |
| \(v_0\) | `v0` | Speed at start of segment | Captured from simulator state |
| \(m_0\) | `m0` | Mass at start of segment | Captured from simulator state |
| \(m(t)\) | `m` | Current vehicle mass | Simulator mass model |
| \(\dot{m}\) | `mdot` | Current/segment mass rate, negative during burn | Engine model |
| \(F_*\) | `F_star` | Effective thrust magnitude | Engine/throttle model |
| \(a_*\) | `a_star` | Effective thrust acceleration | Usually `F_star / m` |
| \(g\) | `g` | Local gravitational acceleration magnitude | Environment model |
| \(\alpha_0\) | `alpha0` | Angle of attack at start of segment | Captured from state |

---

## 3. What the User Should Define

The user should not define low-level state variables like mass, thrust acceleration, or initial velocity manually unless they are running an analytic test case. Those values should be captured from the simulator.

### 3.1 User Inputs for CPR

For **Constant Pitch Rate**, the user should define either:

```text
theta_dot_cmd
duration
```

or:

```text
theta_final
duration
```

If `theta_final` is given instead of `theta_dot_cmd`, compute:

```python
theta_dot_cmd = (theta_final - theta0) / duration
```
The duration can be given by the user in the simulation_parameters.py file or can be calculated using the apollo time to go estimation.
Every user choice or input shoul be in the simulation_parameters.py file.

### 3.2 User Inputs for CFPAR

For **Constant Flight-Path-Angle Rate**, the user should define either:

```text
gamma_dot_cmd
duration
```

or:

```text
gamma_final
duration
```

If `gamma_final` is given instead of `gamma_dot_cmd`, compute:

```python
gamma_dot_cmd = (gamma_final - gamma0) / duration
```
The duration can be given by the user in the simulation_parameters.py file or can be calculated using the apollo time to go estimation.
Every user choice or input shoul be in the simulation_parameters.py file.
---

## 4. Segment Initialization

The CPR and CFPAR should start right after the vertical flight so there is no need for a kick maneuver.

At the start of a CPR or CFPAR segment, capture the initial state:

---

# 5. CPR: Constant Pitch Rate

## 5.1 Concept

CPR commands the vehicle pitch angle to change at a constant rate:

```text
theta(t) = theta0 + theta_dot_cmd * t
```

The direct kinematic implementation is:

```python
theta_cmd = theta0 + theta_dot_cmd * t_segment
alpha_cmd = theta_cmd - state.gamma
```

This is the simplest and most robust implementation if the simulator already propagates the true flight-path angle.

---

## 5.2 Analytic CPR Formula

The provided CPR formula is:

```math
\alpha(t)=
\frac{a_*/2-g}{a_*-g}
\left(\dot{\theta}t+\alpha_0\right)
+
\frac{1}{2}
\frac{v_0\dot{\gamma}}{a_*-g}
```

Suggested implementation:

```python
def alpha_cpr_analytic(
    t_segment: float,
    theta_dot_cmd: float,
    alpha0: float,
    v0: float,
    gamma_dot: float,
    F_star: float,
    m: float,
    g: float,
    eps: float = 1e-9,
) -> float:
    """
    Compute commanded angle of attack using the analytic CPR expression.

    Units:
        t_segment: s
        theta_dot_cmd: rad/s
        alpha0: rad
        v0: m/s
        gamma_dot: rad/s
        F_star: N
        m: kg
        g: m/s^2

    Returns:
        alpha_cmd in radians
    """

    a_star = F_star / m
    denom = a_star - g

    if abs(denom) < eps:
        raise ValueError("CPR analytic formula singular because a_star is close to g.")

    alpha = ((0.5 * a_star - g) / denom) * (theta_dot_cmd * t_segment + alpha0) \
            + 0.5 * v0 * gamma_dot / denom

    return alpha
```

Then:

```python
theta_cmd = state.gamma + alpha_cmd
```

---

## 5.3 Which CPR Variant to Use?

Implement both options if possible:

```python
CPRMode.KINEMATIC
CPRMode.ANALYTIC
```

Recommended default:

```python
CPRMode.KINEMATIC
```

Reason: the kinematic form directly enforces constant pitch rate and uses the simulated `gamma`.

The analytic form is useful if the simulator wants to command angle of attack from a closed-form approximation.

# 6. CFPAR: Constant Flight-Path-Angle Rate

## 6.1 Concept

CFPAR commands the flight-path angle to change at a constant rate:

```text
gamma(t) = gamma0 + gamma_dot_cmd * t
```

The guidance law computes the angle of attack needed to approximately achieve that flight-path-angle profile.

---

## 6.2 Analytic CFPAR Formula

The provided CFPAR formula is:

```math
\sin\alpha(t)
=
\dot{\gamma}
\left[
\frac{m}{\dot{m}}
\ln\left(1+\frac{\dot{m}}{m_0}t\right)
+
\frac{mv_0}{F_*}
\right]
-
2\frac{mg}{F_*}\sin(\dot{\gamma}t)
```

Therefore:

```math
\alpha(t) = \arcsin(\text{right-hand side})
```

Suggested implementation:

```python
import math

def alpha_cfpar_analytic(
    t_segment: float,
    gamma_dot_cmd: float,
    v0: float,
    m0: float,
    m: float,
    mdot: float,
    F_star: float,
    g: float,
    eps: float = 1e-9,
    clamp: bool = True,
) -> float:
    """
    Compute commanded angle of attack using the analytic CFPAR expression.

    Sign convention:
        mdot = dm/dt
        mdot < 0 during powered flight

    Units:
        t_segment: s
        gamma_dot_cmd: rad/s
        v0: m/s
        m0: kg
        m: kg
        mdot: kg/s
        F_star: N
        g: m/s^2

    Returns:
        alpha_cmd in radians
    """

    if abs(mdot) < eps:
        raise ValueError("CFPAR formula requires nonzero mdot.")

    if abs(F_star) < eps:
        raise ValueError("CFPAR formula requires nonzero thrust F_star.")

    log_argument = 1.0 + (mdot / m0) * t_segment

    if log_argument <= 0.0:
        raise ValueError("Invalid CFPAR log argument. Segment may extend past burnout.")

    log_term = (m / mdot) * math.log(log_argument)

    rhs = gamma_dot_cmd * (log_term + (m * v0 / F_star)) \
          - 2.0 * (m * g / F_star) * math.sin(gamma_dot_cmd * t_segment)

    if clamp:
        rhs = max(-1.0, min(1.0, rhs))
    else:
        if rhs < -1.0 or rhs > 1.0:
            raise ValueError("CFPAR arcsin input outside [-1, 1].")

    alpha_cmd = math.asin(rhs)

    return alpha_cmd
```

Then compute:

```python
gamma_cmd = gamma0 + gamma_dot_cmd * t_segment
theta_cmd = gamma_cmd + alpha_cmd
```