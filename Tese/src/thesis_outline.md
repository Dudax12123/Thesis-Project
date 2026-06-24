# Comprehensive Outline of What Is Done in the Thesis

**Source file:** `Eduardo_Thesis.pdf`  
**Thesis title:** *Concurrent Simulation of Rocket Ascent Trajectories*  
**Author:** Eduardo De Almeida Helena

Based on the PDF, the thesis is about building and using a configurable numerical simulator to compare rocket ascent guidance strategies for reaching orbit. The current document already has a strong Introduction, Background, and Methodology, while the Results, Conclusions, Abstract, Resumo, Software section, and some subsections are still placeholders or partially drafted.

---

## Thesis Title and Central Topic

The thesis develops a numerical tool for simulating rocket ascent trajectories and comparing different explicit guidance laws under a common modelling framework. The main application is a Falcon 9-class launch vehicle targeting a circular low Earth orbit.

---

# 1. Introduction

## 1.1 Objective and Motivation

The thesis begins by defining the main objective: to create a configurable simulation tool for rocket ascent trajectory analysis. The tool is intended to compare several explicit guidance laws and an optimized reference trajectory while considering a spherical Earth, a fixed vertical-plane trajectory model, and atmospheric effects.

The motivation is linked to the growth of commercial launch services, the increasing importance of cost-efficient and flexible launch operations, and the need for fast preliminary-design tools that can evaluate ascent performance, losses, staging, coasting, and payload capability.

## 1.2 Launch Vehicle and Ascent Fundamentals

This section introduces the basic physics and architecture of launch vehicles. It explains:

- what a space launch vehicle is;
- the purpose of stages, propulsion, fairings, avionics, telemetry, and safety systems;
- the rocket equation and its relation to mass ratio and ideal velocity increment;
- the difference between ideal ∆V and real ascent performance;
- the main loss mechanisms: gravity losses, drag losses, thrust losses, and steering losses;
- the benefit of Earth’s rotation for prograde launches;
- launch-site and azimuth constraints;
- staging architectures;
- the typical ascent sequence: lift-off, vertical rise, pitch/yaw/roll manoeuvres, max-q, staging, closed-loop ascent, SECO, coast, and circularization.

The introduction also discusses how different orbit types may require direct insertion, coast phases, or transfer-orbit strategies.

## 1.3 Literature Review

The thesis reviews several ascent guidance approaches:

- **Polynomial guidance**, including its use in Apollo-related applications and later mission studies.
- **Gravity-turn guidance**, which is simple and near fuel-optimal under certain assumptions.
- **Iterative Guidance Mode**, historically used on Saturn vehicles.
- **Powered Explicit Guidance**, especially associated with the Space Shuttle.
- PEG-inspired and higher-order refinements.
- Comparative studies between IGM and PEG.
- More recent approximate minimum-time pitch/yaw guidance laws for lunar ascent.

The literature review positions the thesis as a comparative work rather than as a development of a completely new guidance law.

## 1.4 Thesis Originality and Overview

The thesis states that its originality lies in placing several established ascent guidance laws inside one common simulation and evaluation framework. The goal is to quantify how the choice of guidance law affects mission-level outcomes such as required ∆V, losses, propellant use, payload capability, and sensitivity to modelling assumptions.

---

# 2. Background

## 2.1 Equations of Motion

The thesis introduces a simplified 3-degree-of-freedom ascent model. The motion is restricted to a vertical plane, with state variables such as velocity, flight-path angle, downrange distance, radius/altitude, and mass.

The model includes:

- thrust acceleration;
- drag and lift;
- gravity;
- flight-path-angle dynamics;
- mass depletion;
- angle of attack as the difference between pitch angle and flight-path angle.

The assumptions are clearly stated: rigid vehicle, prescribed attitude tracking, no full rotational dynamics, and simplified planar motion.

## 2.2 Reference Frames

The thesis distinguishes between:

- a rotating Earth-fixed frame, useful for atmospheric-relative velocity and launch-site-relative motion;
- an inertial frame, required for orbital mechanics and orbital-element calculations.

The conversion between rotating-frame and inertial-frame velocity is introduced, including the contribution from Earth’s rotation.

## 2.3 Atmospheric Model

This section is still incomplete in the current PDF. It contains a placeholder indicating that atmospheric models still need to be discussed.

However, later in the methodology, the implemented atmosphere is defined as an exponential density model.

## 2.4 External Forces and Moments

The thesis presents the main force models used in ascent simulation.

### Gravity

Gravity is modelled using an inverse-square relationship with altitude.

### Thrust

Thrust is described using mass flow rate, exhaust velocity, pressure correction, and specific impulse. A mass depletion model is also introduced.

### Thrust Vector Control

The thesis explains how gimballed thrust can generate lateral force and torque. It decomposes thrust into tangential and normal components relative to the velocity direction.

### Aerodynamic Forces

Lift and drag are introduced, including their dependence on dynamic pressure, velocity, atmospheric density, reference area, and aerodynamic coefficients. The concept of max-q is also discussed.

## 2.5 Performance: Losses and Gains

The thesis formulates ascent ∆V as a budget:

- ideal rocket-equation ∆V;
- steering loss;
- drag loss;
- gravity loss;
- gain from Earth rotation and trajectory shaping.

The thesis emphasizes the trade-off between drag losses and gravity losses, which strongly shapes ascent trajectory design.

## 2.6 Guidance and Steering Laws

The thesis defines guidance laws as the rules used to command vehicle attitude, pitch angle, flight-path angle, or angle of attack.

It distinguishes between:

- **open-loop guidance**, where the vehicle follows a precomputed attitude profile;
- **closed-loop guidance**, where the vehicle uses real-time state feedback to update steering commands.

## 2.7 Atmospheric-Arc Steering

The thesis describes atmospheric steering methods, including:

- initial pitch manoeuvre;
- gravity turn;
- constant pitch rate;
- constant flight-path-angle rate.

The focus is on controlling the early ascent while keeping angle of attack small to limit aerodynamic loads.

## 2.8 Exoatmospheric Guidance

After leaving the dense atmosphere, guidance shifts from load management to accurate orbital insertion. The thesis explains that closed-loop explicit guidance becomes more appropriate because aerodynamic forces are negligible and real-time correction is needed.

## 2.9 Explicit Guidance Schemes

Several explicit guidance schemes are developed theoretically.

### Polynomial Guidance

The acceleration components are assumed to vary linearly with time. Coefficients are computed from boundary conditions on position and velocity. The thesis also discusses the numerical issue that coefficients become singular as time-to-go approaches zero, requiring coefficient freezing.

### Iterative Guidance Mode

IGM is presented as a method that assumes thrust-direction angles vary approximately linearly with time under simplified flat-Earth, uniform-gravity assumptions. The method solves for guidance parameters using terminal position and velocity constraints.

### Powered Explicit Guidance

PEG is introduced as a linear-tangent-law-based guidance method. The thesis presents its assumptions, thrust-acceleration integrals, predictor-corrector structure, time-to-go iteration, velocity-to-go update, and steering-parameter computation.

## 2.10 Optimal Trajectory Design

The thesis introduces optimal control theory as the mathematical basis for trajectory optimization. It presents:

- Bolza-form optimal control problem formulation;
- dynamic constraints;
- boundary constraints;
- path constraints;
- direct and indirect solution methods;
- Pontryagin’s Maximum Principle;
- Hamiltonian formulation;
- costate equations;
- stationarity and transversality conditions;
- the two-point boundary value problem.

This section provides theoretical context for comparing guidance laws with optimized or near-optimized trajectories.

---

# 3. Methodology

This is the core implementation chapter.

## 3.1 Simulator Implementation

The thesis describes a numerical ascent simulator organized into three mission phases.

### Phase 1: First-Stage Powered Ascent

The vehicle starts at lift-off, rises vertically, performs an initial pitchover kick, flies through the atmosphere, and continues until first-stage propellant depletion or MECO.

### Phase 2: Second-Stage Powered Ascent with Guidance

After stage separation and a coast interval, the second stage ignites. One of the available guidance laws controls the trajectory. SECO occurs when the current unpowered trajectory would reach the target apogee altitude.

### Phase 3: Ballistic Coast and Circularization

After SECO, the vehicle coasts ballistically to apogee. At apogee, an impulsive circularization burn is computed to achieve the target circular orbit.

## 3.2 State Vector

The simulator uses a base state vector containing:

- downrange distance;
- geocentric radius;
- velocity magnitude;
- flight-path angle;
- mass.

When Earth rotation is enabled, latitude is added. When heading tracking is enabled, heading/azimuth is also added.

## 3.3 Numerical Integration

The simulator uses `scipy.integrate.solve_ivp` with an explicit Runge–Kutta 4(5) Dormand–Prince scheme, adaptive step size, tight tolerance, and event functions.

Events include:

- MECO;
- stage separation;
- atmosphere exit;
- target-apogee matching;
- ground collision safeguards.

## 3.4 Reference-Frame Strategy

The simulator uses a mixed-frame strategy:

- rotating-frame dynamics for ascent and atmospheric effects;
- inertial-frame calculations for orbital elements and apogee/perigee evaluation;
- local ENU frame for Coriolis and centrifugal pseudo-force calculations.

The thesis also explains when the simulation switches to inertial dynamics after SECO.

## 3.5 Launch Azimuth Calculation

The methodology derives launch azimuth for direct ascent from a rotating spherical Earth. It computes:

- inertial launch azimuth from target inclination and launch latitude;
- Earth-rotation contribution;
- ground-relative azimuth used by the guidance law;
- feasibility condition for direct launch;
- achieved inclination at the end of the simulation.

## 3.6 Heading Propagation

When heading tracking is enabled, heading is propagated using spherical geometry and optional cross-heading pseudo-force corrections. The final achieved inclination is computed from the final inertial velocity components.

## 3.7 Equations of Motion Implemented

The implemented equations include:

- downrange rate;
- radius/altitude rate;
- velocity rate;
- flight-path-angle rate;
- mass depletion.

In the rotating-frame case, Coriolis and centrifugal terms are added to the velocity and flight-path-angle equations.

## 3.8 Environmental Models

The implemented environment uses:

- central inverse-square gravity;
- spherical Earth;
- exponential atmospheric density;
- dynamic pressure calculation.

Atmosphere exit can be triggered either by altitude threshold or dynamic-pressure threshold.

## 3.9 External Force Models

The thesis implements:

- constant drag coefficient;
- fixed reference area;
- drag force from dynamic pressure;
- no lift in the presented analyses;
- stage-dependent thrust and specific impulse;
- mass depletion from thrust and specific impulse;
- Coriolis and centrifugal pseudo-forces when Earth rotation is enabled.

## 3.10 Guidance Laws Implemented

Five guidance modes are available.

### 1. Gravity Turn

After the initial kick, the angle of attack is set to zero. The trajectory evolves naturally under gravity.

### 2. Simple Polynomial Guidance

A linear steering law drives the flight-path angle toward zero for circular-orbit insertion. Coefficients are recomputed periodically.

### 3. Linear Tangent Steering

The tangent of the total inclination angle is prescribed as a linear function of time-to-go. This provides smooth steering toward horizontal flight.

### 4. Bilinear Tangent Steering

This generalizes linear tangent steering using a ratio of two linear functions. It allows more control over the terminal approach and can produce smoother insertion.

### 5. Apollo Polynomial Guidance

This is the most advanced implemented mode. It commands acceleration in downrange and vertical channels using polynomial coefficients chosen to satisfy target position and velocity constraints. It also includes coefficient freezing near small time-to-go to avoid numerical instability.

## 3.11 Launcher Configuration

The baseline launcher is representative of **SpaceX Falcon 9**. The thesis gives stage parameters including:

- first-stage and second-stage specific impulse;
- thrust;
- propellant type;
- propellant mass;
- dry mass;
- payload capacity;
- diameter.

The baseline mission uses a 500 km circular orbit.

## 3.12 Mission Geometry

The baseline mission is:

- target altitude: 500 km;
- target inclination: 51.6°;
- launch latitude: 28.5°;
- launch longitude: −80.5°.

This is representative of a launch from Kennedy Space Center to an ISS-like inclination.

## 3.13 Initial Kick-Angle Optimization

The thesis optimizes the initial kick angle used in the pitchover manoeuvre. The outer optimization loop varies only this kick angle, while the simulator determines the cutoff time through the target-apogee event.

The objective is to minimize total propellant required for:

- powered ascent;
- circularization at apogee.

The cost function is:

```text
total propellant = ascent propellant + circularization propellant
```

The thesis argues that minimizing this propellant requirement is equivalent, under the simplifying assumptions, to maximizing deliverable payload.

The optimization uses a brute-force grid search over the admissible kick-angle interval, which is robust for a low-dimensional problem with event-driven discontinuities.

## 3.14 Software Section

The current PDF includes a placeholder for explaining how the user handles the software. This part still needs to be completed.

---

# 4. Results and Discussion

This chapter is currently mostly incomplete. It contains placeholders and planning notes, but it clearly indicates the intended structure.

The planned results include:

## 4.1 Problem Description

A description of the baseline simulation case.

## 4.2 Use Cases

The thesis plans to select one or more launch vehicles, possibly one large and one small, and one or more target orbits. The notes indicate that vehicle data should come from manufacturers, literature, or direct estimation from available videos.

## 4.3 No-Atmosphere Cases

The thesis plans to compare each guidance method without atmosphere.

## 4.4 With-Atmosphere Cases

The thesis plans to compare each guidance method with atmospheric effects included.

## 4.5 Effects of Spherical Earth and Earth Rotation

The thesis plans to compare, likely using gravity turn as a reference case, the influence of spherical Earth modelling and Earth rotation.

## 4.6 Losses

The thesis plans to discuss:

- gravity losses;
- drag losses;
- steering losses.

## 4.7 Results Discussion

The chapter currently contains a note to add discussion subsections to each results section.

---

# 5. Conclusions

This chapter is also still mostly a placeholder.

## 5.1 Achievements

The thesis plans to summarize the major achievements of the work.

## 5.2 Future Work

The thesis plans to list future improvements and extensions.

---

# What the Thesis Has Accomplished So Far

At its current stage, the thesis has already done the following:

1. Defined the research objective: comparing rocket ascent guidance laws inside a common numerical framework.
2. Reviewed launch vehicle fundamentals, ascent phases, staging, losses, and launch constraints.
3. Reviewed major explicit guidance laws, including gravity turn, polynomial guidance, IGM, and PEG.
4. Introduced the optimal-control background needed to understand trajectory optimization.
5. Developed a detailed simulator methodology, including state definition, numerical integration, event handling, environmental models, force models, Earth rotation effects, and orbital calculations.
6. Implemented or at least specified five ascent guidance strategies.
7. Defined a Falcon 9-class baseline launcher and a 500 km, 51.6° target orbit.
8. Defined an optimization procedure for the initial kick angle, minimizing total propellant required for ascent and circularization.

---

# What Remains Incomplete in the Current PDF

Several parts are still unfinished:

- Portuguese abstract and English abstract are placeholders.
- Atmospheric models section in Chapter 2 is incomplete.
- Software usage section is incomplete.
- Launcher configuration section contains a note to discuss other launchers.
- Mission geometry section contains a note saying the user should choose mission parameters.
- Results chapter is mostly a planned structure, not final results.
- Conclusions chapter is still a placeholder.
- Some internal notes remain in Portuguese and should be removed or converted into final thesis text.

Overall, the thesis is currently strongest in the theoretical background and methodology. The main remaining work is to run and present the simulation results, compare the guidance laws quantitatively, discuss the losses, and write the final conclusions.
