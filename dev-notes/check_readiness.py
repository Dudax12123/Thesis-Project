"""Quick readiness check for the indirect PMP implementation."""
import sys, warnings
sys.path.insert(0, 'Tese/src')

print('=== DEPENDENCY CHECK ===')
import numpy as np; print(f'numpy      {np.__version__}  OK')
import scipy;       print(f'scipy      {scipy.__version__}  OK')
import matplotlib;  print(f'matplotlib {matplotlib.__version__}  OK')

try:
    import pygmo as pg
    print(f'pygmo      {pg.__version__}  OK')
except ImportError:
    print('pygmo      NOT installed  --> scipy.differential_evolution fallback will be used')

print()
print('=== MODULE IMPORTS ===')
from Guidance.indirect_pmp_guidance import pmp_control_law, costate_derivatives, compute_hamiltonian
print('indirect_pmp_guidance       OK')

from Input_File import simulation_parameters as sp
assert hasattr(sp, 'PSO_N_PARTICLES') and hasattr(sp, 'PSO_LB') and hasattr(sp, 'PENALTY_W_ALTITUDE')
print('simulation_parameters       OK  (PSO params present)')

from Simulation import indirect_pso_solver as sol
print('indirect_pso_solver         OK')

from Simulation import rocket_ascent as ra
assert callable(ra.run_stage1)
print('rocket_ascent.run_stage1    OK')

print()
print('=== UNIT TESTS ===')
from Auxiliary import constants as c

lv, lg, v = -0.3, 0.7, 6000.0
a = pmp_control_law(lv, lg, v)
dHda = 10.0 * (-lv * np.sin(a) + (lg / v) * np.cos(a))
assert abs(dHda) < 1e-9, f'dH/da = {dHda}'
print('PMP dH/dalpha=0             OK')

dl = costate_derivatives(c.R_EARTH + 200e3, 7000., 0.05, 1e5, 10000., 0.1, -0.2, 0.3, a)
assert all(np.isfinite(d) for d in dl)
print('Costate derivatives finite  OK')

print()
print('=== SINGLE TRAJECTORY EVALUATION ===')
sp.EVENTS_PRINT = False
sp.INTERRUPTS_PRINT = False

with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    result = sol.run_indirect_trajectory(0.1, -0.5, 0.3, 0.0, 80.0, 0.0, 1.555)

assert not result['crashed']
h_f = (result['state_final'][1] - c.R_EARTH) / 1e3
print(f'Trajectory eval:  h={h_f:.0f} km, crashed={result["crashed"]}  OK')

J = sol.compute_augmented_objective(result)
assert np.isfinite(J) and J < 1e18
print(f'Augmented obj J_prime={J:.1f}  OK')

print()
print('=== MINI-PSO (5 particles x 2 generations) ===')
sp.PSO_N_PARTICLES = 5
sp.PSO_MAX_GENERATIONS = 2

with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    best_x, best_J = sol.run_pso_optimization(verbose=False)

assert best_J < 1e18
print(f'Mini-PSO  J_prime={best_J:.1f}  OK')

print()
print('=== ALL CHECKS PASSED ===')
print('Ready to run:  set GUIDANCE_MODE = "indirect_pmp" in simulation_parameters.py')
try:
    import pygmo
    print('Optimizer: PyGMO PSO (paper-exact algorithm)')
except ImportError:
    print('Optimizer: scipy.differential_evolution (pygmo fallback)')
