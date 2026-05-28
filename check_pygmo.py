"""Full readiness check using the pygmo-env Python."""
import sys, warnings, os

os.chdir(r'C:\Users\eduar\Desktop\Tese\Thesis-Project')
sys.path.insert(0, 'Tese/src')

print('=== DEPENDENCY CHECK ===')
import numpy as np;   print(f'numpy      {np.__version__}  OK')
import scipy;         print(f'scipy      {scipy.__version__}  OK')
import matplotlib;    print(f'matplotlib {matplotlib.__version__}  OK')
import pygmo as pg;   print(f'pygmo      {pg.__version__}  OK')

print()
print('=== PYGMO PSO SANITY (rosenbrock) ===')
prob = pg.problem(pg.rosenbrock(dim=2))
algo = pg.algorithm(pg.pso(gen=20, omega=0.7298, eta1=2.05, eta2=2.05, max_vel=0.5))
pop  = pg.population(prob, size=20)
pop  = algo.evolve(pop)
print(f'Rosenbrock champion: {pop.champion_f[0]:.4f}  (should be near 0)  OK')

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
print('PMP  dH/dalpha=0            OK')

dl = costate_derivatives(c.R_EARTH + 200e3, 7000., 0.05, 1e5, 10000., 0.1, -0.2, 0.3, a)
assert all(np.isfinite(d) for d in dl)
print('Costate derivatives finite  OK')

H = compute_hamiltonian(c.R_EARTH + 200e3, 7000., 0.05, 1e5, 10000., a, 0.1, -0.2, 0.3)
assert np.isfinite(H)
print(f'Hamiltonian = {H:.4f}      OK')

print()
print('=== SINGLE TRAJECTORY EVALUATION ===')
sp.EVENTS_PRINT = False
sp.INTERRUPTS_PRINT = False

with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    result = sol.run_indirect_trajectory(0.1, -0.5, 0.3, 0.0, 80.0, 0.0, 1.555)

assert not result['crashed']
h_f  = (result['state_final'][1] - c.R_EARTH) / 1e3
J    = sol.compute_augmented_objective(result)
assert np.isfinite(J) and J < 1e18
print(f'Trajectory eval:  h={h_f:.0f} km,  J_prime={J:.1f}  OK')

print()
print('=== MINI PSO WITH PYGMO (5 particles x 5 gen) ===')
sp.PSO_N_PARTICLES   = 5
sp.PSO_MAX_GENERATIONS = 5

with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    best_x, best_J = sol.run_pso_optimization(verbose=False)

assert best_J < 1e18
print(f'Mini-PSO  J_prime={best_J:.1f}  OK')

print()
print('=== ALL CHECKS PASSED ===')
print()
print('To run the full optimisation:')
print('  1. Set GUIDANCE_MODE = "indirect_pmp" in simulation_parameters.py')
print('  2. Run with:  C:\\Users\\eduar\\miniforge3\\envs\\pygmo-env\\python.exe Tese/src/main.py')
print(f'     Python: {sys.version.split()[0]},  pygmo: {pg.__version__}')
