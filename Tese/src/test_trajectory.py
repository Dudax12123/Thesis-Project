""" ===============================================
    SIMPLE TEST SCRIPT
    
    Quick test to verify the rocket ascent simulation
    runs without errors.
=============================================== """

import numpy as np
import rocket_ascent as ra
import simulation_parameters as sim_params


def test_single_trajectory():
    """
    Test a single trajectory run with a fixed kick angle.
    """
    print("="*60)
    print("TESTING SINGLE TRAJECTORY")
    print("="*60 + "\n")
    
    # Use the initial guess kick angle
    test_kick_angle = sim_params.INITIAL_KICK_ANGLE
    
    print(f"Testing with kick angle: {np.rad2deg(test_kick_angle):.2f} degrees\n")
    
    # Set to optimization mode (faster)
    ra.SINGLE_BURN_FULL_SIMULATION = False
    
    try:
        time, data, alt_stopped, delta_v, m_propellant_total = ra.run(test_kick_angle)
        
        print("Test Results:")
        print(f"\t* Simulation time:\t\t{time[-1]:.2f} seconds")
        print(f"\t* Final altitude:\t\t{(data[1, -1] - 6378e3)/1000:.2f} km")
        print(f"\t* Final velocity:\t\t{data[2, -1]:.2f} m/s")
        
        if alt_stopped is not None:
            print(f"\t* Stop altitude:\t\t{alt_stopped/1000:.2f} km")
            print(f"\t* Delta-v required:\t\t{delta_v:.2f} m/s")
            print(f"\t* Propellant used:\t\t{m_propellant_total:.2f} kg")
        else:
            print("\t* Did not reach optimal coast trajectory")
        
        print("\n" + "="*60)
        print("TEST PASSED!")
        print("="*60 + "\n")
        
        return True
        
    except Exception as e:
        print(f"\nERROR: {e}")
        print("\n" + "="*60)
        print("TEST FAILED!")
        print("="*60 + "\n")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_single_trajectory()
    if success:
        print("You can now run 'main_coasting_single_burn.py' for full optimization.")
