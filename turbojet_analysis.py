

"""
Project: 0-D Parametric Thermodynamic Cycle Analysis (Real Cycle Turbojet)
Author: Husam ASLAN
Date: 2025
Description: 
    This tool performs a parametric study on a single-spool turbojet engine.
    It calculates design point performance (Specific Thrust, TSFC) considering
    variable gas properties (Cp, Gamma) and component efficiencies.
"""

import numpy as np
import matplotlib.pyplot as plt

# --- 1. Global Simulation Constants ---
# These parameters are fixed for the scope of this study
R = 287.05          # Gas Constant (J/kg.K)
LHV_FUEL = 43.0e6   # Lower Heating Value of Jet-A (J/kg)

# Flight Conditions (Sea Level Static)
gamma_air = 1.4
T_atm = 288.15
P_atm = 101325.0
Mach = 0.0 

# Component Efficiencies (State-of-the-art assumptions)
ETA_C = 0.85        # Compressor Isentropic Efficiency
ETA_B = 0.98        # Burner Efficiency
P_LOSS_B = 0.05     # Combustor Pressure Loss Factor (5%)
ETA_T = 0.90        # Turbine Isentropic Efficiency
ETA_T_M = 0.99      # Turbine Mechanical Efficiency
ETA_N = 0.98        # Nozzle Adiabatic Efficiency

Pr_range = np.linspace(5, 40, 15) # From 5 to 40. You can change it according to your study
TIT_range = [1200, 1400, 1600, 1800] # Kelvin

# --- 2. Helper Functions (Thermodynamics) ---

def get_air_properties(T):
    """
    Calculates specific heat (Cp) and heat capacity ratio (Gamma) 
    as a function of temperature.
    """
    if T < 200.0:
        T = 200.0

    # Polynomial approximation for Cp (J/kg.K)
    cp = 1005.0 + ((T - 250)**2) / 3500.0

    # Calculate Gamma
    gamma = cp / (cp - R)
    return cp, gamma

# --- 3. Component Models ---

def compressor_real(T_in, P_in, pr, eta_c):
    """ Calculates Compressor outlet state based on polytropic efficiency. """
    P_out = P_in * pr

    # Initial guess using inlet properties
    cp_in, gamma_in = get_air_properties(T_in)
    m = (gamma_in - 1) / gamma_in
    T_out_guess = T_in * (1 + (1/eta_c) * ((pr ** m) - 1))

    # Refinement using average properties
    T_avg = (T_in + T_out_guess) / 2.0
    cp_avg, gamma_avg = get_air_properties(T_avg)
    m_avg = (gamma_avg - 1) / gamma_avg

    T_out_final = T_in * (1 + (1/eta_c) * ((pr ** m_avg) - 1))
    work = cp_avg * (T_out_final - T_in)

    return T_out_final, P_out, work

def combustor(T_in, T_max, P_in, eta_b, pressure_loss_factor):
    """ Calculates fuel-to-air ratio required to reach TIT (T_max). """
    P_out = P_in * (1 - pressure_loss_factor)

    T_avg = (T_in + T_max) / 2.0
    cp_avg, _ = get_air_properties(T_avg)

    # Energy Balance: m_dot_air * cp * dT = m_dot_fuel * LHV * eta
    numerator = cp_avg * (T_max - T_in)
    denominator = (eta_b * LHV_FUEL) - (cp_avg * T_max) # Includes fuel mass enthalpy

    f = numerator / denominator
    if f < 0: f = 0  # Safety check

    return T_max, P_out, f

def turbine(T_in, P_in, W_required, eta_t, eta_mech):
    """ Calculates Turbine outlet state based on work requirement from compressor. """
    # Work balance: Turbine Work = Compressor Work / Mechanical Efficiency
    W_turb_total = W_required / eta_mech

    # Simplify: Use inlet properties for expansion (can be iterated for higher accuracy)
    cp_gas, gamma_gas = get_air_properties(T_in)

    # Calculate T_out based on work extraction
    T_out = T_in - (W_turb_total / cp_gas)

    # Calculate P_out using isentropic efficiency relation
    # T_in - T_out = eta_t * (T_in - T_out_isentropic)
    # Rearranging for Pressure...
    delta_T = T_in - T_out

    # Safety Check: Cannot extract more energy than available
    if delta_T < 0 or T_out < 0:
        return T_in, P_in * 0.1 # Failure state

    exponent = gamma_gas / (gamma_gas - 1)

    # Base term check to avoid complex numbers in power
    base_term = 1 - (delta_T / (eta_t * T_in))
    if base_term <= 0:
        P_out = P_in * 0.01 # Deep choke/Failure
    else:
        P_out = P_in * (base_term ** exponent)

    return T_out, P_out

def nozzle(T_in, P_in, P_atm, eta_n):
    """ Calculates Nozzle exit velocity considering Choked/Unchoked flow. """
    cp_gas, gamma_gas = get_air_properties(T_in)

    # 1. Critical Pressure Ratio (Choking Condition)
    critical_ratio = ((gamma_gas + 1) / 2) ** (gamma_gas / (gamma_gas - 1))
    P_critical = P_in / critical_ratio

    # 2. Determine Exit State
    if P_critical > P_atm:
        # Choked Flow
        P_exit = P_critical
        T_exit_s = T_in * (2 / (gamma_gas + 1))

        # Apply Efficiency
        T_exit = T_in - eta_n * (T_in - T_exit_s)
        V_exit = np.sqrt(gamma_gas * R * T_exit) # Sonic Velocity at Exit

    else:
        # Unchoked Flow (Full Expansion)
        P_exit = P_atm
        # Isentropic Expansion
        if P_in <= P_atm: # No pressure head
            return 0.0, T_in, P_atm

        T_exit_s = T_in * (P_atm / P_in) ** ((gamma_gas - 1) / gamma_gas)

        # Apply Efficiency
        T_exit = T_in - eta_n * (T_in - T_exit_s)

        delta_T = T_in - T_exit
        if delta_T < 0: delta_T = 0
        V_exit = np.sqrt(2.0 * cp_gas * delta_T)

    return V_exit, T_exit, P_exit

# --- 4. Main Engine Assembler ---

def run_turbojet_analysis(T_atm, P_atm, Mach, Pr_c, T_max):
    """ Runs one complete cycle for a given design point. """
    # Intake (Diffuser) - Ideal Recovery Assumption for simplicity
    # Real intake recovery (Mil-Spec) can be added here
    T02 = T_atm * (1.0 + 0.5 * (gamma_air - 1) * Mach**2)
    P02 = P_atm * (1.0 + 0.5 * (gamma_air - 1) * Mach**2) ** (gamma_air / (gamma_air - 1))

    # Components Sequence
    T03, P03, W_comp = compressor_real(T02, P02, Pr_c, ETA_C)
    T04, P04, f = combustor(T03, T_max, P03, ETA_B, P_LOSS_B)
    T05, P05 = turbine(T04, P04, W_comp, ETA_T, ETA_T_M)
    V_exit, T_exit, P_exit = nozzle(T05, P05, P_atm, ETA_N)

    # Performance Calculations
    a0 = np.sqrt(gamma_air * R * T_atm)
    V0 = Mach * a0

    # Thrust = Momentum + Pressure
    # Assuming m_dot_air = 1 kg/s
    rho_exit = P_exit / (R * T_exit)

    if V_exit > 0:
        area_exit = 1.0 / (rho_exit * V_exit) # Area required for 1 kg/s flow
        pressure_thrust = (P_exit - P_atm) * area_exit
        momentum_thrust = ((1.0 + f) * V_exit) - V0
        specific_thrust = momentum_thrust + pressure_thrust
    else:
        specific_thrust = 0.0

    # TSFC (kg_fuel / N_thrust / hr) -> converted later
    if specific_thrust > 0:
        tsfc = f / specific_thrust
    else:
        tsfc = 0.0

    return specific_thrust, tsfc, f

# --- 5. Parametric Study & Plotting ---

def plot_parametric_cycle():
    print("\nStarting Parametric Analysis (Generating Carpet Plot)...")



    # Design Space Ranges
    pr_range = Pr_range 
    tit_range = TIT_range 

    plt.figure(figsize=(12, 8))

    for TIT in tit_range:
        spec_thrust_list = []
        tsfc_list = []
        valid_pr = [] 

        for Pr in pr_range:
            try:
                F_spec, TSFC, f, = run_turbojet_analysis(T_atm, P_atm, Mach, Pr, TIT)

                # Filter out failed design points
                if F_spec > 0 and TSFC > 0:
                    spec_thrust_list.append(F_spec)
                    # Convert TSFC to g/(kN.s) for industry standard plotting
                    tsfc_scaled = TSFC * 1e6 
                    tsfc_list.append(tsfc_scaled)
                    valid_pr.append(Pr)
            except Exception as e:
                continue 

        # Plotting
        if len(valid_pr) > 0:
            plt.plot(spec_thrust_list, tsfc_list, '-o', linewidth=2, label=f'TIT = {TIT} K')

            # Annotations for Pressure Ratio
            indices = [0, len(valid_pr)//2, len(valid_pr)-1]
            for i in indices:
                plt.annotate(f'Pr={int(valid_pr[i])}', 
                             (spec_thrust_list[i], tsfc_list[i]), 
                             textcoords="offset points", xytext=(0, 8), 
                             ha='center', fontsize=9, fontweight='bold')

    # Formatting
    plt.title('Turbojet Design Space: Parametric Analysis', fontsize=14)
    plt.xlabel('Specific Thrust [N / (kg/s)] \n (Engine Size Indicator)', fontsize=12)
    plt.ylabel('TSFC [g / (kN.s)] \n (Efficiency Indicator)', fontsize=12)
    plt.grid(True, which='both', linestyle='--', alpha=0.6)
    plt.legend(title="Turbine Inlet Temp (TIT)")

    # Optional: Invert Y-axis because lower TSFC is better
    # plt.gca().invert_yaxis()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_parametric_cycle()
