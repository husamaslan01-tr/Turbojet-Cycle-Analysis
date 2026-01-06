# Turbojet Real Cycle Analysis Tool

A Python-based parametric analysis tool for single-spool Turbojet engines. 
## This project calculates engine performance metrics (Specific Thrust, TSFC) under various design conditions, accounting for real gas properties and component efficiencies.

## Overview

This tool performs a **0-D Thermodynamic Cycle Analysis** for a turbojet engine. Unlike ideal cycle analysis, this model incorporates:
- **Variable Gas Properties:** $C_p$ and $\gamma$ change with temperature.
- **Component Efficiencies:** Isentropic and polytropic efficiencies for compressor, turbine, and nozzle.
- **Pressure Losses:** Accounts for pressure drop in the combustion chamber.
- **Choked Flow:** Checks for nozzle choking conditions.

It generates a **Carpet Plot** (Design Space) to help engineers visualize the trade-offs between Specific Thrust and Specific Fuel Consumption (TSFC) across different Pressure Ratios ($Pr$) and Turbine Inlet Temperatures ($TIT$).

## Features

- **Modular Code Structure:** Separate functions for Compressor, Combustor, Turbine, and Nozzle.
- **Parametric Study:** Automatically sweeps through a range of Pressure Ratios (5-40) and TITs (1200K-1800K).
- **Visualization:** Generates a professional "Carpet Plot" using Matplotlib.
- **Realistic Physics:** Detects nozzle choking and calculates thrust accordingly (Momentum + Pressure Thrust).

##  How to Run

### Prerequisites
You need Python 3.x and the following libraries:
pip install numpy matplotlib

Execution
Simply run the main script:
python turbojet_analysis.py

Results
The tool outputs a Carpet Plot showing the engine design space.

X-Axis: Specific Thrust (N/kg/s) - Indicator of engine size.

Y-Axis: TSFC (g/kN.s) - Indicator of fuel efficiency.

Note: The plot includes constant TIT lines and markers for different Pressure Ratios.


## Design Parameters (Default)
Parameter                Value                    Description.

Flight Condition         Sea Level Static (M=0)   Standard Atmosphere
Compressor Eff (ηc​)      0.85                     Isentropic
Turbine Eff (ηt​)         0.90                     Isentropic
Burner Eff (ηb​)          0.98                     Combustion Efficiency
Nozzle Eff (ηn​)          0.98                     Adiabatic

Author
Husam Aslan Mechanical Engineer. (PhD)


