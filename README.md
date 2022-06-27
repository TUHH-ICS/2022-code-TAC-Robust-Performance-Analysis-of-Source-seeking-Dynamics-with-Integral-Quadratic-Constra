# Robust Performance Analysis of Source-Seeking Dynamics with Integral Quadratic Constraints

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.6759553.svg)](https://doi.org/10.5281/zenodo.6759553)

## General

This repository contains the simulation code to reproduce the tables and figures presented in

> A. Datar and C. Hespe and H. Werner, "Robust Performance Analysis of Source-Seeking Dynamics with Integral Quadratic Constraints", 2022 (in preparation)

The code has the following 7 entry points:
1. Quadrotor robustness analysis (corresponding to Fig.3 from the paper): quadrotor_robustness\main_quadrotor_example.m 
2. Non-minimum phase example robustness analysis (corresponding to Fig.4 from the paper): non_minimum_phase_example\main_non_minimum_phase_example.m
3. LPV examples robustness analysis (corresponding to Fig.5 and Fig.6): LPV_Uncertain_system_example\main_LPV_example.m
4. Robustness analysis under formation control(corresponding to Fig.7): formation_source_seek\Example1\main_1.m
5. Robustness analysis under formation control with unknown spectrum (corresponding to Fig.8): formation_source_seek\Example2\main_2.m
6. Robustness analysis under flocking dynamics: flocking_analysis\analyze_flocking_with_LMIs.m
7. Simulation trajectories under flocking dynamics (corresponding to Fig.9): flocking_analysis\Simulation.m 

These will produce among others the material presented in the paper. 

## Prerequisites

To run the simulation files, some additional packages need to be installed first.
1. Install [CVX](http://cvxr.com/cvx/download/)
2. For the flocking simulation (item 7 above), a simulation library (found on [Github](https://github.com/TUHH-ICS/MAS-Simulation)) needs to be imported which can be done by running
```shell
git submodule update --init
``` 

The simulation code in this repository was tested in the following environment:
* *Windows 10 Pro* Version 20H2
* *Matlab* 2021a
