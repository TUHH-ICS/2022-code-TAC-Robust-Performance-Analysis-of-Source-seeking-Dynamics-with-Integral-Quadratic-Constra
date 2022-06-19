%---------------------------------------------------------------------------------------------------
% For Paper
% "Robust Performance Analysis of Source-seeking Dynamics with Integral Quadratic Constraints"
% by Adwait Datar and Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script is used to generate data and plot the results for the formation example presented in 
% Fig.7 in the above paper.
%---------------------------------------------------------------------------------------------------

%% Generate convergence rate estimates with different restrictions of the multiplier search
sweep_L_LMI_analysis
%% Generate exact convergence rates for a fixed scalar field and uncertain Laplacians
gen_LTI_examples_uncertain_Lap
%% Generate  exact convergence rates for a fixed Laplacian and varying scalar field
gen_LTI_examples_uncertain_psi
%% Plot obtained results
plot_data
