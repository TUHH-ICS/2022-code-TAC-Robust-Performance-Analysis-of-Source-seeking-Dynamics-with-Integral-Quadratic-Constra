%---------------------------------------------------------------------------------------------------
% For Paper
% "Robust Performance Analysis of Source-seeking Dynamics with Integral Quadratic Constraints"
% by Adwait Datar and Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script is used to generate data and plot the results for the formation example presented in 
% Fig.8 in the above paper.
%---------------------------------------------------------------------------------------------------
% This function obtains the convergence rate estimates from LMIs with
% different multiplier search restrictions

function alpha_best=get_alpha_with_LMIs(G_veh,m,L_vec,save_data,L_psi_vec,file_name_LMI)

n_L=length(L_vec);
%% Setup optimization
% Optimization tolerences
tolerences.cvx_tol=1e-3; % Tolerence for definiteness in LMIs
tolerences.bisect_tol=1e-3; % Tolerence in alpha for the bisect-algorithm 
tolerences.cond_tol=1e8; % Tolerence for bounding the cond no of positive def variables  

% Run the analysis for different cases defined in the multiplier structure
% with the following properties:

% id: 
% This determines the kind of multiplier used with the following choices
%     1. Circle criterion
%     6. Zames Falb multipliers with analysis LMIs for LTI systems
%     7. Zames Falb multipliers with analysis LMIs for LPV systems

% rho:
% This is valid only for Zames Falb multipliers and is the pole location
% for the basis functions parameterizing the multiplier

% psi_order:
% This is valid only for Zames Falb multipliers and is the order of the
% multiplier that is being searched over

% odd_flag:
% This is valid only for Zames Falb multipliers and is set to one if the
% non-linearity under consideration is odd and is set to 0 otherwise

% causal_flag:
% This is valid only for Zames Falb multipliers. It should be set to 1 if
% restricting the search to causal multipliers, set to -1 is restricting
% the search to anti-causal multipliers and set to 0 is searching over
% general non-causal multipliers which includes causal and non-causal
% parts.
multiplier_flag=[1,60,61,59];
for i=1:4
    switch multiplier_flag(1,i)
    case 1
        multiplier_class.id=1;
        save_path=[file_name_LMI,'_CC_',num2str(multiplier_flag(1,i))];
    case 60  
        multiplier_class.id=6;
        multiplier_class.rho=-1;
        multiplier_class.psi_order=1;
        multiplier_class.odd_flag=0;
        multiplier_class.causal_flag=0; % 1: causal, -1:anti-causal, 0:non-causal
        save_path=[file_name_LMI,'_non_causal_',num2str(multiplier_flag(1,i))];
    case 61  
        multiplier_class.id=6;
        multiplier_class.rho=-1;
        multiplier_class.psi_order=1;
        multiplier_class.odd_flag=0;
        multiplier_class.causal_flag=1; % 1: causal, -1:anti-causal, 0:non-causal
        save_path=[file_name_LMI,'_causal_',num2str(multiplier_flag(1,i))];
    case 59  
        multiplier_class.id=6;
        multiplier_class.rho=-1;
        multiplier_class.psi_order=1;
        multiplier_class.odd_flag=0;
        multiplier_class.causal_flag=-1; % 1: causal, -1:anti-causal, 0:non-causal
        save_path=[file_name_LMI,'_anti_causal_',num2str(multiplier_flag(1,i))];
    end
    alpha_best=zeros(n_L,1);
    alpha_lims=[0,10]; % Initial range for the bisection algorithm
    for j=1:n_L        
        if j>1 && alpha_best(j-1)==-1
                alpha_best(j)=-1;            
            else       
                L=L_vec(j);
                [alpha_best(j)]=bisection_exponent(G_veh,m,L,alpha_lims,tolerences,multiplier_class);
        end
    end
    if save_data==1
        save(save_path)
    end
end
end
