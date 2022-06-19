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
% This script generates sweeps over L_psi and tests the conservatism of the
% analysis conditions by finding worst case examples
clear
clc
addpath('..\..\analysis_scripts')
addpath('..\.')
save_data=1;
analyze=1;
%% Assumptions and compute m and L
% 1) At least one third of total number of agents are leaders.
% 2) Every non-leader agent has an edge with at least one leader.
% 3) Maximum degree of all agents (including leaders)  is $2$.				
% 4) $\psi \in \mathcal{S}(3,L_{\psi})$.

% Sector bounds for underlying field
m_psi=3; % lower bound on the sector
L_psi_vec=[3:0.5:18];  % Upper bound on the sector

% Get m and L for analysis
L1=[1+m_psi,-1;-1,1];
L2=[2+m_psi,-1,-1;-1,1,0;-1,0,1];
m=min([eig(L1);eig(L2);m_psi]);
L_vec=L_psi_vec+2*2;  % Upper bound on the sector

%% Generate data for specific examples
% Vehicle Dynamics
% Select a Vehicle model from the following choices
% 1. Mass with friction
% 2. Linearized Quadrotor
Veh_mod=2;
switch(Veh_mod)
    case 1
        % Mass with friction dynamics
        addpath(genpath('..\..\vehicles\mass_with_friction'))
        c_damp=1;mass=1;step_size=1;
        dim=1;% spatial dimension (of positions and velocities)
        G_veh=define_G_mass_with_friction_wrapped(dim,c_damp,mass,step_size);
    case 2
        % Define quadrotor dynamics 
        addpath(genpath('..\..\vehicles\quadrotor'))
        dim=2;% spatial dimension (of positions and velocities)
        % Current implementation only supports dim=2 for quadrotors
        kp=1;kd=10; % gains for the pre-filter
        G_veh=define_G_quad_wrapped(dim,kp,kd);        
end
%% Get analytical bounds with LMIs
file_name_LMI=['.\data\alpha_LMI'];
if analyze==1
    alpha_best_LMI=get_alpha_with_LMIs(G_veh,m,L_vec,save_data,L_psi_vec,file_name_LMI);
end

%% Get the actual bounds on first set of Linear examples
% Fix the Laplacian for the first set of examples
n0=4; % number of agents in the sub-graph
ng=1; % Number of groups
min_topo='cycl'; % topology of the minimal graph
pertb_prob=0; pertb_topo='latt'; % Perturb the nominal Topology
[Lap0,~,Lap,~]=get_Laplacian(n0,ng,min_topo,pertb_topo,pertb_prob);
q_vec=ones(n0,1); % Set all agents have leaders
Q=kron(eye(ng),diag(q_vec));
file_name_lin_eg1=['.\data\alpha_lin_eg1'];
alpha_best_lin=get_alpha_LTI_example(m_psi,L_psi_vec,G_veh,Lap,Q);
if save_data==1
    save(file_name_lin_eg1);
end
%% Get the actual bounds on first set of Linear examples
% Fix the Laplacian for the first set of examples
n0=3; % number of agents in the sub-graph
ng=1; % Number of groups
min_topo='star'; % topology of the minimal graph
pertb_prob=0; pertb_topo='latt'; % Perturb the nominal Topology
[Lap0,~,Lap,~]=get_Laplacian(n0,ng,min_topo,pertb_topo,pertb_prob);
q_vec=zeros(n0,1);q_vec(1)=1; % Set all agents have leaders
Q=kron(eye(ng),diag(q_vec));
file_name_lin_eg2=['.\data\alpha_lin_eg2'];
alpha_best_lin=get_alpha_LTI_example(m_psi,L_psi_vec,G_veh,Lap,Q);
if save_data==1
    save(file_name_lin_eg2);
end

%% Plot data
% % If filenames not already defined
% if exist('min_topo','var')== 0
%     min_topo='star'; % topology of the minimal graph
%     file_name_lin_eg=['.\data\alpha_lin_eg_topo_',min_topo];
%     file_name_LMI=['.\data\alpha_LMI_topo_',min_topo];
% end
% 


% Load data from linear examples
data_eg1=load(file_name_lin_eg1,'alpha_best_lin','L_psi_vec');
alpha_best_eg1=data_eg1.alpha_best_lin;
L_psi_vec_eg1=data_eg1.L_psi_vec; 

% Load data from linear examples
data_eg2=load(file_name_lin_eg2,'alpha_best_lin','L_psi_vec');
alpha_best_eg2=data_eg2.alpha_best_lin;
L_psi_vec_eg2=data_eg2.L_psi_vec; 


figure()
plot_data_perf([file_name_LMI,'_CC_1'],'ro')
hold on    
plot_data_perf([file_name_LMI,'_causal_61'],'g--')
plot_data_perf([file_name_LMI,'_anti_causal_59'],'b-.')
plot_data_perf([file_name_LMI,'_non_causal_60'],'c')
plot(L_psi_vec_eg1,alpha_best_eg1,'ko','LineWidth',1)
plot(L_psi_vec_eg2,alpha_best_eg2,'k*','LineWidth',1)
ylim([0,1.1*max(alpha_best_eg2)])
legend('CC(P_1=0,P_3=0)','ZF causal (P_1=0)','ZF anti-causal (P_3=0)','ZF','Examples','Examples')
xlabel('L \psi')
ylabel('\alpha')
%title('Convergence rates(exponents) for Quadrotor')
%% User defined functions
function []=plot_data_perf(file,plot_style)
    save_path=['.\',file];
    % Load data from analysis
    data_LMI=load(file,'alpha_best','L','L_psi_vec');
    alpha_best=data_LMI.alpha_best;
    L=data_LMI.L_psi_vec;
    plot(L,alpha_best,plot_style,'LineWidth',1)    
end
