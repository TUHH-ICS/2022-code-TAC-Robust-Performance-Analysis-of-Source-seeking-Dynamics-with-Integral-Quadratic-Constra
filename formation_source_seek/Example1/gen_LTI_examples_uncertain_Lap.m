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
% This script generates exact convergence rates for a fixed scalar field
% and uncertain Laplacians
clear
clc
addpath('..\.')
save_data=1;
rng(0)
%% Sector bounds for underlying field
m_psi=1.85; % lower bound on the sector
L_psi=1.85;  % Upper bound on the sector

% Gain on the gradient forcing term on the leaders
k_psi=1;
% Gain on the interaction term (Laplacian) term
k_lap=1;
% Gain on the interaction term (Laplacian) term
k_net=1;
%% Vehicle Dynamics
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

%% Fix the Laplacian
n0=5; % number of agents in the sub-graph
m0=1; % number of informed agents in the sub-graph
ng=5; % Number of groups
n=n0*ng; % total number of agents
no_leaders=m0*ng; % total no of ledaers who can measure gradient

% Nominal adjacency within each group
A0=gen_topology(n0,1,'star'); % 1 (link probability) is irrelavent
q_vec=zeros(n0,1);
q_vec(1)=1;
D0=A0*ones(n0,1);
Lap0=diag(D0)-A0;
Lg0_min=Lap0+k_psi*m_psi*diag(q_vec);
% Build up big matrices
A_nom=kron(eye(ng),A0);
Q=kron(eye(ng),diag(q_vec));

% Perturb the Laplacian with a random matrix with a given probability
link_prob_vec=[0.35:0.01:0.45]; 
for i=1:length(link_prob_vec)
    link_prob=link_prob_vec(i);
    topo='rand';
    A=perturb_adjacency(A_nom,link_prob,topo);
    D=A*ones(n,1);
    Lap=diag(D)-A;
    % Scale the Laplacian
    Lap=k_lap*Lap;
    eig_Lap=eig(Lap);

    Lg1=Lap+k_psi*m_psi*Q;
    % Overall scaling
    Lg1=k_net*Lg1;
    m(i)=min(eig(Lg1));


    Lg2=Lap+k_psi*L_psi*Q;
    Lg2=k_net*Lg2;
    L(i)=max(eig(Lg2));
    %% Big system matrices for an example quadratic field to test conservatism
    switch dim
        case 1
            H=m_psi;        
        case 2
            H=[m_psi,0;0,L_psi]; 
    end
    L_per_big=kron(Lap,eye(dim))+k_psi*kron(Q,H);
    L_per_big=k_net*L_per_big;
    G_veh_big=ss(kron(eye(n),G_veh.A),kron(eye(n),G_veh.B),kron(eye(n),G_veh.C),kron(eye(n),G_veh.D));
    G_cl=feedback(G_veh_big,L_per_big,-1);
    alpha_best(i)=-max(real(pole(G_cl)));
    
end

if save_data==1
    save('.\data\lb_lin_uncertain_Lap');
end