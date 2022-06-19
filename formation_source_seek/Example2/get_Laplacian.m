%---------------------------------------------------------------------------------------------------
% For Paper
% "Robust Performance Analysis of Source-seeking Dynamics with Integral Quadratic Constraints"
% by Adwait Datar and Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script generates nominal and perturbed Laplacians matrices for a
% specified topology and probability of perturbation.

function [Lap0,Q0,Lap,Q]=get_Laplacian(n0,ng,min_topo,pertb_topo,pertb_prob)
% Nominal adjacency within each group
A0=gen_topology(n0,1,min_topo); % 1 (link probability) is irrelavent
% Set the first agent as the leader
q_vec=zeros(n0,1);
q_vec(1)=1;
Q0=diag(q_vec);
D0=A0*ones(n0,1);
Lap0=diag(D0)-A0;
% Build up big matrices
A_nom=kron(eye(ng),A0);
Q=kron(eye(ng),diag(q_vec));

% Perturb the Laplacian with a random matrix with a given probability
if pertb_prob>0.2
    A=perturb_adjacency(A_nom,pertb_prob,pertb_topo);
else
    A=A_nom;
end
n=n0*ng;
D=A*ones(n,1);
Lap=diag(D)-A;

end
