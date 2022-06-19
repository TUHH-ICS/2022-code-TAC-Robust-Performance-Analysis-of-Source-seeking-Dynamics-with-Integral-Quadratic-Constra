%---------------------------------------------------------------------------------------------------
% For Paper
% "Robust Performance Analysis of Source-seeking Dynamics with Integral Quadratic Constraints"
% by Adwait Datar and Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This function generates worst case linear examples and computes the exact
% convergence rates for these examples
function alpha_best=get_alpha_LTI_example(m_psi,L_psi_vec,G_veh,Lap,Q)
n_L=length(L_psi_vec);
alpha_best=zeros(n_L,1);
    for i=1:n_L
        L_psi=L_psi_vec(i);
        % Big system matrices for an example quadratic field to test conservatism
        dim=size(G_veh.C,1);
        switch dim
            case 1
                H=m_psi;        
            case 2
                H=[m_psi,0;0,L_psi]; 
        end
        L_per_big=kron(Lap,eye(dim))+kron(Q,H);
        n=size(Lap,1);
        G_veh_big=ss(kron(eye(n),G_veh.A),kron(eye(n),G_veh.B),kron(eye(n),G_veh.C),kron(eye(n),G_veh.D));
        G_cl=feedback(G_veh_big,L_per_big,-1);
        alpha_best(i)=-max(real(pole(G_cl)));    
    end
end
