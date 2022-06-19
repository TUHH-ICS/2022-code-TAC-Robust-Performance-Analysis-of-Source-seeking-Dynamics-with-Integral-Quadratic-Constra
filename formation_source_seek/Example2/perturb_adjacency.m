%---------------------------------------------------------------------------------------------------
% For Paper
% "XYZ"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script is used to take an adjacency matrix and return a perturbed
% adjacency matrix by randomly adding some edges 
function A=perturb_adjacency(A0,link_prob,topo)
N=size(A0,1);
Ad=gen_topology(N,link_prob,topo);
A=A0|Ad;
end