%---------------------------------------------------------------------------------------------------
% For Paper
% "Robust Performance Analysis of Source-seeking Dynamics with Integral Quadratic Constraints"
% by Adwait Datar and Christian Hespe and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script is used to generate data and plot the results for the flocking example presented in 
% Fig.9 in the above paper.
%---------------------------------------------------------------------------------------------------

% Load data
data2=load('data\traj2','t_sampled','sampled');
data3=load('data\traj3','t_sampled','sampled');
data4=load('data\traj4','t_sampled','sampled');
data5=load('data\traj5','t_sampled','sampled');

figure()
subplot(2,2,1)
plot_traj(data2.t_sampled,data2.sampled)
title('kd=2')
subplot(2,2,2)
plot_traj(data3.t_sampled,data3.sampled)
title('kd=3')
subplot(2,2,3)
plot_traj(data4.t_sampled,data4.sampled)
title('kd=4')
subplot(2,2,4)
plot_traj(data5.t_sampled,data5.sampled)
title('kd=5(LMI feasible)')
%% Use defined functions
function []=plot_traj(t_sampled,sampled)
    xx=squeeze(sampled.position(:,1,:));
    yy=squeeze(sampled.position(:,2,:)); 
    plot(t_sampled,xx(:,1),'r')
    hold on
    plot(t_sampled,xx(:,2),'g')
    plot(t_sampled,xx(:,3),'b')
    plot(t_sampled,yy(:,1),'r--')
    plot(t_sampled,yy(:,2),'g--')
    plot(t_sampled,yy(:,3),'b--')
    xlabel('time')
end
