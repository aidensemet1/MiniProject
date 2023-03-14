% Group 5 
% Mini-project closed loop system
%% Demo1_5.m
%voltage values
va5=0.5;
va6=0.5;
%turning rad/sec
theta_dot5=3.6;
theta_dot6=3.6;
%% Run a Simulation
open_system('Demo1_5phi')
out=sim('Demo1_5phi');
%% A Plot of the results
figure
plot(out.phi_dot,'g') % phi_dot
figure
plot(out.phi,'b') %phi