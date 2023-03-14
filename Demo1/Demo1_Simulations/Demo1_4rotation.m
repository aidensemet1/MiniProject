% Group 5 
% Mini-project closed loop system
%% Demo1_4rotation.m
va3=0.5;
va4=0.5;
theta_dot3=3.6;
theta_dot4=-3.6;
%% Run a Simulation
open_system('Demo1_4loopPhi')
out=sim('Demo1_4loopPhi');
%% A Plot of the results
figure
plot(out.phi_dot,'g') % phi_dot
figure
plot(out.phi,'b') %phi