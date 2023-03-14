% Group 5 
% Mini-project closed loop system
%% Demo1_5.m
x_s=[0 3 6 10 5 8 7 2 6];
y_s=[0 0 3 3 5 7 8 2 6];
roh_dot_d=0.1; %ft/s
Y_0=0;
X_0=0;
Phi_0=0;
%% Run a Simulation
open_system('Demo1_6Camera');
out=sim('Demo1_6Camera');
%% A Plot of the results
figure
plot(out.pos,'g') % phi_dot
