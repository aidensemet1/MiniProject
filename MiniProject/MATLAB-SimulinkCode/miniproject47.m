% Ahmed Aldhamari 
% SEED Lab EENG350
%mini project closed loop system
%% miniproject47.m
%% Transferfunction Values
K=0.00375;
a=20;
%% Run a Simulation
open_system('Miniproject')
out=sim('Miniproject');
%% A Plot of the results
figure
plot(out.simout,'r:')