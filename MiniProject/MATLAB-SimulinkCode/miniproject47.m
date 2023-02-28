% Ahmed Aldhamari 
% SEED Lab EENG350
% Group 5 
% Mini-project closed loop system
%% miniproject47.m
%% Transferfunction Values
K=0.00375;
a=20;
%% Run a Simulation
open_system('Miniproject')
out=sim('Miniproject');
%% A Plot of the results
figure
plot(out.velocity,'g') %Actual Velocity
figure
plot(out.position,'b')  %Actual Position 