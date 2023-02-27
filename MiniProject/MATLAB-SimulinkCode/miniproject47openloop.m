% SEED Lab EENG350
%Mini project open loop system
%% miniproject47openloop.m
%% Transferfunction Values
K=0.00375;
a=20;
%% Run a Simulation
open_system('MiniprojectOL')
out=sim('MiniprojectOL');
%% A Plot of the results
figure
plot(out.velocity,'r:') %Actual Velocity 
figure
plot(out.position,'b') %Actual position