% Group 5 
%% Demo1_4forward.m
%voltage provided to motors
va1=0.5; 
va2=0.5;
%turning rad/sec
theta_dot1=3.6;
theta_dot2=3.6;
%% Run a Simulation
open_system('Demo1_4loopRoh')
out=sim('Demo1_4loopRoh');
%% A Plot of the results
figure
plot(out.roh_dot,'g') %roh_dot
figure
plot(out.roh,'b') %roh 
