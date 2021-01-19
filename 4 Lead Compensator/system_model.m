
%Case study example for 5th Sem. control Systems example. Faculty incharge:
%Gurunayk Nayak guru@msrit.edu, gurunayk@ieee.org
%https://ctms.engin.umich.edu/CTMS/index.php?example=AircraftPitch&section=SystemModeling

clc; clear all;
format long;

A = [-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B = [0.232; 0.0203; 0];
C = eye(3);%[0 0 1];
D = [0;0;0];

states = {'AoA' 'Pitch_rate','Pitch_angle'};
inputs = {'elevator_deflection_angle'};
outputs = {'AoA' 'Pitch_rate','Pitch_angle'}; 
% Sometimes the output choosen is only pitch angle. as,
% Then we have to set C =[0 0 1] and D =[0].

pitch_ss = ss(A,B,C,D, 'StateName',states,'InputName',inputs, 'OutputName', outputs);
disp("the state space model is...");
(pitch_ss)

s = tf('s');
oltf = ss2tf(A,B,C,D);
%oltf = tf(pitch_ss);
[b,a] =ss2tf(A, B, C, D);
ol_1 = tf(b(1,:),a);
ol_2 = tf(b(2,:),a);
ol_3 = tf(b(3,:),a);
disp("elevator_deflection_angle to output AoA is");
(ol_1)

disp("elevator_deflection_angle to output Pitch_rate is");
(ol_2)

disp("elevator_deflection_angle to output Pitch_angle is (Chose this one for case study)");
(ol_3 )
disp("Notice that above transfer function is type 1");

disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
% Closed loop Response
disp("Closed loop system transfer function with unity negative feedback is");
sys_cl = feedback(ol_3,1);
sys_cl
disp("Pole-zero form");
zpk(sys_cl)
step(0.2*sys_cl); % elevator_deflection_angle is the command signal of magnitude 0.2
ylabel('pitch angle (rad)');
title('Closed-loop Step Response');grid;

zpk(sys_cl)