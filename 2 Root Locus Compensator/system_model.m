
%Case study example for 5th Sem. control Systems example. Faculty incharge:
%Gurunayk Nayak guru@msrit.edu, gurunayk@ieee.org
%https://ctms.engin.umich.edu/CTMS/index.php?example=AircraftPitch&section=SystemModeling

clc; clear all;
format long;

A = [-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B = [0.232; 0.0203; 0];
C = [0 0 1];%eye(3);%
D = [0];%[0;0;0];

states = {'AoA' 'Pitch_rate','Pitch_angle'};
inputs = {'elevator_deflection_angle'};
outputs = {'pitch_angle'}; %{'AoA' 'Pitch_rate','Pitch_angle'}; 
% Sometimes the output choosen is only pitch angle. as,
% Then we have to set C =[0 0 1] and D =[0].

pitch_ss = ss(A,B,C,D, 'StateName',states,'InputName',inputs, 'OutputName', outputs);
disp("the state space model is...");
(pitch_ss)

s = tf('s');
oltf = ss2tf(A,B,C,D);
%oltf = tf(pitch_ss);
[b,a] =ss2tf(A, B, C, D);
% ol_1 = tf(b(1,:),a);
% ol_2 = tf(b(2,:),a);
ol_3 = tf(b(1,:),a);
% disp("elevator_deflection_angle to output AoA is");
% (ol_1)

% disp("elevator_deflection_angle to output Pitch_rate is");
% (ol_2)

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
step(0.25*sys_cl); % elevator_deflection_angle is the command signal of magnitude 0.2
ylabel('pitch angle (rad)');
title('Closed-loop Step Response');grid;

zpk(sys_cl)

% Lead Compensator
disp("-----LEAD COMPENSATOR---------")

K = 10;
alpha = 0.10;
T = 0.52;
C_lead = K*(T*s + 1) / (alpha*T*s + 1);
margin(C_lead*ol_3), grid

sys_cl = feedback(C_lead*ol_3,1);
step(0.25*sys_cl), grid
title('Closed-loop Step Response with K = 10, \alpha = 0.10, and T = 0.52')

stepinfo(0.25*sys_cl)

disp("-----LQR Controller---------")
% LQR Controller
%To use this LQR method, we need to define two parameters: the 
% state-cost weighted matrix (Q) and the control weighted matrix (R). 
% For simplicity, we will choose the control weighted matrix equal to 1 (R = 1), 
% and the state-cost matrix (Q) equal to pC'C.
p = 2;
Q = p*C'*C; % Check the matrix structure whether square or row matrix.
R = 1;
[K] = lqr(A,B,Q,R);

sys_cl = ss(A-B*K, B, C, D);
step(0.25*sys_cl)
ylabel('pitch angle (rad)');
title('Closed-Loop Step Response: LQR');

% Examination of the above demonstrates that the response is too slow. 
% We can tune the performance of our system to be faster by weighting 
% the importance of the error more heavily than the importance of the control effort. 
% More specifically, this can be done by increasing the weighting factor p. 
% After some trial and error, we settle on a value of p = 50. 

% Also Add precompensation
p = 50;
Q = p*C'*C;
R = 1;
[K] = lqr(A,B,Q,R);
Nbar = rscale(A,B,C,D,K);

sys_cl = ss(A-B*K,B*Nbar,C,D);
step(0.25*sys_cl)
ylabel('pitch angle (rad)');
title('Closed-Loop Step Response: LQR with Precompensation');

