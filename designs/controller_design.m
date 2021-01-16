clear;close all;clc;
% parameters
g=9.8; l1=0.085; l2=0.105; 
m1=17/1000; m2=17/1000;

% A matrix
A32 = 18.0*g*l1*m2/(8.0*(l1^2)*m1 + 6.0*(l1^2)*m2);
A42 = 6.0*g*l1*m1/(4.0*l1*l2*m1 + 3.0*l1*l2*m2) + 18.0*g*l1*m2/(4.0*l1*l2*m1 + 3.0*l1*l2*m2);
A = [0 0 1 0; 0 0 0 1; 0 A32 0 0; 0 A42 0 0];

% B matrix
B3 = 24.0/(8.0*(l1^2)*m1 + 6.0*(l1^2)*m2);
B4 = 18.0/(4.0*l1*l2*m1 + 3.0*l1*l2*m2);
B = [0; 0; B3; B4];

% C matrix
C = eye(4);

% D matrix
D = 0;

% LQR
Q = diag([1.6 5 0 0]);
R = 0.0028;
[K, S, EIG] = lqr(A, B, Q, R);
display(K);
display(EIG);

% system simulation
sys = ss(A,B,C,D);
sys_feedback = feedback(sys,K);
step(sys_feedback);