clear;close all;clc;
% parameters
g=9.8;
l1=0.085;
l2=0.105/2; # Centre of mass of rod = length / 2
m1=17/1000; m2=17/1000;
R = 12; Km = 0.3;

% A matrix
A32 = 36.0*g*m2/(l1*(16.0*m1 + 12.0*m2));
A33 = -48.0*Km^2/(R*l1^2*(16.0*m1 + 12.0*m2));
A42 = g*(12.0*m1 + 36.0*m2)/(l2*(8.0*m1 + 6.0*m2));
A43 = -36.0*Km^2/(R*l1*l2*(8.0*m1 + 6.0*m2))
A = [0 0 1 0; 0 0 0 1; 0 A32 A33 0; 0 A42 A43 0];

% B matrix
B3 = 48.0*Km/(16.0*R*l1^2*m1 + 12.0*R*l1^2*m2);
B4 = 36.0*Km/(8.0*R*l1*l2*m1 + 6.0*R*l1*l2*m2);
B = [0; 0; B3; B4];

% C matrix
C = eye(4, 4);

% D matrix
D = 0;

% LQR
Q = diag([1 100 0 10]);
R = 0.001;
[K, S, EIG] = lqr(A, B, Q, R);
display(K);
display(EIG);
display(S);


% system simulation
sys = ss(A,B,C,D);
sys_feedback = feedback(sys, K);
SP = 10
step(SP * sys_feedback)