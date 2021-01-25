clear;close all;clc;
% parameters
g=9.8; l1=0.085; l2=0.105; 
m1=17/1000; m2=17/1000;
R = 17.4; K = 0.2;

% A matrix
A32 = 36.0*g*m2/(l1*(16.0*m1 + 12.0*m2));
A33 = -48.0*K^2/(R*l1^2*(16.0*m1 + 12.0*m2));
A42 = g*(12.0*m1 + 36.0*m2)/(l2*(8.0*m1 + 6.0*m2));
A43 = -36.0*K^2/(R*l1*l2*(8.0*m1 + 6.0*m2))
A = [0 0 1 0; 0 0 0 1; 0 A32 A33 0; 0 A42 A43 0];

% B matrix
B3 = 48.0*K/(16.0*R*l1^2*m1 + 12.0*R*l1^2*m2);
B4 = 36.0*K/(8.0*R*l1*l2*m1 + 6.0*R*l1*l2*m2);
B = [0; 0; B3; B4];

% C matrix
C = [1,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];

% D matrix
D = 0;

% LQR
Q = diag([1 10 0 0]);
R = 0.001;
[K, S, EIG] = lqr(A, B, Q, R);
display(K);
display(EIG);


Co = ctrb(A,B);
unco = length(A) - rank(Co);
display(unco);


Fs = 5;
dt = 1/Fs;
N = 50;
t = dt*(0:N-1);
u = [1 zeros(1,N-1)];
[b,a] = ss2tf(A,B,C,D);
display(b);
display(a);
yt = filter(b,a,u);
stem(t,yt,'filled')
xlabel('t')

% system simulation
sys = ss(A,B,C,D);
sys_feedback = feedback(sys,K);
display(sys_feedback)
step(sys_feedback);