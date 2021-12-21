clc; clear; close all;
% LQG for Linear system
syms mass1 mass2 M l1 l2 F 
g=10;
 
A = [0 1 0 0 0 0; 
     0 0 -g*mass1/M 0 -g*mass2/M 0;
     0 0 0 1 0 0;
     0 0 -g/l1*(1+mass1/M) 0 -g/l1*(mass2/M) 0;
     0 0 0 0 0 1;
     0 0 -g/l2*(mass1/M) 0 -g/l2*(1+mass2/M) 0 ];

B = [0;1/M;0;1/(M*l1);0;1/(M*l2)];
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];
Q = C'*C;
Q(1,1) = 9000
Q(2,1) = 1000
R = 0.01;
C_1 = [1 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0];
  
C_3 = [1 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 1 0]; 
  
C_4 = [1 0 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 1 0];
 
% substituting system values
M=1000; mass1=100;mass2=100; l1=20; l2=10; g=10; 
A = double(subs(A));
B = double(subs(B));
X_inital = [6,0,90,0,90,0,0,0,0,0,0,0];

% Defining noises
p_noise= 0.1*eye(6)
m_noise= 1;

% LQR for optimum control
K_t = lqr(A,B,Q,R);
% Constructing state estimator for C_1 output vector
gain_k = lqr(A', C_1', p_noise, m_noise)'
A_obs_1 = [(A-B*K_t) B*K_t;zeros(size(A)) (A-gain_k*C_1)]; 
B_obs_1 = [B ;zeros(size(B))];
C_obs_1 = [C_1 zeros(size(C_1))];
est_c1 = ss(A_obs_1,B_obs_1,C_obs_1,0);

figure
initial(est_c1,X_inital)
figure
step(est_c1)