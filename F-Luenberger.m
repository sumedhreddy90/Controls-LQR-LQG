clc; clear; close all;
% Luenberger observer for Linear system
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

% Pole placement 
p = [-3 -4 -5 -6 -7 -8];
X_inital = [0,0,45,0,55,0,0,0,0,0,0,0];
% LQR for optimum control
K_t = lqr(A,B,Q,R);
% Constructing state estimator for C_1 output vector
Luen_1 = place(A',C_1',p)'
A_obs_1 = [(A-B*K_t) B*K_t;zeros(size(A)) (A-Luen_1*C_1)]; 
B_obs_1 = [B ;zeros(size(B))];
C_obs_1 = [C_1 zeros(size(C_1))];
est_c1 = ss(A_obs_1,B_obs_1,C_obs_1,0);
t1 = 0:0.1:10;
unit_step = 1*ones(size(t1));
[yc1,t1,xc1]=lsim(est_c1,unit_step,t1,X_inital);

% Constructing state estimator for C_3 output vector
Luen_3 = place(A',C_3',p)'
A_obs_3 = [(A-B*K_t) B*K_t;zeros(size(A)) (A-Luen_3*C_3)]; 
B_obs_3 = [B ;zeros(size(B))];
C_obs_3 = [C_3 zeros(size(C_3))];
est_c3 = ss(A_obs_3,B_obs_3,C_obs_3,0);
t3 = 0:0.1:10;
unit_step = 1*ones(size(t3));
[yc3,t3,xc3]=lsim(est_c3,unit_step,t3,X_inital);

% Constructing state estimator for C_4 output vector
Luen_4 = place(A',C_4',p)'
A_obs_4 = [(A-B*K_t) B*K_t;zeros(size(A)) (A-Luen_4*C_1)]; 
B_obs_4 = [B ;zeros(size(B))];
C_obs_4 = [C_4 zeros(size(C_4))];
est_c4 = ss(A_obs_4,B_obs_4,C_obs_4,0);
t4 = 0:0.05:1;
unit_step = 1*ones(size(t4));
[yc4,t4,xc4]=lsim(est_c4,unit_step,t4,X_inital);

figure
initial(est_c1,X_inital)
figure
step(est_c1)


figure
initial(est_c3,X_inital)
figure
step(est_c3)

figure
initial(est_c4,X_inital)
figure
step(est_c4)
grid on