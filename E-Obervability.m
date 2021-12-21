% observability check
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
 
% substituting system values
M=1000; mass1=100;mass2=100; l1=20; l2=10; g=10; 
A = double(subs(A)); B = double(subs(B));



C_1 = [1 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0];
obs = obsv(A,C_1);
observability_C1 = rank(obs)

C_2 = [0 0 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 1 0];
obs = obsv(A,C_2);
observability_C2 = rank(obs)

C_3 = [1 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 1 0];
obs = obsv(A,C_3);
observability_C3 = rank(obs)

C_4 = [1 0 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 1 0];
obs = obsv(A,C_4);
observability_C4 = rank(obs)