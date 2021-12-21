% linearized LQR
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
 
% controllability check
control = simplify( [B A*B A*A*B A*A*A*B A*A*A*A*B A*A*A*A*A*B] )
control_test = length(A) - rank(control) 
var = simplify(det(control))
solve(var==0, M,mass1,mass2,l1,l2, 'ReturnConditions', true);
[ans.M ans.mass1 ans.mass2 ans.l1 ans.l2]
% substituting system values
M=1000; mass1=100;mass2=100; l1=20; l2=10; g=10; 
A = double(subs(A)); B = double(subs(B));
% stability check
stability = eig(A)
% controlabilty check after substitution
control = ctrb(A,B) 
control_test = det(control)
control_test = length(A) - rank(control) 
Q = C'*C;
R=1.5;
[K_init,P_init,e_init] = lqr(A,B,Q,R);
% Stability check using Lyapunov's indirect method
Stablity_Lyp = eig(A-B*K_init)
display(K_init)
R = 0.005;
Q = C'*C ;
display(Q)
% Adjusting the values of Q
Q(1,1) = 9000
[K_load,P_load,e_load] = lqr(A,B,Q,R)
Stablity_Lyp_load = eigs(A-B*K_load)
display(Stablity_Lyp_load)

% LQR simulation for 3 min
A_feedback = (A-B*K_load);
% state response without LQR
ss_init = ss(A ,B,C,0);
% state response with LQR
ss_lqr = ss(A_feedback,B,C,0);

t = 0:0.5:360; 
force = 10*ones(size(t));
[y_init,t,x]=lsim(ss_init,force,t);
[y_lqr,t,x_lqr]=lsim(ss_lqr,force,t);
% Plotting state response without LQR
figure
yyaxis left
hold on
plot(t,y_init(:,1), 'r-');
title('Linearized State Response without LQR')
xlabel('time (seconds)')
ylabel('cart position, distnace x (meters)')
yyaxis right
plot(t,y_init(:,2), 'b-');
plot(t,y_init(:,3), 'r-');
ylabel('angle, \theta (radians)')
legend('x','\theta_1','\theta_2')
hold off

% Plotting state response with LQR
figure
yyaxis left
hold on
plot(t,y_init(:,1), 'r-');
title('Linearized LQR State Response')
xlabel('time (seconds)')
ylabel('cart position, distnace x (meters)')
yyaxis right
plot(t,y_lqr(:,2), 'b-');
plot(t,y_lqr(:,3), 'r-');
ylabel('angle, \theta (radians)')
legend('x_{lrq}','\theta_1','\theta_2')
hold off