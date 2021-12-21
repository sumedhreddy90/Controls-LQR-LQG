% Non Linear LQG
clear all
x_initial = [0;0;45;0;55;0;0;0;0;0;0;0];
span = 0:0.1:50;
[tout,xout] = ode45(@odesolver,span,x_initial);
hold on
plot(tout,xout,'LineWidth',2)
grid on
hold off

function dxdt = odesolver(~,x)
M=1000; 
ms1=100;
ms2=100;
l1=20;
l2=10;
g=10;
A=[0 1 0 0 0 0; 
    0 0 -(ms1*g)/M 0 -(ms2*g)/M 0;
    0 0 0 1 0 0;
    0 0 -((M+ms1)*g)/(M*l1) 0 -(ms2*g)/(M*l1) 0;
    0 0 0 0 0 1;
    0 0 -(ms1*g)/(M*l2) 0 -(g*(M+ms2))/(M*l2) 0];
B=[0; (1/M); 0; (1/(M*l1)); 0; (1/(M*l2))];
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];
Q = C'*C;
Cm = [10 0 0 0 0 0];
R = 0.01;
[Kgain, ~, ~] = lqr(A,B,Q,R);
F=-Kgain*x; 
p_noise=0.1*eye(6);
m_noise=1;
K_lqg=lqr(A',Cm',p_noise,m_noise)';
lm =(A-K_lqg*Cm)*y(7:12);

dxdt=zeros(12,1);
dxdt(1) = x(2); 
dxdt(2)=(F-(g/2)*(ms1*sind(2*x(3))+ms2*sind(2*x(5)))-(ms1*l1*(x(4)^2)*sind(x(3)))-(ms2*l2*(x(6)^2)*sind(x(5))))/(M+ms1*((sind(x(3)))^2)+ms2*((sind(x(5)))^2));
dxdt(3)= x(4); 
dxdt(4)= ((dxdt(2)*cosd(x(3))-g*(sind(x(3))))/l1'); 
dxdt(5)= x(6); 
dxdt(6)= ((dxdt(2)*cosd(x(5))-g*(sind(x(5))))/l2);
dxdt(7)= x(2)-x(10);
dxdt(8)= dxdt(2)-lm(2);
dxdt(9)= x(4)-x(11);
dxdt(10)= dxdt(4)-lm(4);
dxdt(11)= x(6)-x(12);
dxdt(12)= dxdt(6)-lm(6);
end