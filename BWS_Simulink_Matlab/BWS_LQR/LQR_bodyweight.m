clc; close all; clearvars;

%% Plant Model

m_J = .2;           % Reel mass (kg)
r = .05;            % Reel radius (m)
J = 1/2*m_J*r^2;    % Moment of inertia for reel (cylinder)
k = 720;            % Spring stiffness (N/m)
b = 0;              % Spring damping 

m = 1;              % Mass (kg)
g = 9.81;           % Gravitational acceleration

ka = .4;      % amplifer gain
km = 0.0507;    % motor gain
N = 5.9;       % Gear ratio
J_motor = 1.2e-6;
b_motor = 0;

% State Model
A = [0,-r/(2*J),0,0;k*r/2,0,-k/2,0;0,1/(2*m),0,0;0,0,1,0];
B = [1/J,0;0,0;0,1/m;0,0];
C = eye(4);
D = zeros(4,2);

sys = ss(A,B,C,D);
sys.InputName = {'T_motor','F_{ext}'};
sys.OutputName = {'\Omega_J','f_k','v_m','x_m'};

TFs = tf(sys);
s = tf('s');

A2 = A;
B2 = B(:,1);
% C2 = [0,0,0,1];
% D2 = 0; 

Fext2Fk = TFs(2,2);
bws = .6;

xf = .75;
vmax = .75;
xi = 0;
v0 = 0;

x0 = [0, 2*m*g*bws, 0, 0];

Q = diag([1/(1*vmax/r)^2, 1/(.005*2*m*g*bws)^2, 1/(1*vmax*.01)^2, 1/(1000000)^2]);
% Q = diag([1/(1*vmax/r)^2, 1/(.005*2*m*g*bws)^2, 1/(1*vmax)^2, 1/(.0001)^2]);
% Q = diag([1/(1*vmax/r)^2, 1/(.00000001*2*m*g)^2, 1/(1*vmax)^2, 1/(.0001)^2]);
% Q = diag([1/(1*vmax/r)^2, 1/(.0000005*2*m*g*bws)^2, 1/(1*vmax*.01)^2, 1/(100)^2]);

% Q = diag([1/10^2; 1/.01^2; 1/.5^2; 1/10^2] );
R = 1/(.13)^2;

% Q = diag([5,100, 1, 30]);
R = 1/(.05)^2;

K = lqrd(A2,B2,Q,R,.0036);
% 
K = lqr(A2,B2,Q,R);

% D2 = zeros(4,1);
% 
% plantR = ss(A,B2,C,D2);
% 
% sysR = feedback(plantR,K);
% 
% initial(sysR, x0);
% 
% return;

Tmax = 1;

ref = [0, 2*m*g*bws, 0, 0];
ref = transpose(ref);

simout = sim('LQR_BWS.slx');
t = simout.tout;
yout = simout.yout;

omega_j = yout{1}.Values.Data;
f_k = yout{2}.Values.Data;
v_m = yout{3}.Values.Data;
x_m = yout{4}.Values.Data;
current = reshape(yout{5}.Values.Data, 1, []);
a_m = yout{6}.Values.Data;

title_size = 30;
axis_size = 27;
lw = 3;

subplot(2,3,1);
plot(t, omega_j, 'LineWidth', lw);
xlabel("time (s)");
ylabel("omega_j (rad/s)");
fontsize(gca, axis_size,'points')   % 'pixels', 'centimeters', 'inches'
[te,s] = title("time vs. omega_j");
te.FontSize = title_size;
grid on;

subplot(2,3,2);
plot(t, f_k, 'LineWidth', lw);
xlabel("time (s)");
ylabel("f_k (Newtons)");
fontsize(gca, axis_size,'points') 
[te,s] = title("time vs. f_k");
te.FontSize = title_size;
grid on;

subplot(2,3,3);
plot(t, v_m, 'LineWidth', lw);
xlabel("time (s)");
ylabel("v_m (m/s)");
fontsize(gca, axis_size,'points') 
[te,s] = title("time vs. v_m");
te.FontSize = title_size;
grid on;

subplot(2,3,4);
plot(t, x_m, 'LineWidth', lw);
xlabel("time (s)");
ylabel("x _m (m)");
fontsize(gca, axis_size,'points')
[te,s] = title("time vs. x_m");
te.FontSize = title_size;
grid on;

subplot(2,3,5);
plot(t, current, 'LineWidth', lw);
xlabel("time (s)");
ylabel("current (A)");
fontsize(gca, axis_size,'points')
[te,s] = title("time vs. current");
te.FontSize = title_size;
grid on;

subplot(2,3,6);
plot(t, a_m, 'LineWidth', lw);
xlabel("time (s)");
ylabel("Disturbance Force (N)");
fontsize(gca, axis_size,'points')
[te,s] = title("time vs. f_d");
te.FontSize = title_size;
grid on;