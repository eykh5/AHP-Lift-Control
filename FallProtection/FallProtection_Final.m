clc; close all; clearvars;
%% system dynamics
m_J = .2;           % Reel mass (kg)
r = .05;            % Reel radius (m)
J = 1/2*m_J*r^2;    % Moment of inertia for reel (cylinder)
k = 690;            % Spring stiffness (N/m)
b = 0;              % Spring damping 
m = 1;              % Mass (kg)
g = 9.81;           % Gravitational acceleration

wn = sqrt(k/(2*m));

A = [0,-r/(2*J),0,0;k*r/2,0,-k/2,0;0,1/(2*m),0,0;0,0,1,0];
B = [1/J,0;0,0;0,1/m;0,0];

C = eye(4);
D = zeros(4,2);

sys = ss(A,B,C,D);
sys.InputName = {'T_motor','F_{ext}'};
sys.OutputName = {'\Omega_J','f_k','v_m','x_m'};

TFs = tf(sys);
s = tf('s');

Fext2Fk = TFs(2,2);

%% Motor dynamics
ka = .41;      % amplifer gain
km = 0.0507;    % motor gain
N = 5.9;       % Gear ratio
J_motor = 1.2e-6;
b_motor = 0;
enc_count = 2000;

Tmax = 1;
%% Simulation Parameters
bws = 0;

xf = .5;
vmax = .3;
amax = 5;
xi = .5;
v0 = -1.5;
% xi = 0;
% v0 = 0;

param(1) = xi;
param(2) = xf;
param(3) = v0;
param(4) = vmax;
param(5) = amax;

BTI = 0.005;


%% LQR
x0 = [v0/r,(bws)*2*m*g,v0,xi];

A2 = A;
B2 = B(:,1);
C2 = [0,0,0,1];
D2 = 0; 

sys2 = ss(A2,B2,C2,D2);


% Q = diag([1/(1*vmax/r)^2,1/(.01*2*m*g)^2,1/(1*vmax)^2,1/(.0001)^2]);
% R = 1/(.01)^2;

% Q = diag([1/(1*vmax/r)^2,1/(.001*2*m*g)^2,1/(1*vmax)^2,1/(.0001)^2]);
% R = 1/(.01)^2;

Q = diag([1/(1*vmax/r)^2,1/(.0001*2*m*g)^2,1/(1*vmax)^2,1/(.00001)^2]);
R = 1/(.005)^2;

Q = diag([1/(1*vmax/r)^2,1/(.0001*2*m*g)^2,1/(1*vmax)^2,1/(.00001)^2]);
R = 1/(.0002)^2;

% ver c
Q = diag([1/(1*vmax/r)^2,1/(.0001*2*m*g)^2,1/(1*vmax)^2,1/(.000005)^2]);
R = 1/(.0002)^2;

% ver d
% Q = diag([1/(1*vmax/r)^2,2.7/(.0001*2*m*g)^2,1/(1*vmax)^2,.5/(.000005)^2]);
% R = 1/(.0002)^2;

% Q = diag([1/(.0005*vmax/r)^2,1/(.0001*2*m*g)^2,1/(1*vmax)^2,1/(.000009)^2]);
% R = 1/(.005)^2;


% K = lqr(A2,B2,Q,R);
% K = lqrd(A2,B2,Q,R,.005);
K = lqrd(A2,B2,Q,R,BTI);


%% Filter
fc = 10;
wc = 2*pi*fc;

LPF = tf(1,[1/wc,1]);
LPFdp = c2d(LPF,BTI,'tustin');
LPFd = tf(LPFdp);
[num,den] = tfdata(LPFd,'v');
sos = tf2sos(num,den);

%% Analisys
e = eig(A2-B2*K);
fs = max(abs(real(e)))/2/pi;
ts = 1/fs;

%% Plots


simout = sim('FallProtectionsim_Final.slx');
t = simout.tout;
yout = simout.yout;
pos_ref = yout(:,1);
pos_mass = yout(:,2);
vel_mass = yout(:,3);
acc_mass = yout(:,4);
torque_motor = yout(:,5);
vel_motor = yout(:,6);
current = yout(:,7);
force_spr = yout(:,8);
vel_ref = yout(:,9);
acc_ref = yout(:,10);

% figure
subplot(3,1,1)
plot(t,pos_mass);
hold on
plot(t,pos_ref,'--');
hold off;
xlabel('Time (s)')
ylabel('Postion (m)')
% title('Mass Position')
subplot(3,1,2)
plot(t,vel_mass);
hold on
plot(t,vel_ref,'--');
hold off;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
% title('Mass Velocity')
subplot(3,1,3)
plot(t,acc_mass);
hold on
plot(t,acc_ref,'--');
hold off;
xlabel('Time (s)')
ylabel('Acceleration (m/s^{2})')
% title('Mass Acceleration')

figure
subplot(2,3,[1,2,4,5])
plot(abs(vel_motor),abs(torque_motor),'.')
hold on
plot([0,85],[1.5,0],'--')
hold off
xlabel('Velocity (rad/s)')
ylabel('Torque (Nm)')
title('Operating Range')
subplot(2,3,3)
plot(t,torque_motor);
xlabel('Time (s)')
ylabel('Torque (Nm)')
ylim([-2,2])
title('Motor Torque')
subplot(2,3,6)
plot(t,vel_motor);
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
title('Motor Velocity')



%% Header export
variables(1) = r;
variables(2) = J;
variables(3) = k;
variables(4) = m;

HeaderName = 'myLQR.h';
fid = fopen(HeaderName,'w');
comment = 'LQR fall protection';
K2header(fid,K,'K',BTI,variables,comment);
fclose(fid);

HeaderName = 'myLPF.h';
fid = fopen(HeaderName,'w');
comment = 'Fall Protection Velocity Filter';
sos2header(fid,sos,'LPF',BTI,comment);
fclose(fid);

%% test
omega = vel_motor;
force = force_spr;
vel = vel_mass;
pos = pos_mass;
reference = pos_ref;

VDAout = torque_motor/N/km/ka;

figure
subplot(4,1,1)
plot(t,omega);
title('omega')
subplot(4,1,2)
plot(t,force);
title('force')
subplot(4,1,3)
plot(t,vel);
title('velocity')
subplot(4,1,4)
hold on
plot(t,pos);
plot(t,reference)
title('position')

figure
plot(t,VDAout)
title('VDAout')

