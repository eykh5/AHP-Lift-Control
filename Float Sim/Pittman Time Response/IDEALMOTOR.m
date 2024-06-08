clc; close all; clearvars;
%% system dynamics
m_J = .5;           % Reel mass (kg)
r = .025;            % Reel radius (m)
J = 1/2*m_J*r^2;    % Moment of inertia for reel (cylinder)
k = 720;            % Spring stiffness (N/m)
b = 0;              % Spring damping 
m = 1;              % Mass (kg)
g = 9.81;           % Gravitational acceleration
tau = 0.05;

A = [0,-k/2;1/(2*m),-b/(4*m)];
B = [k*r/2,0; b*r/(4*m),1/m];
C = eye(2);
D = [0,0;0,0];

sys = ss(A,B,C,D);
sys.InputName = {'\Omega','F_{ext}'};
sys.OutputName = {'f_k','v_m'};

% Omega to v_m
TFs = tf(sys);
T1 = TFs(2,1);

% F_ext to v_m
T2 = TFs(2,2);

% Omega to f_k
T3 = TFs(1,1);

% F_ext to f_k
T4 = TFs(1,2);

s = tf('s');

%% Motor dynamics
% % Maxon == P: 19.205, I: 52.222, D: 1.7, N: 5454.334
% km = 0.0452;    % motor gain
% N = 62;       % Gear ratio
% J_motor = 8.85e-7;
% PIDF1 = pid(19.205, 52.2, 1.7, 1/5454);


% Dr.Garbini's == P:3.37, I: 10.94, D: 0.25, N:275.9
km = 0.0507;
N = 5.9;
J_motor = 7.06e-6;
PIDF1 = pid(3.37, 10.94, 0.25, 1/275.9);

% % Oyo == P: 5.05, I: 14.86, D: 0.373, N: 5711.4
% km = 0.038;    % motor gain
% N = 13;       % Gear ratio
% J_motor = 14.2e-6;
% PIDF1 = pid(5.05, 14.86, 0.373, 1/5711.4);

ka = .41;      % amplifer gain
b_motor = 0;

% Tm to Omega
T0 = tf(1,[J_motor+J/N,b_motor]);

%% Motor veloicty controller
wc = 2*pi/tau;
opt = pidtuneOptions('designFocus','disturbance-rejection');
[PIDF0,info0] = pidtune(ka*km*T0,'pidf',wc,opt);


%% Simulation 
close all
T_sim=10;
simout=sim( 'IDEALMOTORVAL','Solver','ode45','RelTol','1e-10','AbsTol','auto','MaxStep','T_sim');
t           = simout.get('tout');
vel    = simout.get('vel').Data;          %---Velocity - m/s
refvel = simout.get('refvel').Data;       %---Reference Velocity (Input) - m/s
Fk = simout.get('springforce').Data;      %---Force in Spring - N
motorvel = simout.get('motorvel').Data;   %---Motor Angular Velocity - rad/s
% current = simout.get('current').Data;     %---current - A
% voltage = simout.get('voltage').Data;     %---voltage - V
position = simout.get('position').Data;   %---Displacement - m
% torque = simout.get('torque').Data;       %---torque - Nm
Fext = simout.get('Fext').Data;           %---External Force - N

motorvel = motorvel* 60/(2*pi);           % rad/s => rpm Conversion

figure (1)
plot(t, Fext)
ylabel('Force (N)'); grid on
xlabel('Time (s)');
title('External Force Applied')
legend("7.5N")
figure(2)
sgtitle("System Response")
subplot(2,3,1)
    plot(t, refvel); grid on, hold on
    plot(t, vel);
    ylabel('m/s'); 
    xlabel('s');
    % axis([1.5 10 -0.1 1.1])
    % xlim([1.5 10])
    title('Velocity of Mass')
    legend("Ref Vel", "Vel")
subplot(2,3,2) 
    plot(t,motorvel); grid on, hold on
    plot(t, motorvel/N);
    ylabel('rpm');   
    xlabel('s')    
    % xlim([1.5 10])
    legend("Motor Angular Velocity", "Reel Angular Velocity")
    title('Angular Velocity') 

subplot(2,3,3) 
    plot(t,Fk/k*100), grid on, hold on 
    ylabel('cm')
    xlabel('s')    
    xlim([1.5, 10])
    title('Spring Deflection')
subplot(2,3,[4, 6])
    plot(t, position*100), grid on, hold on 
    ylabel('cm')
    xlabel('s')    
    % xlim([1.5, 10])
    title('Mass Displacement')

