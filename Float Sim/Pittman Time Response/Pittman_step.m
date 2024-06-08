clc; close all; clear

data = importdata("Pittman1.mat");

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

% Dr.Garbini's == P:3.37, I: 10.94, D: 0.25, N:275.9
km = 0.0507;
N = 5.9;
J_motor = 7.06e-6;
PIDF1 = pid(3.37, 10.94, 0.25, 1/275.9);

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
T_sim=1.2;
simout=sim( 'IDEALMOTORVAL','Solver','ode45','RelTol','1e-10','AbsTol','auto','MaxStep','T_sim');
t           = simout.get('tout');
motorvel = simout.get('motorvel_test').Data;   %---Motor Angular Velocity - rad/s



figure (1)
plot(t, motorvel)
ylabel('Motor Angular Velocity (rad/s)'); grid on; hold on
xlabel('Time (s)');
title('Motor Step Response')








velocity = data.velocity;
vref = nonzeros(data.reference);

T = 0.005;            % Period = 5 ms, or 200 Hz
t = 0:T:249*T; 
kvi = 0.41;
kt = 0.0507;
J = 7.06e-6;
plot(t, velocity); 
title("Step Response: \tau = 0.05s")
xlabel("time (s)")
ylabel("Motor velocity (rad/s)")


fprintf("63.2%% of steady state is %.2f\n", velocity(end)*0.632)
vq = interp1(t,velocity,0.00242);
fprintf("Interpolation: %.2f when t= %.2f ms \n", vq, 0.00242*1000)


t1 = 0.00242;

legend("Sim data", "Motor data")