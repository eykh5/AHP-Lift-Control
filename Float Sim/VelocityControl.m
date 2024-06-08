clc; close all; clearvars;
%% system dynamics
m_J = 0.170;           % Reel mass (kg)
r = .05;            % Reel radius (m)
J = 1/2*m_J*r^2;    % Moment of inertia for reel (cylinder)
k = 720;            % Spring stiffness (N/m)
b = 0;              % Spring damping 
m = 1;              % Mass (kg)
g = 9.81;           % Gravitational acceleration
tao = 1;

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
% PIDF1 = pid(3.37, 10.94, 0.25, 1/275.9);
km = 0.0507;
N = 5.9;
J_motor = 7.06e-6;
% PIDF1 = pid(54.9905391674036, 100.463632665372, 4.86748418055206, 1/44.0963966861224);
PIDF1 = pid(544.5, 0, 8.674, 1/276);
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
ts = 1;
wc = 2*pi/1 ;
opt = pidtuneOptions('designFocus','disturbance-rejection');
[PIDF0,info0] = pidtune(ka*km*T0,'pidf',wc,opt);


%% Simulation 
close all
T_sim=15;
simout=sim( 'FLOATFINALSIM','Solver','ode45','RelTol','1e-10','AbsTol','auto','MaxStep','T_sim');
t           = simout.get('tout');
vel    = simout.get('vel').Data;          %---Velocity - m/s
refvel = simout.get('refvel').Data;       %---Reference Velocity (Input) - m/s
potx = simout.get('springdisp').Data;      %---Force in Spring - N
motorvel = simout.get('motorvel').Data;   %---Motor Angular Velocity - rad/s
current = simout.get('current').Data;     %---current - A
voltage = simout.get('voltage').Data;     %---voltage - V
position = simout.get('position').Data;   %---Displacement - m
torque = simout.get('torque').Data;       %---torque - Nm
Fext = simout.get('Fext').Data;           %---External Force - N
vel_pot = simout.get('vel_pot').Data;
T = simout.get('T').Data;
motorvel = motorvel* 60/(2*pi);           % rad/s => rpm Conversion
% 
% figure (1)
% plot(t, Fext)
% ylabel('Force (N)'); grid on
% xlabel('Time (s)');
% title('External Force Applied')
% legend("7.5N")



figure(2)

subplot(3,2,1)

    plot(t,motorvel); grid on, hold on
    % plot(t, motorvel/N);
    ylabel('Velocity (rpm)');   
    xlabel('Time (s)')       
    % xlim([1.5 10])
    % legend("Motor Angular Velocity", "Reel Angular Velocity")
    title('Motor Velocity') 

subplot(3,2,2)
 plot(t,T); grid on, hold on
    ylabel('Force (N)');   
    xlabel('Time (s)')      
    title('Tension') 

subplot(3,2,3)

    plot(t,vel_pot)         
    xlabel('Time (s)')      
    ylabel('Velocity (m/s)'); grid on, hold on
    title('Potentiometer Velocity')


subplot(3,2,4)

    plot(t, vel*100); grid on, hold on
    plot(t, refvel*100); 
    ylabel('Velocity (cm/s)'); 
    xlabel('Time (s)')     
    title('Mass Velocity')
    legend("Mass Velocity", "Reference")


subplot(3,2,5) 
    plot(t,voltage)
    ylabel('Voltage (V)'); grid on,hold on
    xlabel('Time (s)')      
    title('VDA Out') 

subplot(3,2,6)
    plot(t, position*100), grid on, hold on 
    ylabel('Position (cm)')
    xlabel('Time (s)')     
    title('Mass Position')

% Power = max(torque)*max(motorvel)*2*pi/60
% figure
% plot(t, motorvel/5.9*0.05-2*vel_pot); hold on
% plot(t, vel,'o')
% legend("Theory", "Simulation")