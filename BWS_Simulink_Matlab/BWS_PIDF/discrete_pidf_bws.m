clc; close all; clearvars;

%% Plant Model

% Parameters
m_J = .2;            % Reel mass (kg)
r = .05;            % Reel radius (m)
k = 720;          % Spring stiffness (N/m)
m = 1;             % Mass (kg)
g = 9.81;
N = 5.9;
J = 1/2*m_J*r^2;    % Moment of inertia for cylinder
ka = .41;          % amplifer gain
km = .0507;         % motor gain 

% State Model
A = [0,-r/(2*J),0;k*r/2,0,-k/2;0,1/(2*m),0];
B = [1/J,0;0,0;0,1/m];
C = [1,0,0;0,1,0;0,0,1;0,-r/(2*J),0;k*r/2,0,-k/2;0,1/(2*m),0];
D = [0,0;0,0;0,0;1/J,0;0,0;0,1/m];

C2 = [1,0,0;0,1,0;0,0,1];
D2 = [0,0;0,0;0,0];

stateModel = ss(A,B,C2,D2);
stateModel.InputName = {'T_m','F_{ext}'};
% stateModel.OutputName = {'\Omega_J','f_k','v_m','\Omega_J^\prime','f_k^\prime','v_m^\prime'};
stateModel.OutputName = {'\Omega_J','f_k','v_m'};

% Transfer Function (T_m to f_k)
sys = tf(stateModel);
T1 = sys(2,1);

% Transfer Function (T_m to f_K')
% T2 = sys(5,1);

% Transfer Function (f_ext to f_k)
T3 = sys(2,2);

% Transfer Function(T_m to v_m)
T4 = sys(3, 1);

% Transfer Function(T_m to v_m)
T5 = sys(3, 2);

% Transfer Function (T_m to Omega_j)
T6 = sys(1, 1);

% Transfer Function (f_ext to omega_j)
T7 = sys(1, 2);

%% Controller Design
close all


% Target
PO = 10;           % Percent overshoot
z = -log(PO/100)/sqrt(pi^2+log(PO/100)^2);
ts = 1;            % Settling time
zw = 4/ts;

s = tf('s');

BTI = .005;

% C = PIDtune(T1, "PIDF");

% pzmap(C);
% return;

discrete_t = linspace(0,10,10/BTI+1);

color1 = ["y", "c"];
color = ["r", "b"];

sample_time = linspace(0, 10, 2001);
for c = 1:3
    u = -c * .33;
    x0 = [0, 0, 0];
    if u == -.99
        u = -.90;
    end

    simout = sim("BWS_discrete_PIDF.slx");
    
    yout = simout.yout;
    t = simout.tout;
    % f_k = yout(:,1);
    % f_ext = yout(:,2);
    % volt = yout(:,3);
    % curr = yout(:,4);
    % tor_m = yout(:,5);
    % error = yout(:, 6);
    % v_mass = yout(:, 7);
    % motor_rot = yout(:, 8);
    f_t = yout{1}.Values.Data;
    f_ext = yout{2}.Values.Data;
    volt = yout{3}.Values.Data;
    curr = yout{4}.Values.Data;
    tor_m = yout{5}.Values.Data;
    err = yout{6}.Values.Data;
    v_mass = yout{7}.Values.Data;
    x_mass = yout{8}.Values.Data;
    f_dist = yout{9}.Values.Data;
    omega_j = yout{10}.Values.Data;
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
    hold on;

    figure(1);
    subplot(2,3,2);
    plot(t, f_t, 'LineWidth', lw);
    ylabel("F_t(N)");
    xlabel("time(s)");
    fontsize(gca, axis_size,'points')   % 'pixels', 'centimeters', 'inches'
    [te,s] = title("time vs. f_t");
    te.FontSize = title_size;
    grid on;
    hold on;

    figure(1);
    subplot(2,3,3);
    plot(t,v_mass, 'LineWidth', lw);
    ylabel("v_m (m/s)");    
    xlabel("time(s)");
    fontsize(gca, axis_size,'points')   % 'pixels', 'centimeters', 'inches'
    [te,s] = title("time vs. v_m");
    te.FontSize = title_size;
    grid on;
    hold on;

    figure(1);
    subplot(2,3,5);
    plot(t, curr, 'LineWidth', lw);
    ylabel("Current (A)")
    xlabel("time(s)");
    fontsize(gca, axis_size,'points')   % 'pixels', 'centimeters', 'inches'
    [te,s] = title("time vs. voltage");
    te.FontSize = title_size;
    grid on;
    hold on;

    figure(1);
    subplot(2,3,4);
    plot(t, x_mass, 'LineWidth', lw);
    ylabel("x_m (m)");
    xlabel("time(s)");
    fontsize(gca, axis_size,'points')   % 'pixels', 'centimeters', 'inches'
    [te,s] = title("time vs. x_m");
    te.FontSize = title_size;
    grid on;
    hold on;

    figure(1);
    subplot(2,3,6);
    % plot(sample_time,err);
    plot(t, f_dist, 'LineWidth', lw);
    ylabel("f_d (N)");
    xlabel("time(s)");
    fontsize(gca, axis_size,'points')   % 'pixels', 'centimeters', 'inches'
    [te,s] = title("time vs. f_d");
    te.FontSize = title_size;
    grid on;
    hold on;


end