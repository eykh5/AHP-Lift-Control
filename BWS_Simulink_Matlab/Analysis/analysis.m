clc; close all; clearvars;

load("Pyramid.mat")
time = 0:.0001:10-.0001;

time_1 = 0:.5: 10;

title_size = 20;
axis_size = 15;
lw = 3;

figure(1);
subplot(4, 1, 1);
plot(time, -fk/2, "LineWidth", lw);
xlabel("Time (s)");
ylabel("Force (N)");
fontsize(gca, axis_size,'points')   % 'pixels', 'centimeters', 'inches'
[te, s] = title("Force in Rope");
te.FontSize = title_size;

hold on;
subplot(4, 1, 2);
plot(time, er, "LineWidth", lw);
xlabel("Time (s)");
ylabel("Position (m)");
fontsize(gca, axis_size,'points')   % 'pixels', 'centimeters', 'inches'
[te, s] = title("Mass Position");
te.FontSize = title_size;


subplot(4, 1, 3);
plot(time, vda, "LineWidth", lw);
xlabel("Time (s)");
ylabel(" VDA (Volts)");
fontsize(gca, axis_size,'points')   % 'pixels', 'centimeters', 'inches'
[te, s] = title("VDA vs. Time");
te.FontSize = title_size;

subplot(4, 1, 4);
plot(time, velocity, "LineWidth", lw);
xlabel("Time (s)");
ylabel("Velocity (m/s)");
fontsize(gca, axis_size,'points')   % 'pixels', 'centimeters', 'inches'
[te, s] = title("Velocity vs. Time");
te.FontSize = title_size;