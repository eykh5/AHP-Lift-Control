clc; clear; close all

%----------------------------------------------------%
%   Converting Motor velocity controller "PIDF0" to
%   discrete form and creating a C header file for
%   embedded applications
%----------------------------------------------------%

T = 0.005;  % 5 ms

m_J = .5;           % Reel mass (kg)
r = .05;            % Reel radius (m)
J = 1/2*m_J*r^2;    % Moment of inertia for reel (cylinder)
k = 720;            % Spring stiffness (N/m)
b = 0;              % Spring damping 
m = 0.5;              % Mass (kg)
g = 9.81;           % Gravitational acceleration
A = [0,-k/2;1/(2*m),-b/(4*m)];
B = [k*r/2,0; b*r/(4*m),1/m];
C = eye(2);
D = [0,0;0,0];
sys = ss(A,B,C,D);
sys.InputName = {'\Omega','F_{ext}'};
sys.OutputName = {'f_k','v_m'};

s = tf('s');
km = 0.0507;
N = 5.9;
J_motor = 7.06e-6;
ka = .41;      % amplifer gain
b_motor = 0;
T0 = tf(1,[J_motor+J/N,b_motor]);

tau = 0.05;
wc = 2*pi/tau;
opt = pidtuneOptions('designFocus','disturbance-rejection');
[PIDF0,info0] = pidtune(ka*km*T0,'pidf',wc,opt);

PIDF0d = c2d(PIDF0, T, 'tustin');              % Discrete form
CPIDF0d = tf(PIDF0d);                       % Transfer function

% Dr.Garbini's == P:3.37, I: 10.94, D: 0.25, N:275.9
PIDF1 = pid(3.37, 10.94, 0.25, 1/275.9);

PIDF1d = c2d(PIDF1, T, 'tustin'); 
CPIDF1d = tf(PIDF1d);   

% Converting to second-order system
[NUM, DEN] = tfdata(CPIDF0d);
NUM = cell2mat(NUM); DEN = cell2mat(DEN);   % Cell to array conversion
sos = tf2sos(NUM, DEN);

[NUM1, DEN1] = tfdata(CPIDF1d);
NUM1 = cell2mat(NUM1); DEN1 = cell2mat(DEN1);   % Cell to array conversion
sos1 = tf2sos(NUM1, DEN1);



% Saving controller to header file
HeaderFileName = "PIDF0.h";
fid = fopen(HeaderFileName, 'w');
comment = "PIDF Controller: Position Control";
sos2header(fid, sos, 'PIDF0', T, comment);
fclose(fid);

HeaderFileName = "PIDF1.h";
fid = fopen(HeaderFileName, 'w');
comment = "Transfer function: Torque to motor velocity";
sos2header(fid, sos, 'PIDF1', T, comment);
fclose(fid);


function sos2header(fid, sos, name, T, comment)
  
  % Print the filter definition to a (.h) header file.
  %
  %   sos2header(fid, sos, name, T, comment)
  %
  %    fid      - File indentity
  %    sos      - Scaled second order sections, from "tf2sos"
  %    name     - Name to be given to the array of biquad structures, and
  %                  associated with the number of sections.
  %    T        - Sample period in seconds
  %    comment  - comment added at top of header
  
%---structure form of cascade

fprintf(fid,'//---%s\n', comment);
fprintf(fid,'//---%s\n', datestr(now,0));
fprintf(fid,'    char        headerTime[] = "%s";\n',datestr(now,0));
[ns,m]=size(sos);
fprintf(fid,'    int         %s_ns = %d;              // number of sections\n',name,ns);
fprintf(fid,'    uint32_t    timeoutValue = %d;      // time interval - us; f_s = %g Hz\n',T*1e6,1/T);
fprintf(fid,'    static\tstruct\tbiquad %s[]={   // define the array of floating point biquads\n',name);
for i=1:ns-1
    fprintf(fid,'        {');
    for j=[1,2,3,4,5,6]
        fprintf(fid,'%e, ',sos(i,j));
    end
    fprintf(fid,'0, 0, 0, 0, 0},\n');
end
    fprintf(fid,'        {');
    for j=[1,2,3,4,5,6]
        fprintf(fid,'%e, ',sos(ns,j));
    end
    fprintf(fid,'0, 0, 0, 0, 0}\n        };\n');
end


