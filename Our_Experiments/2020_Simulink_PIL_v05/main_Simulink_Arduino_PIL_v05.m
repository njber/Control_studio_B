clc;
clear;
simu="Simulink_Arduino_PIL_v05";     % Simulink file name

%% Simulation Settings
simulate= true; % True: To simulate
Tsim = 10;        % Total Simulation length in seconds. 
                        % Set Tsim=Inf to run indefinitely                            
fs=100;              % Sampling Frequency in Hz

xo=[0 0 0 0 0 0]';        % Initial System State Condition

Ts=1/fs;            % Sampling Period

linear = 1;
matlabController = 1; % else use Arduino controller
obs = 0;
PIL=0;          %0: Manually start the PIL controller 
                %   after simulation started
                %1: Automatically start PIL controller 
                %   from the beginning of the simulation

%% Initial Condition
x_o = [0 0 0 0 0 0]';                  %cart position 
xsp_o = 0;                  %cart speed
th_o = 0.1;                 %pendulum angle from vertical (up)
w_o = 0;                    %angular speed of the pendulum

xo = [0 0 0 0 0 0]'; %State initial condition
xo_hat=[0 0 0 0 0 0]';          %Observer initial condition

%% Model Constant Parameters
I1 = 8*exp(-4);
I2 = I1;
m = 0.35;
r = 0.03;
k = 50;
k0 = 200;
b1 = 0.09;
b2 = 0.09;
b0 = 0.5;
%let cos(alpha) = a
a = sqrt(2)/3;
G = 300; %amplifier/motor voltage to torque gain.
y = 0.4;

%% Continuous-Time Linear State-Space Model
% x_dot(t) = Ac?x(t)+Bc?x(t)
%        y(t) = Cc?x(t)

n=6;    % Number of System States

%Liam's A matrix
Ac=[0 1 0 0 0 0;
    -1.25*k*r^2/I1 -b1/I1 -0.5*k*r^2/I1 0 k*a/I1 0;
    0 0 0 1 0 0;
    0.5*k*r^2/I2 0 -1.25*k*r^2/I2 -b2/I2 -k*a/I2 0;
    0 0 0 0 0 1;
    k*a/m 0 -k*a/m 0 (k0 - 2*k*a^2/m) -b0/m];

%Nicks A matrix

eigAC = eig(Ac)
 
Bc=[0 0;
    1/I1 0;
    0 0;
    0 1/I2;
    0 0;
    0 0];

Cc=[0 1/2 0 1/2 0 0;
    0  0  0  0  1 0];

CO = ctrb(Ac,Bc);

if (rank(CO)==n)
    disp('System is Controllable')
else 
    disp('System is NOT Controllable')
    simulate = false;
end

%% Controller Design
sys_ct = ss(Ac, Bc, Cc, 0);
sys_dt = c2d(sys_ct, Ts);

A = sys_dt.A;
B = sys_dt.B;
C = sys_dt.C;

Pc = [-0.4 -0.5 -1.2 -1.4 -1.6 -1.8];
Pz = exp(Pc*Ts);
F=place(A, B, Pz);

eigAF = eig(A-B*F);

%% Observer Design
OM = obsv(A, C);
rank_OM=rank(OM);
if (rank_OM==n)
    disp('System is Observable')
end

POc = 5*Pc;
POz = exp(POc*Ts);

L = place(A', C', POz)';
%% Start Simulation
if (simulate)
    open_system(simu);
    disp('Simulating...')
    sim(simu)
    disp('Plotting...')
    
%     x1=x(:,1);
%     x2=x(:,2);
%     u1=u(:,1);
%     u2=u(:,2);
    save('result_data.mat')
    plot_results;
    disp('Done!!!')
    
end
