clc;
clear;
simu="Simulink_Arduino_PIL_v05";     % Simulink file name

%% Simulation Settings
simulate= true; % True: To simulate
Tsim = 3;        % Total Simulation length in seconds. 
                 % Set Tsim=Inf to run indefinitely                            
fs=500;          % Sampling Frequency in Hz

Ts=1/fs;         % Sampling Period

linear = 1;
closedloop = 1;
matlabController = 1; % else use Arduino controller

du = 1;     %Enable input disturbace, 5*sin(2*pi*2*t)
du_freq = pi;

obs = 0;
PIL=0;          %0: Manually start the PIL controller 
                %   after simulation started
                %1: Automatically start PIL controller 
                %   from the beginning of the simulation
                
              

%% Model Constant Parameters
I = 0.0008;
I1 = I;
I2 = I;
m = 0.35;
r = 0.03;
k = 50;
k0 = 200;
b = 0.09;
b1 = b;
b2 = b;
b0 = 0.5;
%let cos(alpha) = a
a = sqrt(3)/2;
G = 300; %amplifier/motor voltage to torque gain.
y = 0.4;                
                
%% Initial Condition

O1_dot_o = 0;
O2_dot_o = 0;
x_dot_o = 0;
O1_o = 0;
O2_o = 0;
x_o = 0;

zo = [O1_o, O2_o, x_o]';
z_hat_o = [O1_dot_o, O2_dot_o, x_dot_o]';
xo = [O1_dot_o, O2_dot_o, x_dot_o, O1_o, O2_o, x_o]'; %State initial condition
xo_hat=[0 0 0 0 0 0]';          %Observer initial condition



%% Continuous-Time Linear State-Space Model
% x_dot(t) = Ac?x(t)+Bc?x(t)
%        y(t) = Cc?x(t)
n = 6;
  

load('linsys.mat')

Ac = linsys1.A
Bc = linsys1.B
%Cc = linsys1.C
Cc = [0.5 0.5 0 0 0 0;
      0 0 0 0 0 1];


CO = ctrb(Ac,Bc);
rank(CO)

if (rank(CO)==n)
    disp('System is Controllable')
else 
    disp('System is NOT Controllable')
    simulate = false;
end

%% Controller Design
sys_ct = ss(Ac, Bc, Cc, 0);
sys_dt = c2d(sys_ct, Ts);

A = sys_dt.A
B = sys_dt.B
C = sys_dt.C

%% Poles
os = 1;
tsettle = 2;
zeta = -log(os/100)\(sqrt(pi^2+log(os/100)^2))
wn=-log(0.02*sqrt(1-zeta^2))/(zeta*tsettle)

%Dominant second order poles
s_poles = [-zeta*wn+wn*sqrt(zeta^2-1),-zeta*wn-wn*sqrt(zeta^2-1)]

Pc = [s_poles(1) conj(s_poles(1)) 4*real(s_poles(1)) 4.2*real(s_poles(1)) 4.4*real(s_poles(1)) 4.6*real(s_poles(1))]
Pc = [-0.707+0.707*i, -0.707-0.707*i, -4-4i,-4+4i, -4.2-4.2i, -4.2+4.2i]*8 
%Pc = [-0.8 -1 -1.2 -1.4 -1.6 -1.8]*10;
Pz = exp(Pc*Ts);
F=place(A, B, Pz)

eigAF = eig(A-B*F)

%%Just to see
Fc = place(Ac, Bc, Pc)
eigAFc = eig(Ac-Bc*Fc)

%% Observer Design
OM = obsv(Ac, Cc);
rank_OM=rank(OM);
if (rank_OM==n)
    disp('System is Observable')

    POc = 10*Pc;
    POz = exp(POc*Ts);

    L = place(A', C', POz)'
else
    disp('System is NOT Observable')
    L = zeros(2,6);
end
%% Start Simulation
if (simulate)
    open_system(simu);
    disp('Simulating...')
    sim(simu)
    disp('Plotting...')
    save('result_data.mat')
    plot_results;
    disp('Done!!!')
    
end
