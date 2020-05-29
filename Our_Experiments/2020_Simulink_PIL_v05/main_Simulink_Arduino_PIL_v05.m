clc;
clear;
simu="Simulink_Arduino_PIL_v05";     % Simulink file name

%% Simulation Settings
simulate= true; % True: To simulate
Tsim = 1.38;        % Total Simulation length in seconds. 
                 % Set Tsim=Inf to run indefinitely                            
fs=100;          % Sampling Frequency in Hz

Ts=1/fs;         % Sampling Period

linear = 1;
closedloop = 1;
matlabController = 1; % else use Arduino controller

%% Input and Output Noise/Disturbance
du1 = 0;     %Enable input disturbace, 5*sin(2*pi*2*t)
du2 = 0;
du_freq1 = 10*pi;
du_offset1 = pi/2;
du_freq2 = 10*pi;
du_offset2 = 0;

w_noise = 0;
x_noise = 0;

obs = 1;
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
x_o = 0.02;

zo = [O1_o, O2_o, -x_o]';
z_hat_o = [O1_dot_o, O2_dot_o, x_dot_o]';
xo = [O1_dot_o, O2_dot_o, x_dot_o, O1_o, O2_o, x_o]'; %State initial condition
xo_hat=[0 0 0 0 0 0]';          %Observer initial condition



%% Continuous-Time Linear State-Space Model
% x_dot(t) = Ac?x(t)+Bc?x(t)
%        y(t) = Cc?x(t)
n = 6;
  

load('SSDClip.mat')

A = SSDClip.A
B = SSDClip.B
C = SSDClip.C


CO = ctrb(A,B);
rank(CO)

if (rank(CO)==n)
    disp('System is Controllable')
else 
    disp('System is NOT Controllable')
    simulate = false;
end

%% Controller Design
% sys_ct = ss(Ac, Bc, Cc, 0);
% sys_dt = c2d(sys_ct, Ts);
sys_dt = ss(A,B,C,0, Ts);
sys_ct = d2c(sys_dt);

Ac = sys_ct.A
Bc = sys_ct.B
Cc = sys_ct.C

%% Poles
os = 10;
tsettle = 0.0602;
zeta = 0.5; % log(os/100)\(sqrt(pi^2+log5(os/100)^2))
wn=-log(sqrt(1-zeta^2))/(zeta*tsettle)

%Dominant second order poles
s_poles = [-zeta*wn+wn*sqrt(zeta^2-1),-zeta*wn-wn*sqrt(zeta^2-1)]

Pc = [s_poles(1) conj(s_poles(1)) 4*real(s_poles(1)) 4.2*real(s_poles(1)) 4.4*real(s_poles(1)) 4.6*real(s_poles(1))]
% Pc = [-0.707+0.707*i, -0.707-0.707*i, -4-4i,-4+4i, -4.2-4.2i, -4.2+4.2i]/5 
% Pc = [-0.8+0.2i -0.8-0.2i -1.2 -1.4 -3.6 -3.8]*5;
% Pc = [-0.8 -1 -1.2 -1.4 -1.6 -1.8]*9.8;
Pz = exp(Pc*Ts);
F=place(A, B, Pz)

eigAF = eig(A-B*F)

%% Steady state control design
 y_star = [100 0.02]';
 Mo = -(Cc)*(Ac^-1)*Bc;
 Nu = Mo^-1;
 Nx = -(A^-1)*B*Nu;
 
 uss = Nu*y_star
 xss = Nx*y_star
%% Observer Design
OM = obsv(Ac, Cc);
rank_OM=rank(OM);
if (rank_OM==n)
    disp('System is Observable')

    POc = Pc;
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
