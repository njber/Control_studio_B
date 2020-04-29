clc;
clear;
simu="Simulink_Arduino_PIL_v05";     % Simulink file name

%% Simulation Settings
simulate= true; % True: To simulate
Tsim = 120;        % Total Simulation length in seconds. 
                 % Set Tsim=Inf to run indefinitely                            
fs=25;          % Sampling Frequency in Hz

Ts=1/fs;         % Sampling Period

linear = 1;
closedloop = 1;
matlabController = 1; % else use Arduino controller
obs = 0;
PIL=0;          %0: Manually start the PIL controller 
                %   after simulation started
                %1: Automatically start PIL controller 
                %   from the beginning of the simulation

%% Model Constant Parameters
I = 8*exp(-4);
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
w1o = 0;
h1o = w1o*I;
w2o = 0;
h2o = w1o*I;
xco = 0;
xeo = 0;
x1o = 0;
po = 0;
xo = [h1o h2o xco xeo x1o po]'; %State initial condition
xo_hat=[0 0 0 0 0 0]';          %Observer initial condition



%% Continuous-Time Linear State-Space Model
% x_dot(t) = Ac?x(t)+Bc?x(t)
%        y(t) = Cc?x(t)
n = 6;

Ac = [-b1*I1 -(3/2)*k*r^2*I1 0 (3/2)*k*r^2*I1 0 (sqrt(3)/2)*k*r*I1; %O1_ddot 
    1 0 0 0 0 0; %O1_dot
    0 (3/2)*k*r^2*I2 -b1*I2 -(3/2)*k*r^2*I2 0 -(sqrt(3)/2)*k*r*I2;%O2_ddot
    0 0 1 0 0 0; %O2_dot
    0 (sqrt(3)/2)*k*r*m 0 (sqrt(3)/2)*k*r*m -b0*m -m;%x_ddot
    0 0 0 0 1 0]%x_dot

Bc = [I1 0 0 0 0 0;
    0 I2  0 0 0 0]'
  
Cc = [0.5 0 0.5 0 0 0;
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

A = sys_dt.A;
B = sys_dt.B;
C = sys_dt.C;

%% Poles
os = 9;
tsettle = 5.5;
zeta = -log(os/100)\(sqrt(pi^2+log(os/100)^2))
wn=-log(0.02*sqrt(1-zeta^2))/(zeta*tsettle)

%Dominant second order poles
s_poles = [-zeta*wn+wn*sqrt(zeta^2-1),-zeta*wn-wn*sqrt(zeta^2-1)]

Pc = [s_poles(1) conj(s_poles(1)) 4*real(s_poles(1)) 4.2*real(s_poles(1)) 4.4*real(s_poles(1)) 4.6*real(s_poles(1))]; 
%Pc = [-0.707+0.707*i, -0.707-0.707*i, -4-4i,-4+4i, -4.2-4.2i, -4.2+4.2i]/8.5; 
%Pc = [-0.8 -1 -1.2 -1.4 -1.6 -1.8];
Pz = exp(Pc*Ts);
F=place(A, B, Pz);

eigAF = eig(A-B*F)

%%Just to see
Fc = place(Ac, Bc, Pc);
eigAFc = eig(Ac-Bc*Fc)

%% Observer Design
OM = obsv(A, C);
rank_OM=rank(OM);
if (rank_OM==n)
    disp('System is Observable')

    POc = 10*Pc;
    POz = exp(POc*Ts);

    L = place(A', C', POz)';
else
    disp('System is NOT Observalbe')
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
