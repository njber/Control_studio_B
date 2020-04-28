clc;
clear;
simu="Simulink_Arduino_PIL_v05";     % Simulink file name

%% Simulation Settings
simulate= true; % True: To simulate
Tsim = 10;        % Total Simulation length in seconds. 
                 % Set Tsim=Inf to run indefinitely                            
fs=100;          % Sampling Frequency in Hz

Ts=1/fs;         % Sampling Period

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

xo = [10 10 0 0 0 0]'; %State initial condition
xo_hat=[0 0 0 0 0 0]';          %Observer initial condition

%% Model Constant Parameters
I = 8*exp(-4);
m = 0.35;
r = 0.03;
k = 50;
k0 = 200;
b = 0.09;
b0 = 0.5;
%let cos(alpha) = a
a = sqrt(3)/2;
G = 300; %amplifier/motor voltage to torque gain.
y = 0.4;

%% Continuous-Time Linear State-Space Model
% x_dot(t) = Ac?x(t)+Bc?x(t)
%        y(t) = Cc?x(t)

n=6;    % Number of System States

Ac = [-b/I 0 r*k -r*k 0 0;
      0 -b/I -r*k r*k 0 0;
      -r/(2*I) r/(2*r) 0 0 0 a/m;
      r/I -r/I 0 0 0 0;
      0 0 0 0 0 1/m;
      0 0 2*k*a 0 -k0 -b0/m];
eigAc = eig(Ac)
  
Bc = [1 0;
      0 1;
      0 0;
      0 0;
      0 0;
      0 0];
 
Cc = [1/(2*I) 1/(2*I) 0 0 0 0;
      0 0 0 0 1 0];
  

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
os = 20;
tsettle = 5;
zeta = -log(1/100)\(sqrt(pi^2+log(1/100)^2));
wn=-log(0.02*sqrt(1-zeta^2))/(zeta*tsettle);

%Dominant second order poles
s_poles = [-zeta*wn+wn*sqrt(zeta^2-1),-zeta*wn-wn*sqrt(zeta^2-1)]

%Pc = [s_poles(1) conj(s_poles(1)) 5*real(s_poles(1)) 5.2*real(s_poles(1)) 5.4*real(s_poles(1)) 5.6*real(s_poles(1))]; 
Pc = [-0.707+0.707*i, -0.707-0.707*i, -4-4i,-4+4i, -4.2-4.2i, -4.2+4.2i]/1; 
%Pc = [-0.6 -0.6 -1.2 -1.4 -1.6 -1.8];
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
    save('result_data.mat')
    plot_results;
    disp('Done!!!')
    
end
