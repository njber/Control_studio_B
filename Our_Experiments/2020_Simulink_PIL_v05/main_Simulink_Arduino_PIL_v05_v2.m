clc;
clear;
simu="Simulink_Arduino_PIL_v05";     % Simulink file name

%% Simulation Settings
simulate= true;       % True: To simulate
Tsim = 5;             % Total Simulation length in seconds.                           
fs=100;               % Sampling Frequency in Hz
Ts=1/fs;              % Sampling Period

linear = 1;           % Plant selection
closedloop = 1;       % Open/closed loop selection
obs = 2;              % No observer: 0, Luenberger: 1, Kalman: 2
controller = 2;       % SFC: 1, LQR: 2, SFCwithIntegral :4;
integralaction = 1;    % on:1; off:0
matlabController = 1; % else use Arduino controller
PIL = 1;                %0: Manually start the PIL controller 
                      %   after simulation started
                      %1: Automatically start PIL controller 
                      %   from the beginning of the simulation
% Set reference
y_star = [100 0.02]';

%% Input and Output Noise/Disturbance
du1 = 0;     %Enable input disturbace, 5*sin(2*pi*2*t)
du2 = 0;
du_freq1 = 1*pi; %10*pi;
du_offset1 = pi/2;
du_freq2 = 1*pi; %10*pi;
du_offset2 = 0;

U_dist = 1; %??

u1_bias = 0;
u2_bias = 0;

w_noise = 0; % process noise
x_noise = 0; % measurment noise

w_ss = 0; %constant error
x_ss = 0; %constant error

%% Model Constant Parameters
% Most parameters declared in Non-linear Plant in Simulink
G = 300; %amplifier/motor voltage to  torque gain.
           
                
%% Initial Condition
O1_dot_o = 0;
O2_dot_o = 0;
x_dot_o = 0;
O1_o = 0;
O2_o = 0;
x_o = 0;

zo = [O1_o, O2_o, -x_o]';
z_hat_o = [O1_dot_o, O2_dot_o, x_dot_o]';
xo = [O1_dot_o, O2_dot_o, x_dot_o, O1_o, O2_o, x_o]'; %State initial condition
xo_hat=[0 0 0 0 0 0]';                                %Observer initial condition



%%Linear State-Space Model
n = 6; %Number of states

% Load in discrete time state space model
load('SSDClip.mat')
A = SSDClip.A;
B = SSDClip.B;
C = SSDClip.C;

% Check Controllability
CO = ctrb(A,B);
rank(CO)

if (rank(CO)==n)
    disp('System is Controllable')
else 
    disp('System is NOT Controllable')
    simulate = false;
end

% Obtain continuous time state space model
sys_dt = ss(A,B,C,0, Ts);
sys_ct = d2c(sys_dt);

Ac = sys_ct.A;
Bc = sys_ct.B;
Cc = sys_ct.C;

%% Controller Design

% Controller 1 : Poles
os = 0.3;
tsettle = 5;
zeta = 0.707; %log(os/100)\(sqrt(pi^2+log(os/100)^2))
wn=-log(sqrt(1-zeta^2))/(zeta*tsettle)

% Dominant second order poles
s_poles = [-zeta*wn+wn*sqrt(zeta^2-1),-zeta*wn-wn*sqrt(zeta^2-1)]

% Pc = [s_poles(1) conj(s_poles(1)) 4*real(s_poles(1)) 4.2*real(s_poles(1)) 4.4*real(s_poles(1)) 4.6*real(s_poles(1))]
Pc = [-0.8 -1 -1.2 -1.4 -1.6 -1.8]*3;
  
% Discrete EigenValues
Pz = exp(Pc*Ts);

if (controller == 1) 
  F=place(A, B, Pz);
end

% Augmented System
O_21 = [0 0 0 0 0 0;
    0 0 0 0 0 0]';
% O_22 = [0 0;
%     0 0]';
O_22 = eye(2);
O_B = [0 0;
    0 0];
A_aug=[A O_21; 
       C O_22];
B_aug=[B;
       O_B];
O_C = [0 0;
    0 0];
C_aug = [C O_C];
   
   
Qy_aug=[0.1 0;
    0 1];
% Q_aug=C_aug'*Qy_aug*C_aug;
Z  = [0 0 0 0 0 0; 
    0 0 0 0 0 0];
% X = zeros(2);
% Q_aug = [QL Z';
%     Z X];
% Q_aug = eye(8);
% Q_aug = [0.0086   -0.0165    0.0027   -0.0004   -0.0005   -0.0003 0 0;
%    -0.0165    0.0349   -0.0057    0.0007    0.0009    0.0005 0 0;
%     0.0027   -0.0057    0.0009   -0.0001   -0.0002   -0.0001 0 0;
%    -0.0004    0.0007   -0.0001    0.0000    0.0000    0.0000 0 0;
%    -0.0005    0.0009   -0.0002    0.0000    0.0000    0.0000 0 0;
%    -0.0003    0.0005   -0.0001    0.0000    0.0000    0.0000 0 0;
%    0 0 0 0 0 0 0.001 0
%    0 0 0 0 0 0 0 0.001];

Q_aug = [0.0086 0 0 0 0 0 0 0;
    0 0.0349 0 0 0 0 0 0;
    0 0 9.4695e-04 0 0 0 0 0;
    0 0 0 2.1372e-05 0 0 0 0;
    0 0 0 0 2.5783e-05 0 0 0;
    0 0 0 0 0 1.0699e-05 0 0;
    0 0 0 0 0 0 1000 0;
    0 0 0 0 0 0 0 1000];
% Q_aug = 0.000001*eye(8);
R = 0.001*eye(2);
% R=[1 0
%     0 1];
% QL = eye(8)
N = [0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0]';
check = [Q_aug N;
    N' R];
    
try chol(check)
    disp('Matrix is symmetric positive definite.')
catch ME
    disp('Matrix is not symmetric positive definite')
end

 %% SFC for augmented system
p3=[-0.8 -1 -1.2 -1.4 -1.6 -1.8 -2 -2.2]*5;

% K_aug=place(A_aug,B_aug,p3);
% [Ki,S,CLP] = lqi(sys_dt,Q_aug,R);
if(integralaction==1)
    K_aug=dlqr(A_aug,B_aug,Q_aug,R);
    % Fi=K_aug(1:2);
    a = [K_aug(1) K_aug(3) K_aug(5) K_aug(7) K_aug(9) K_aug(11)];
    b = [K_aug(2) K_aug(4) K_aug(6) K_aug(8) K_aug(10) K_aug(12)];
    Fi = [a;
        b];
% Fi=K_aug(1:12);
% Ki=K_aug(3);

    Ki=[K_aug(13) K_aug(15);
        K_aug(14) K_aug(16)];
else
    Fi=0;
    Ki=0;
end



%% LQR Controller Design
Qy=[0.1 0;
    0 1];
Q=C'*Qy*C;
% Q=0.0001*eye(6);
R=1*eye(2);

if(controller==2)
    F=dlqr(A,B,Q,R);
end



% [K,S,e] = lqi(SYS,Q,R,N); %LQI controller



%% Steady state control design
 Mo = -(Cc)*(Ac^-1)*Bc;
 Nu = Mo^-1;
 Nx = -(Ac^-1)*Bc*Nu;
 
 uss = Nu*y_star;
 xss = Nx*y_star;
%% Observer Design
OM = obsv(Ac, Cc);
rank_OM=rank(OM);
if (rank_OM==n)
  disp('System is Observable')

  POc = 5*Pc;       % Set Luenberger Observer Speed
  POz = exp(POc*Ts); % Discretise

  if (obs == 1)
    L = place(A', C', POz)';
  end
  
  
   cov1 = 8.9809; %OL simulation
%    cov1 = 17.5015; %OL simulation
%    cov1 = 9.0465; %OL simulation
   cov2 = 8.8837; %OL simulation
%    cov2 = 0.0284; %OL simulation
   
%    Rf=eye(2);
   Rf=[cov1 0;
       0 cov2];
   Qf=0.00001*[1 0 0 0 0 0;
       0 1000 0 0 0 0;
       0 0 1 0 0 0;
       0 0 0 1 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1];
%    Qf=0.000001*eye(6); % initial
   [Pf,po_dt,Kf_t] = dare(A',C',Qf,Rf,[],[]);
   
   if (obs == 2)
     L=Kf_t';
   end
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