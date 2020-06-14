clc;
clear;
simu="Simulink_Arduino_PIL_v05";     % Simulink file name

%% Simulation Settings
simulate= true;       % True: To simulate
Tsim = 5;             % Total Simulation length in seconds.                           
fs=100;               % Sampling Frequency in Hz
Ts=1/fs;              % Sampling Period

noise = 0;            % Aplies +-10% noise per unit for w =100 and x = 0.02
linear = 0;           % Plant selection
closedloop = 1;       % Open/closed loop selection
obs = 2;              % No observer: 0, Luenberger: 1, Kalman: 2
controller = 2;       % SFC: 1, LQR: 2, SMC: 3, MPC:4
integralaction = 1;   % on:1; off:0
matlabController = 1; % else use Arduino controller
PIL=1;                %0: Manually start the PIL controller 
                      %   after simulation started
                      %1: Automatically start PIL controller 
                      %   from the beginning of the simulation
                      
N = 10;                      
                      
% Set reference
y_star = [100 0.02]';
y_ref = [y_star(1) y_star(2)]';

F = zeros(2,6);

%% Data visualisation
%refline variables
ref = 1; % reference tolerance band on or off
reference = 5; %10 = 10%; 5 = 5%

%% Input and Output Noise/Disturbance
du1 = 0;     %Enable input disturbace
du2 = 0;
du_freq1 = 10*pi;
du_offset1 = pi/2;
du_freq2 = 10*pi;
du_offset2 = 0;

du1_bias = 1; %DC bias for sinusoidal input
du2_bias = 5; %DC bias for sinusoidal input

w_noise = 0;
x_noise = 0;

%% Input and State Constraints for MPC
% not required for LQR
% change these constraints as required
value = 300;
umin=[-value,-value]';
umax=[value,value]';

value2 = 10000;
xmin=[-value2;-value2;-value2;-value2;-value2;-value2];    %Large number implies no constraint
xmax=[value2;value2;value2;value2;value2;value2];       %Large number implies no constraint

%% Model Constant Parameters
% Most parameters declared in Non-linear Plant in Simulink
G = 30; %amplifier/motor voltage to torque gain.
w_gain = 1/100;
x_gain = 100;
           
                
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

%% Augmented System for integral action
O_21 = [0 0 0 0 0 0;
    0 0 0 0 0 0]';
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

Q_aug = [0.0086 0 0 0 0 0 0 0;
    0 0.0349 0 0 0 0 0 0;
    0 0 9.4695e-04 0 0 0 0 0;
    0 0 0 2.1372e-05 0 0 0 0;
    0 0 0 0 2.5783e-05 0 0 0;
    0 0 0 0 0 1.0699e-05 0 0;
    0 0 0 0 0 0 10000 0;
    0 0 0 0 0 0 0 10000];

R_aug = 0.001*eye(2);

% N = [0 0 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 0]';
% check = [Q_aug N;
%     N' R];
%     
% try chol(check)
%     disp('Matrix is symmetric positive definite.')
% catch ME
%     disp('Matrix is not symmetric positive definite')
% end


%% Controller Design

% Controller 1 : Poles
os = 0.3;
tsettle = 5;
zeta = 0.707; %log(os/100)\(sqrt(pi^2+log(os/100)^2))
wn=-log(sqrt(1-zeta^2))/(zeta*tsettle);

% Dominant second order poles
s_poles = [-zeta*wn+wn*sqrt(zeta^2-1),-zeta*wn-wn*sqrt(zeta^2-1)];

Pc = [s_poles(1) conj(s_poles(1)) 4*real(s_poles(1)) 4.2*real(s_poles(1)) 4.4*real(s_poles(1)) 4.6*real(s_poles(1))];
Pc = [-0.7 -0.71 -1.9 -1.81 -1.82 -1.83]*5;
  
% Discrete EigenValues
Pz = exp(Pc*Ts);

if (controller == 1) 
  F=place(A, B, Pz);
end


% LQR for augmented system
p3=[-0.8 -1 -1.2 -1.4 -1.6 -1.8 -2 -2.2]*5;

K_aug=dlqr(A_aug,B_aug,Q_aug,R_aug);

a = [K_aug(1) K_aug(3) K_aug(5) K_aug(7) K_aug(9) K_aug(11)]; % option here to tidy up (Nick)
b = [K_aug(2) K_aug(4) K_aug(6) K_aug(8) K_aug(10) K_aug(12)];
Fi = [a;
    b];

Ki=[K_aug(13) K_aug(15);
    K_aug(14) K_aug(16)];

% LQR Controller Design
Qy=[1 0;
    0 1];
Q=C'*Qy*C;
R=0.00001*eye(2);

if (controller ==2)
  F=dlqr(A,B,Q,R);
end

%% Steady state control design
 Mo = -(Cc)*(Ac^-1)*Bc;
 Nu = Mo^-1;
 Nx = -(Ac^-1)*Bc*Nu;
 
   if (controller == 4)
      y_star(1,1) = y_star(1,1)*3.075;
      y_star(2,1) = y_star(2,1)*1.28;
   end

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

  
   %Rf=eye(2);
   Rf=[31.5429 0;
       0 1.2617e-6];
   Qf=0.0005*eye(6);
   [Pf,po_dt,Kf_t] = dare(A',C',Qf,Rf,[],[]);
   
   if (obs == 2)
     L=Kf_t';
   end
else
  disp('System is NOT Observable')
  L = zeros(2,6);
end

%% MPC Design
n=6;
m=2;
[K,P]=dlqr(A,B,Q,R);
[W,Fm,Phi,Lambda] = MPC_matrices(A,B,Q,R,P,N);


if (N<1)
N=1;
end
Umax=[];
Umin=[];
Xmax=[];
Xmin=[];
for k=1:N
    Umax=[Umax;umax];
    Umin=[Umin;umin];

    Xmax=[Xmax;xmax];
    Xmin=[Xmin;xmin];
end



%% Inequality constraint  AN*U(k) < bN
% This matrix is correct, provided you have properly computed Phi
% Therefore, do not change it.
INm=eye(N*m);
aN=[INm;
   -INm;
    Phi;
   -Phi];
% bN must be computed inside the controller


%% Design SMC
% Desing surface Cs and switching gain gamma

Cs1=[41 0 0 0 0 0.1];
Cs2=[0 0 0 10 0 0];
Cs=[Cs1;
    Cs2];
CsRank = rank(Cs);
gamma=0.01;
Keq=(Cs*B)^-1 * Cs*A;
Ksw=gamma*(Cs*B)^1;

if(controller == 3)
    F=dlqr(A,B,Q,R);
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