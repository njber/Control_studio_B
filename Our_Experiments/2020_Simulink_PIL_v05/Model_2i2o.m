%% Multivariable System Identification
%
% Ricardo P. Aguilera
% UTS, Sydney, NSW, Australia
% Oct 2018
%
% ?     ?   ?                ??     ?
%?|Y1(s)|   |G11(s)  G12(s)  ||U1(s)|
% |     | = |                ||     |
% |Y2(s)|   |G21(s)  G22(s)  ||U2(s)|
% ?     ?   ?                ??     ?
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear

%% Sampling frequency
fs=100;   
Ts=1/fs;    %sampling time

%% TF equations
I1 = 8*exp(-4);
I2 = I1;
I=I1;
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

s=tf('s'); 

Gw = 1/(2*(I*s+b0)); %* 0.001
Gx = (sqrt(3)*k*r)/((I*s^2+3*k*r^2)*(2*m*s^2 + 2*k0+3*k)-3*k^2*r^2)
%% Tramsfer Functions
% s=tf('s');

% Gw = (1/(0.3*s+1))/0.001;
% Gx = (-185600/((s^2+11*s+150)*(s^2+1.6*s+800)))/10;
% 
% Ksen = [0.001 0;
%         0 10];

G11s = Gw;   
G21s = -Gx;
G12s = Gw;
G22s = Gx;

%Format required by Simulink
[num11,den11] = tfdata(G11s,'v');
[num21,den21] = tfdata(G21s,'v');
[num12,den12] = tfdata(G12s,'v');
[num22,den22] = tfdata(G22s,'v');


%% Runing Simulation to generate data
sim('sys_2i2o.slx');
% sim('NonLinearPlant.slx');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Identifyng Individual Transfer Functions

%% Read data
Nini=900;   %Only the length of data for a change in u1
Nf=3000;   % from 5seconds to 12seconds
y11=y1(Nini:Nf);
y21=y2(Nini:Nf);
u=u1(Nini:Nf);

clip = iddata(900:3000)

%% plot
subplot(311)
plot(time(Nini:Nf),y11)
ylabel('output y1')
subplot(312)
plot(time(Nini:Nf),y21)
ylabel('output y2')
subplot(313)
plot(time(Nini:Nf),u)
ylabel('input u')
xlabel('Time [s]')

%% Get TF G11  Y1(s)/U1(s)
display('Identified indiviual TFs');
data = iddata(y11,u,Ts);      %data format required by tfest;
np=1;                               %Number of poles
sys11 = tfest(data,np)        %Estimated transfer function
%% Get TF G21  Y2(s)/U1(s)
data = iddata(y21,u,Ts);      %data format required by tfest;
np=1;                                %Number of poles
sys21 = tfest(data,np)         %Estimated transfer function
sys21;

%% Identifying all trnsfer functions at once
%  Here, the whole length of data is considered
disp('~');
disp('****************************');
disp('Identified all 4 TFs at once');
data = iddata([y1,y2],[u1,u2],Ts)
np=[1 1;1 4];                                 %Number of poles
sys_all = tfest(data,np);                     %Estimated transfer function

%% Comparison
disp('~')
disp('****************************')
disp('Comparison G11(s)')
sys11=tf(sys11)
sys11_all=tf(sys_all(1,1))
%sys11_all.Report.Fit.FitPercent
disp('~')
disp('****************************')
disp('Comparison G21(s)')
sys21=tf(sys21)
sys21_all=tf(sys_all(2,1))

%% State-Space Identification

ns=6;    % number of states
SSData = n4sid(data,ns)

