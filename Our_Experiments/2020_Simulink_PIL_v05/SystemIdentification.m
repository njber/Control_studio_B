%% Setup
simu="NonLinearPlant";

%% Initial Condition/ Operating point

y_star_w = 100;
y_star_x = 0.02;

O1_dot_o = y_star_w;
O2_dot_o = y_star_w;
x_dot_o = 0;
O1_o = 0;
O2_o = 0;
x_o = y_star_x;

zo = [O1_o, O2_o, x_o]';
z_hat_o = [O1_dot_o, O2_dot_o, x_dot_o]';


uss1 = (112.5*y_star_w - 16238*y_star_x)/1250
uss2 = (112.5*y_star_w + 16238*y_star_x)/1250
   
%% Exitation
u1_step = 1;
u2_step = 1;

u1_step_time = 10;
u2_step_time = 20;

Tsim = 30;
Fs   = 1000;
Ts   = 1/Fs;

%% Launch Simulation
Sim = sim(simu)
disp('Simulating...')
Data = Sim.get('Data');
