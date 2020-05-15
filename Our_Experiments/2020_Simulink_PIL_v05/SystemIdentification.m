%% Setup
simu="NonLinearPlant";

%% Initial Condition/ Operating point

y_star_w = 100;
y_star_x = 0.02;

O1_dot_o = 0;
O2_dot_o = 0;
x_dot_o = 0;
O1_o = 0;
O2_o = 0;
x_o = 0;

zo = [O1_o, O2_o, x_o]';
z_hat_o = [O1_dot_o, O2_dot_o, x_dot_o]';


uss1 = (112.5*y_star_w - 16238*y_star_x)/1250
uss2 = (112.5*y_star_w + 16238*y_star_x)/1250
   
%% Exitation
u1_step = 0.1;
u2_step = 0.1;

u1_step_time = 20;
u2_step_time = 10;

Tsim = 30;
Fs   = 100;
Ts   = 1/Fs;

%% Launch Simulation
Sim = sim(simu)
disp('Simulating...')
Data = Sim.get('Data');
