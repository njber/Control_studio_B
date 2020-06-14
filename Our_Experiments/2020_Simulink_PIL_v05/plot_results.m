clear;
load('result_data.mat')
y1 = y(:,1);
y2 = y(:,2);
u1 = u(:,1);
u2 = u(:,2);

%Noisy Outputs
y_n1 = noisy_y(:,1);
y_n2 = noisy_y(:,2);

%Estimator states (In terms of output only)
y_hat1 = y_hat(:,1);
y_hat2 = y_hat(:,2);

%Sensor and Controller Voltages
u_v1 = u_v(:,1);
u_v2 = u_v(:,2);
y_v1 = y_v(:,1);
y_v2 = y_v(:,2);

w1 = x(:,1);
w2 = x(:,2);

% Input after amplifier
% ug1 = u_gain(:,1);
% ug2 = u_gain(:,2);

% Input disturbance
dist1 = dist(:,1);
dist2 = dist(:,2);


if(reference==10)
    y1_r = y_ref(1);
    y1_p = y_ref(1) * 1.1; % +10%
    y1_m = y_ref(1) * 0.9; % -10%

    y2_r = 1000*y_ref(2);
    y2_p = 1000*y_ref(2) * 1.1; % +10%
    y2_m = 1000*y_ref(2) * 0.9; % -10%
elseif(reference==5)
    y1_r = y_ref(1);
    y1_p = y_ref(1) * 1.05; % +5%
    y1_m = y_ref(1) * 0.95; % -5%

    y2_r = 1000*y_ref(2);
    y2_p = 1000*y_ref(2) * 1.05; % +5%
    y2_m = 1000*y_ref(2) * 0.95; % -5%
end

switch(controller)
    case 1
        Title='SFC';
    case 2
        Title='LQR';
    case 3
        Title='SMC';
    case 4
        Title='MPC';
end

if(integralaction==1)
    Title = append(Title, ' with integral action');
end

Controller_str = {Title};

figure('Position', 1.4*[0, 0, 800, 600])
sgtitle(Controller_str)
subplot(311)
hold on

p1 = plot(time, y_hat1, 'b');
if(noise==1)
    p2 = plot(time, y_n1, 'c');
end
p3 = plot(time,y1,'r', 'LineWidth',1.2);

if(ref==1)
    yline(y1_p,'--')
    yline(y1_m,'--')
    yline(y1_r,'g')
end


set(gca,'GridLineStyle','--')
grid
title('Jockey Wheel Speed')
ylabel('Speed (rad/s)','fontsize',16,'interpreter','latex')
hold off
% strr = {'$\hat{\omega}$', 'noisy $\omega$' '$\omega$','fontsize',16,'interpreter','latex'};
if(noise==1)
    strr = {'$\hat{\omega}$', 'noisy $\omega$' '$\omega$'};
    legend([p1 p2 p3], strr, 'fontsize',16,'interpreter','latex');
else
    strr = {'$\hat{\omega}$', '$\omega$'};
    legend([p1 p3], strr, 'fontsize',16,'interpreter','latex');
end

subplot(312)

hold on
p1 = plot(time, y_hat2*1000, 'b');
if(noise==1)
    p2 = plot(time, y_n2*1000, 'c');
end
p3 = plot(time,y2*1000,'r', 'LineWidth',1.2);
if(ref==1)
    yline(y2_p,'--')
    yline(y2_m,'--')
    yline(y2_r,'g')
end
set(gca,'GridLineStyle','--')
grid
title('Tension Arm Displacement')
ylabel('Displacement $(mm)$','fontsize',16,'interpreter','latex')
hold off
if(noise==1)
    strr = {'$\hat{x}$', 'noisy $x$', '$x$'};
    legend([p1 p2 p3], strr, 'fontsize',16,'interpreter','latex');
else
    strr = {'$\hat{\omega}$','$x$'};
    legend([p1 p3], strr, 'fontsize',16,'interpreter','latex');
end
% legend('$\hat{x}$', 'noisy $x$', '$x$', '+lim', '-lim','fontsize',16,'interpreter','latex');


subplot(313)
plot(time,u1,'b', time, u2, 'r', 'LineWidth',1.2)
%axis([0 Tsim -11 11])
title('Control effort')
legend('$u_1$', '$u_2$','fontsize',16,'interpreter','latex');
set(gca,'GridLineStyle','--')
grid
ylabel('Torque $(Nm)$','fontsize',16,'interpreter','latex')

xlabel('Time (s)','fontsize',16,'interpreter','latex')

%% Plot Voltage ranges
figure('Position', 1.4*[0, 0, 800, 600])
sgtitle(Controller_str)
subplot(211)
plot(time, u_v1, 'r', time, u_v2, 'b','LineWidth',1.2);
legend('$u_1$', '$u_2$','fontsize',16,'interpreter','latex');
set(gca,'GridLineStyle','--')
grid on
title('Control Effor & Sensor Voltages')
ylabel('Voltage $(V)$','fontsize',16,'interpreter','latex')

subplot(212)
plot(time, y_v1, 'r', time, y_v2, 'b','LineWidth',1.2);
legend('$\omega$', '$x$','fontsize',16,'interpreter','latex');
set(gca,'GridLineStyle','--')
grid on
xlabel('Time (s)','fontsize',16,'interpreter','latex')
ylabel('Voltage $(V)$','fontsize',16,'interpreter','latex')


%% Input Disturbance
% figure(11)
% plot(time, ug1, 'r', time, ug2, 'b', time, dist1, 'g', time, dist2, 'm', 'LineWidth', 2);
% legend('u1', 'u2', 'dist1', 'dist2');
% title('Input Disturbance')
% ylabel('Torque [Nm]')
% grid