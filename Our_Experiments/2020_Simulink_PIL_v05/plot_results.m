clear;
load('result_data.mat')
y1 = y(:,1);
y2 = y(:,2);
u1 = u(:,1);
u2 = u(:,2);

w1 = x(:,1);
w2 = x(:,2);

% Input after amplifier
ug1 = u_gain(:,1);
ug2 = u_gain(:,2);

% Input disturbance
dist1 = dist(:,1);
dist2 = dist(:,2);

figure(10)
subplot(411)
plot(time,y1,'b', 'LineWidth',2)
grid
title('Speed')
legend('w1', 'w2', 'w3');

subplot(412)
plot(time,y2,'r', 'LineWidth',2)
grid
title('Tension')
legend('tension');

subplot(413)
plot(time,u1,'b', time, u2, '--r', 'LineWidth',2)
%axis([0 Tsim -11 11])
title('Control effort')
legend('u1', 'u2');
grid

subplot(414)
plot(time,w1,'b', time, w2, '--r', 'LineWidth',2)
%axis([0 Tsim -11 11])
title('Motor Speeds')
legend('w1', 'w2');
grid

%% Input Disturbance
figure(11)
plot(time, ug1, 'r', time, ug2, 'b', time, dist1, 'g', time, dist2, 'm', 'LineWidth', 2);
legend('u1', 'u2', 'dist1', 'dist2');
grid