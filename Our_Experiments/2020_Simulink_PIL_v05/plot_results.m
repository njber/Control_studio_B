clear;
load('result_data.mat')
y1 = y(:,1);
y2 = y(:,2);
y3 = y(:,3);
u1 = u(:,1);
u2 = u(:,2);

w1 = x(:,1);
w2 = x(:,3);

figure(10)
subplot(411)
plot(time,y1,'b', time, y2, 'r', time, (y1 + y2)/2, 'g', 'LineWidth',2)
grid
title('Speed')
legend('w1', 'w2', 'w3');

subplot(412)
plot(time,y3,'r', 'LineWidth',2)
grid
title('Tension')
legend('tension');

subplot(413)
plot(time,u1,'b', time, u2, 'r', 'LineWidth',2)
%axis([0 Tsim -11 11])
title('Control effort')
legend('u1', 'u2');
grid

subplot(414)
plot(time,w1,'b', time, w2, 'r', 'LineWidth',2)
%axis([0 Tsim -11 11])
title('Motor Speeds')
legend('w1', 'w2');
grid