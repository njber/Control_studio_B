clear;
load('result_data.mat')
y1 = y(:,1);
y2 = y(:,2);
u1 = u(:,1);
u2 = u(:,2);

w1 = x(:,2);
w2 = x(:,4);

figure(10)
subplot(311)
plot(time,y1,'b',time,y2,'r', 'LineWidth',2)
grid
legend('speed', 'tension');
subplot(312)
plot(time,u1,'b', time, u2, 'r', 'LineWidth',2)
%axis([0 Tsim -11 11])
title('Control effort')
legend('u1', 'u2');
grid

subplot(313)
plot(time,w1,'b', time, w2, 'r', 'LineWidth',2)
%axis([0 Tsim -11 11])
title('Motor Speeds')
legend('w1', 'w2');
grid