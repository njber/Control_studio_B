clear;
load('result_data.mat')
y1 = y(:,1);
y2 = y(:,2);
u1 = u(:,1);
u2 = u(:,2);

figure(10)
subplot(211)
plot(time,y1,'b',time,y2,'r')
grid
legend('speed', 'tension');
subplot(212)
plot(time,u1,'b', time, u2, 'r', 'LineWidth',2)
%axis([0 Tsim -11 11])
title('Control effort')
legend('u1', 'u2');
grid