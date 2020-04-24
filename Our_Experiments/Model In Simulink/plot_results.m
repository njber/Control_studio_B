clear;
load('result_data.mat')
x1 = x(:,1);
x2 = x(:,2);
u1 = u(:,1);
u2 = u(:,2);

figure(1)
subplot(211)
plot(time,x1,'b',time,x2,'r')
grid
subplot(212)
plot(time,u1,'b', time,u2, 'r', 'LineWidth',2)
%axis([0 Tsim -11 11])
grid