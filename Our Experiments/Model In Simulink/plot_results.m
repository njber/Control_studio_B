clear;
load('result_data.mat')

figure(1)
subplot(211)
plot(time,x1,'b',time,x2,'r')
grid
subplot(212)
plot(time,u1,'b','LineWidth',2)
axis([0 Tsim -11 11])
grid