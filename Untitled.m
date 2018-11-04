%% 1.Soru

clc
clear all
close all
% A Þýkký
fs = 100000;
t = 0:1/fs:1.5;
x1 = 2*sawtooth(2*pi*100*t);
subplot(1,2,1)
plot(t,x1)
axis([0 0.2 -2.2 2.2])
xlabel('Zaman (Saniye)')
ylabel('Genlik') 
title('Periyodik Testere Diþi Sinyali')
% B þýkký
fs = 1000000;
x2 = square(2*pi*20*t);
subplot(1,2,2)
plot(t,x2)
axis([0 0.2 -1.2 1.2])
xlabel('Zaman (Saniye)')
ylabel('Genlik')
title('Kare Periyodik Sinyal')
% C Þýkký
fs = 100000;
t = -1:1/fs:1;
x1 = tripuls(t,100e-3);
subplot(2,1,1)
plot(t,x1)
axis([-0.1 0.1 -0.2 1.2])
xlabel('Zaman (Saniye)')
ylabel('Genlik')
title('Periyodik Olmayan Üçgen Sinyal')
% D þýkký
fs = 10000;
x2 = rectpuls(t,50e-3);
subplot(2,1,2)
plot(t,x2)
axis([-0.1 0.1 -0.2 1.2])
xlabel('Zaman (Saniye)')
ylabel('Genlik')
title('Periyodik OLmayan Kare Sinyal')
% E Þýkký
tc = gauspuls('cutoff',50e3,0.5,[],-60); 
t1 = -tc : 1e-6 : tc; 
y1 = gauspuls(t1,50e3,0.6);
plot(t1*1e3,y1)
xlabel('Zaman (milisaniye)')
ylabel('Genlik')
title('Gaussian Pulse')
% F Þýkký
fs = 200E9;                   
D = [2.5 10 17.5]' * 1e-9;   
t = 0 : 1/fs : 2500/fs;       
w = 2e-9;                     
yp = pulstran(t,D,@rectpuls,w);
subplot(2,1,1)
plot(t*1e9,yp);
axis([0 25 -0.2 1.2])
xlabel('Zaman (nanosaniye)')
ylabel('Genlik')
title('Rectangular Train')
% G Þýkký
T = 0 : 1/50e3 : 10e-3;
D = [0 : 1/1e3 : 10e-3 ; 0.6.^(0:10)]';
Y = pulstran(T,D,@gauspuls,10E3,.5);
plot(T*1e3,Y)
xlabel('Zaman (milisaniye)')
ylabel('Genlik')
title('Gaussian Pulse Train')
%% 2.Soru
clear all;
close all;
clc;
%Continious Unit Ýmpulse
N=50;
n=-N:1:N;
y=[zeros(1,N),ones(1,1),zeros(1,N)];
subplot(2,1,1);
plot(n,y);
ylabel('Genlik');
xlabel('Zaman--->');
title('Unit Ýmpulse Discrete Signal');
display(y);
%Discrete Unit Ýmpulse
subplot(2,1,2);
stem(n,y);
ylabel('Genlik');
xlabel('Number of Samples--->');
title('Unit Ýmpulse Discrete Signal');
display(y);
%Continious Unit Ramp
t = (-40:1:40)';
ramp = t.*unitstep;
subplot(6,1,5);
plot(t,ramp,'r','linewidth',3) 
ylabel('Genlik');
xlabel('Zaman--->');
title('Unit Ramp Discrete Signal');
display(ramp);
%Discrete Unit Ramp
subplot(6,1,6);
stem(t,ramp,'r','linewidth',3) 
ylabel('Genlik');
xlabel('Zaman--->');
title('Unit Ramp Discrete Signal');
display(ramp);  
%Continious Unit Step
t = (-40:1:40)';
unitstep = zeros(size(t)); 
unitstep(t>=1) = 1; 
subplot(6,1,3);
plot(t,unitstep,'g','linewidth',3) 
ylabel('Genlik');
xlabel('Zaman--->');
title('Unit Step Continious Signal');
display(unitstep);
%Discrete Unit Step
subplot(6,1,4);
stem(t,unitstep,'g','linewidth',3) 
ylabel('Genlik');
xlabel('Zaman--->');
title('Unit Step Discrete Signal');
display(unitstep);

%% 3.Soru
%1 kHz olan ve 20 Hz’lik surekli zamanlý periyodik kosinus ve sinus
%sinyallerinin grafiði.
fs=1000 %fs=100 hz
f=20 %20 Hz
t=0:1/fs:1
y1=sin(2*pi*f*t)
subplot(3,1,1)
plot(t,y1,'LineWidth',3,'Color','blue')
title('x[n] Sinyalinin Discrete Sinus Grafiði')
grid on
y2=cos(2*pi*f*t)
subplot(3,1,2)
plot(t,y2,'LineWidth',3)
title('x[n] Sinyalinin Discrete Cosunus Grafiði')
grid on
%3.Soru x1[n] ayrýk zamanlý grafiði.
n=-30:1:30
a=cos(2*pi*n/36)
b=sin(2*pi*n/36)
subplot(3,1,3)
stem(a,b)
title('x[n] Sinyalinin Continious Grafiði')
%% 4.Soru
x=t.*(t.^2+3)
t=-0:10:100;
g_even=(g(x)+g(-x))/2
g_odd=(g(x)-g(-x))/2
plot(x,g_odd)
xlabel('t.*(t.^2+3)')
ylabel('(g(x)-g(-x))/2')
title('g(t) Sinyalinin Tek-Çift Grafiði')
%% 5.Soru
clc
clear all
close all
n=-50:50
y=(0.9.^abs(n)).*sin(2*pi*n/4)
sum(abs(y.^2))
title('x[n] Ayrýk Zamanlý Sinyalin Enerjisi')
%% 6.Soru
% Altý grafikten dördünü çýkarabildim ikisi yanlýþ çýktý. Araþtýrmama raðmen yapamadým.  
%% 7.Soru
n = 0:100;
x =4+cos(2*pi*n/24);
x0 = downsample(x,2,0);
x1 = downsample(x,2,1);
figure
stem(x)
title('Original Signal')
xlabel('4+cos(2*pi*n/24)')
ylabel('Cosunus Signal')
%n=2 için
n = 0:100;
x = 4+cos(2*pi*n/24);
x0 = downsample(x,2,0);
subplot(3,1,1)
stem(x)
title('Original Signal')

y0 = upsample(x0,2,0);
subplot(3,1,2)
stem(y0)
title('N=2 Signal')
xlabel('4+cos(2*pi*n/24)')
ylabel('Cosunus Signal')
%n=10 için
n = 0:100;
x = 4+cos(2*pi*n/24);
x0 = downsample(x,10,0);
subplot(3,1,1)
stem(x)
ylim([0.5 3.5])
title('Original Signal')

y0 = upsample(x0,10,0);
subplot(3,1,2)
stem(y0)
ylabel('N=10 Signal')
xlabel('4+cos(2*pi*n/24)')
ylabel('Cosunus Signal')
