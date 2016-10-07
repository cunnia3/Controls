clc
clear all
close all

% system parameters
m = 1500
b = 70
%% the larger the value of k, the faster the system converges to the desired velocity
k = 1000000000

num = [k/m]
den = [1 (b+k)/m]
u0 = 1
sys1 = tf(u0*num,den)

hold on
step(sys1)