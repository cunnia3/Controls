function [ T,E ] = getRefSysFromPID( P,I,D,G )
%GETSYSFROMPID generate transfer function for PID system
% P I D are constants for gains
% G is plant
% T is transform of system 
% E is error of system (no sensor or disturbance)
s = tf('s');
D = P + D*s + I/s;
T = G*D/(1+G*D);
E = 1/(1+G*D);
return
end

