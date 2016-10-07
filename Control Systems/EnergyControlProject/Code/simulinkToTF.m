function [ num,den,sysTF ] = simulinkToTF( simulinkName )
%Convert simulink model to tf model
sys = linmod(simulinkName);
[num,den]= ss2tf(sys.a,sys.b,sys.c,sys.d);
sysTF = tf(num,den);


end

