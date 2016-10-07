function [ sysTF ] = sim2tf( simulinkName )
%sim2tf Converts simulink diagram with specified name into TF
%   Detailed explanation goes here
    sys = linmod(simulinkName)
    [numTemp,denTemp]= ss2tf(sys.a,sys.b,sys.c,sys.d)
    sysTF = tf(numTemp,denTemp);
    return
end

