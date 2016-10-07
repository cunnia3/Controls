% Author: Andrew Cunningham
% Email: andrew64ce@gmail.com
% CONTROL SYSTEMS ENGINEERING PROJECT QUESTION 1
clc;
clear;
Damping_Controller_Num = [1];
Damping_Controller_Den = [1]; 
PI_Controller_Den = [1];
PI_Controller_Num = [1];
% Import system model
epow;
s = tf('s');
Gwo = 10*s/(1+10*s);
Gtor = 1/(.0027*s^2 + .0762*s + 1);
vr2w = tf(num2,den2);

%% TASK 1
% Develop the linear power system model in MATLAB using V as the output variable. Plot
% the poles and zeros of the open-loop system and list them in a table.

% Form sys1 to be the transfer function from u to V
sysOpenLoop = tf(num1,den1)
pzmap(sysOpenLoop)

% Open Loop System Poles and Zeros
pOpen = pole(sysOpenLoop)
zOpen = zero(sysOpenLoop)

%% TASK 2
% Simulate and plot the response of the open-loop system for a 0.1 pu step input
step(sysOpenLoop*.1)

%% TASK 3
% Perform a root-locus analysis of the system using a P (proportional gain) controller K(s) =
% Kp. Plot the root locus. You may need several plots to show all the details. Find
% approximately the gain Ku when the lightly damping EM mode becomes unstable. The
% MATLAB data marker feature is useful for finding Ku. List all the poles of the closed-loop
% system for K(s) = Ku in the table of Task 1.

figure()
rlocus(sysOpenLoop);

% Using the data curser reveals that:
ku1 = 50.9;
ku2 = 990;

polesku1 = pole(feedback(ku1*sysOpenLoop,1))
polesku2 = pole(feedback(ku2*sysOpenLoop,1))

%% TASK 4
% For each of the following values
% Kp = 10, 20, 30, 40, 50
% simulate and plot the response of the closed-loop system and the control signal u for a
% 0.1 pu step change in Vref . Put all the output responses on the same plot, and all the
% control signals on a separate plot. Be sure to label all the curves. (You can use the text
% command to put text on a plot.)
sys1 = feedback(10*sysOpenLoop,1);
sys2 = feedback(20*sysOpenLoop,1);
sys3 = feedback(30*sysOpenLoop,1);
sys4 = feedback(40*sysOpenLoop,1);
sys5 = feedback(50*sysOpenLoop,1);

figure()
step(.1*sys1,.1*sys2,.1*sys3,.1*sys4,.1*sys5,20)
title('Step Responses of Closed Loop Systems with Specified Gains')
legend('Kp = 10','Kp = 20','Kp = 30','Kp = 40','Kp = 50')

%% TASK 5
% Design a PI controller for the voltage control loop such that for a 0.1 pu Vref step input, the rise time of the
% voltage (V ) response satisfies tr ? 0.45 s and the over-shoot satisfies Mp ? 8%. In
% your design, select several sets of Kp and KI and perform step response simulations.
% Reasonable ranges of these parameters are 0 < Kp < Ku and 0.1 < KI < 10. Then from
% the plots select the most appropriate Kp and KI . Verify your design by simulating a 0.1
% pu Vref step input.

%%
% *Approach*
% Use the fact that $K(s) = Kp(1+KI/s) = Kp((KI+s/s)$ which is similar in form 
% to a lag compensator

%rltool(sysOpenLoop)
s = tf('s');

% ORIGINALS, FOUND USING RLTOOL
% D = 7.5*(5*s +1)/s;
% D = 11*(3.3*s+1)/s;
% D = 78*(s*.5+1)/s;

% RANDOM SEARCHER
% Overshoot = 10;
% RiseTime = 1;
% while Overshoot > 8 || RiseTime > .45 || isnan(Overshoot) || isnan(RiseTime)
%     KI = rand*.5;
%     KP = rand * 50;
%     D = KP*KI*((s/KI + 1))/s;
%     testSys = feedback(D*sysOpenLoop,1);
%     test = stepinfo(testSys);
%     Overshoot = test.Overshoot 
%     RiseTime = test.RiseTime
% end;

% BEST VALUES
%KP=40.0866;
% KI=0.0959;

KP = 40.0866;
KI = .0959;


D = KP*KI*((s/KI + 1))/s;
testSys = feedback(D*sysOpenLoop,1);
test = stepinfo(testSys)
pole(testSys)
PI_Controller = D;
[PI_Controller_Num,PI_Controller_Den]=tfdata(D,'v');


%% TASK 6
% Add the PI controller K(s) and the washout filter to your model. Leave the damping
% controller Kd(s) out for now. Generate the model (or transfer function) from Vref to ?f .
% Plot the zeros and poles of the transfer function from Vref to ?f with K(s) in the loop.

% Reference voltage to w
%vr2w;
[tempNum,tempDen,sysTask6]=simulinkToTF('problem1DampingControlLoop');
% Transfer function with washout filter and K
sysTask6

pzmap(sysTask6)

%% TASK 7
% Angle of departure is about 60 degrees
figure()
rlocus(sysTask6)
axis([-30 10 -5 25])

%% TASK 8

% Finding Damping Controller, we need to compensate 110 degrees to get to 
% 170 degree departure angle
% rltool(sysTask6)

DampingControllerSys = .1*[14*(s+2.94)/(s+34.9)]^2;
[Damping_Controller_Num,Damping_Controller_Den] = tfdata(DampingControllerSys,'v');
[tempNum,tempDen,finalSys]=simulinkToTF('problem1');


%% TASK 9
stepinfo(finalSys)
figure()
step(finalSys*.1,5)
title('Root Locus Design Step Response')