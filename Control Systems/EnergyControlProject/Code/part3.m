% Author: Andrew Cunningham
% Email: andrew64ce@gmail.com
% CONTROL SYSTEMS ENGINEERING PROJECT QUESTION 3

%% SYSTEM DESCRIPTION
s = tf('s');
epow
Gwo = 10*s/(1+10*s);
Gtor = 1/(.0027*s^2 + .0762*s + 1);
Damping_Controller_Num = [0];
Damping_Controller_Den = [1];
PI_Controller_Den = [1];
PI_Controller_Num = [1];

%% TASK 1
% Develop the state-space form of the linearized power system model in MATLAB or
% Simulink using V as the output variable. List the poles and the zeros of the singleinput,
% single-output system. Find the time constants of the poles (the time constant is
% equal to the inverse of the negative of the real part of the pole).

% Get the system in control canonical form
tfSys = tf(num1,den1);
[A,B,C,D]=tf2ss(num1,den1) 

poles = pole(tfSys);
zeros1 = zero(tfSys);
timeConstants = (-poles).^-1;

%%
% 
% <<p3t1.jpg>>
% 


%% TASK 2
% With the damping control loop open, design an observer-based controller K1(s) and the
% scalar gain N such there for a unit step input Vref, the rise time is less than 0.5 s, the
% overshoot is less than 5%, and the steady-state voltage error is zero

%%
% 
% <<p3t2a.jpg>>
% 


%%
% *SUBTASK A*
% Design a full-state feedback control to place the poles of the voltage closed-loop
% system at the desired locations to reduce the dominant time constant(s). There
% is no need to move the poles that do not significantly influence the rise time. In
% particular, do not move the EM modes.

%%
% Controller is of form $\dot{x}=Ax-BKx$ which simplifies to
% $\dot{x}=(A-BK)x$ for full state feedback.  Place the poles of the system
% to acheive the desired specifications
p = poles(1:6)';
p = [p -4];
K = place(A,B,p);
controllerPoles=eig(A-B*K)

%%
% *SUBTASK B*
% Design an observer so that the states of the system can be estimated using the voltage
% measurement. You have to assign the observer poles such that their transients decay
% faster than the full-state feedback poles. Again, there is no need to move the observer
% poles that do not significantly influence the estimation transients.


%%
% Observer is of form $\dot{\hat{x}}=A\hat{x}x + Bu + L(y-\hat{c}x)$
% Find L to place observer, place real part of dominant mode further out
p = poles(1:6)';
p = [p -7];

% Duality is our friend!
L = place(A',C',p)';
eig(A-L*C)

%%
% *SUBTASK C*
% Construct the observer-based controller K1(s) using the full-state feedback gain and
% the observer gain. Find the scaling N such that the controller will achieve zero
% steady-state error for a unit-step input
K1 = ss(A-B*K-L*C, -L, -K, 0)
[numK1,denK1] = ss2tf(K1.a,K1.b,K1.c,K1.d);
DK1 = tf(numK1,denK1)

%%
% *SUBTASK D*
% Apply the controller K1(s) to the voltage loop. Simulate the closed-loop system
% response to a 0.1 step in Vref. Use tstats to verify that the controller meets all the
% specifications. Comment on the step response.

K1sysFB=feedback(1.037*tfSys*DK1,1);
stepinfo(K1sysFB)
figure()
step(K1sysFB)

%%
% The step response meets the rise time and overshoot requirements with a
% high margin.

%% 
% *SUBTASK E*
% Find the gain and phase margins achieved by K1(s).
[GM,PM] = margin(K1sysFB)

%% TASK 2
% The controller K1(s) is 7th order, with many of its dynamics not important. We will now
% proceed to reduce its order

%%
% *SUBTASK A*
% Plot the frequency response of the 7th-order K1(s). What kind of controller is K1(s)?
% Does your K1(s) exhibit a notch in the magnitude at about the EM mode frequency?
% Do you know what the purpose of this notch is? This notch is at a fixed frequency,
% implying it is less useful when the frequency of the EM mode changes

%%
% Notch-like low pass filter, its there to reduce its effect on the EM
% poles
figure()
bode(DK1)
title('Frequency Response of Unreduced K1')
%%
% *SUBTASK B*
% Express K1(s) in ZPK form, showing all the factors of poles and zeros. You will
% notice that most of the poles have nearby zeros, except for one pole pd. Thus we
% can perform approximate pole-zero cancellations and retain only pd in the first-order
% reduced controller K1(s), which will have the form
% $K_1(s)=\frac{K_p}{s-p_d}$ where Kp is selected so that the first-order K1(s) and the 7th-order K1(s) have the
% same DC gain.



%%
% The poles and zeros approximately cancel in most places
zpk(DK1)
DK1_Reduced = 415/(s+12.02)

%%
% *SUBTASK C*
% Plot the frequency response of the first-order K1(s) versus the frequency response of
% the 7th-order K1(s). Comment on their similarities and differences.
figure()
bode(DK1_Reduced)
title('DK1 Reduced frequency response')
figure()
bode(DK1)
title('DK1 Unreduced frequency response')

%%
% The reduced order controller loses the notch at the EM frequency.  They
% are, however, similar in their DC gain, and approximately the same in
% their phase contributions (with the notable exception of the notch in the
% full order controller)

%%
% *SUBTASK D*
% Close the voltage loop with this first-order controller and perform a step response
% with a 0.1 pu Vref step input. Comment on the time response.
K1_ReducedsysFB=feedback(1.037*tfSys*DK1_Reduced,1);
step(K1_ReducedsysFB*.1)
stepinfo(K1_ReducedsysFB)

%% 
% The response is pretty good in terms of rise time and overshoot relative
% to previous parts.  However, there is a lot of oscillation in the output

%% TASK 4
% Use Simulink or MATLAB to generate the state-space model G?(s) from Vref to ?f with
% the voltage controller K1(s) loop closed. The damping controller K2(s) is then designed
% to improve the damping ratio of the EM mode to 0.17. The recommended steps to obtain
% K2(s) are contained in the following list

%%
% 
% <<p3t4a.jpg>>
% 

%%
% We then use simulink to obtain the transfer function from Vref to Wf with
% the damping control disconnected.

%%
% 
% <<p3t4a2.jpg>>
% 

%% 
% *SUBTASK A*
% Design a full-state feedback control to place the EM poles at their desired locations.
% There is no need to move the other poles.

%%
% Get the system in control canonical form
[PI_Controller_Num,PI_Controller_Den] = tfdata(DK1_Reduced,'v');
[tempNum,tempDen,tfSys2]=simulinkToTF('problem3');


[num3,den3] = tfdata(tfSys2, 'v');
[A2,B2,C2,D2]=tf2ss(num3,den3); 

poles = pole(tfSys2)

p = poles(1:6)';
p = [p -1.6+9.3345i -1.6-9.3345i poles(9:11)'];
K2 = place(A2,B2,p);
controllerPoles=eig(A2-B2*K2)

%%
% *SUBTASK B*
% Design an observer so that the states of the system can be estimated using the
% torsional filter output. You have to assign the observer poles such that the EM
% mode is estimated well. Again, there is no need to move the observer poles that do
% not significantly influence the observer transients

%%
% Find L to place observer, place real part of dominant mode further out

% Duality is our friend!
p = poles(1:6)';
p = [p -4.6+9.3345i -4.6-9.3345i poles(9:11)'];
L2 = place(A2',C2',p)';
eig(A2-L2*C2)

%%
% *SUBTASK C*
% Construct the observer-based controller K2(s) using the full-state feedback gain and
% the observer gain. Plot and comment on its frequency response

%%
% Obesrver based-controller
K2 = ss(A2-B2*K2-L2*C2, -L2, -K2, 0);
[numK2,denK2] = ss2tf(K2.a,K2.b,K2.c,K2.d);
DK2 = tf(numK2,denK2);
bode(DK2)

%%
% *SUBTASK D*
% Do a root-locus analysis of the damping controller loop using K2(s). Comment on
% your observations
figure()
rlocus(tfSys2*DK2)
axis([-30 10 -25 25])

%%
%  The root locus now exhibts a departure angle of about 180 degrees which
%  is similar to the compensated root locus of part 1.  We now have damping
%  on the EM poles.


%%
% *SUBTASK E*
% Apply the controller K2(s) to the damping control loop. Simulate the closed-loop
% system response to a 0.1 step in Vref. Use tstats to assess the performance of the
% overall system, that is, the combined K1(s) and K2(s) controller. Comment on the
% step response
[Damping_Controller_Num,Damping_Controller_Den]=tfdata(DK2,'v');
[PI_Controller_Num,PI_Controller_Den]=tfdata(DK1_Reduced,'v');
[tempNum,tempDen,tfSysFinal_unReduced]=simulinkToTF('problem1');
stepinfo(tfSysFinal_unReduced*.1)
figure()
step(.1*tfSysFinal_unReduced)

%%
% This is an awesome step response!  It performs better than the control
% system in part 1 in both rise time and overshoot!

%% TASK 5
% The controller K2(s) is 11th order (why?), again with many of its dynamics not important.
% We will now proceed to reduce its order.


%%
% K2 is of order 11 because the forward path of the damping loop is of
% order 11.  

%% 
% *SUBTASK A*
%  Express K2(s) in ZPK form, showing all the factors of poles and zeros. You will
% notice that a few poles have nearby zeros. Instead of manually doing the polezero
% cancellation to reduce the order of K2(s), this time we will use the MATLAB
% 12
% minreal function (for minimal realization) to reduce the order of the controller. Use
% a tolerance of about 0.001 for minreal to obtain a controller of about 5th order.
% (Note that if the tolerance is set to zero, only uncontrollable and unobservable poles
% will be eliminated.) Plot the frequency response of this low-order controller and
% compared it to that of the 11th-order controller. (We could reduce further the order
% of K2(s) to perhaps a 2nd- or 3rd-order controller. This reduction will require more
% analysis and will not be attempted here.)

zpk(DK2)

%%
% The controller used requires a more rough cancellation of poles/zeros
% to achieve a 5th order controller
DK2_Reduced = minreal(DK2, .1)


%%
% *SUBTASK B*
%  Close the damping control loop with this low-order controller and perform a step
% response of the overall closed-loop system with a 0.1 pu Vref step input. Comment
% on the time response.
[Damping_Controller_Num,Damping_Controller_Den]=tfdata(DK2_Reduced,'v');
[PI_Controller_Num,PI_Controller_Den]=tfdata(DK1_Reduced,'v');
[tempNum,tempDen,tfSysFinal_Reduced]=simulinkToTF('problem1');
stepinfo(tfSysFinal_Reduced*.1)
figure()
step(.1*tfSysFinal_Reduced)
title('K1 & K2 reduced step response')


%% Comparing results
figure()
step(.1*tfSysFinal_Reduced)
title('K1 & K2 reduced step response')

figure()
step(.1*tfSysFinal_unReduced)
title('K1 reduced K2 unreduced step response')

[Damping_Controller_Num,Damping_Controller_Den]=tfdata(DK2,'v');
[PI_Controller_Num,PI_Controller_Den]=tfdata(DK1,'v');
[tempNum,tempDen,tfSysFinal_NoneReduced]=simulinkToTF('problem1');
figure()
step(.1*tfSysFinal_NoneReduced)
title('K1 & K2 unreduced step response')

figure()
step(.1*tfSysFinal_NoneReduced,5)
title('State Space Design Step Response')

%%
% The full unreduced system performed the best 