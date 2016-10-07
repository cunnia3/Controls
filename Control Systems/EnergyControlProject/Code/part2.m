%Problem2
clc;
clear;
%% System Description
epow;
ssSys = ss(A,B,C,0);
s = tf('s');
Gwo = 10*s/(1+10*s);
Gtor = 1/(.0027*s^2 + .0762*s + 1);

%% Task 1
% Develop the state-space form of the linear power system model in MATLAB or Simulink
% using V as the output variable. (Note that you need to use the state-space form in order
% for you to do the damping controller design.) Plot the frequency response of the system
% from u to V . Find the DC gain and the gain and phase margins (with respect to unity-gain
% feedback) of the system. Identify the gain and phase crossover frequencies.
% 

u2V=tf(num1,den1);
u2W=tf(num2,den2);
figure()
bode(u2V);
gainDC = 10^(-2.54/20)
[gm,pm] = margin(u2V)

%%
% Crossover values and margins
[Gm,Pm,Wgm,Wpm]=margin(u2V);
%% Task 2
% Design a phase-lag controller K`(s) such that the steady-state voltage error with respect
% to a unit step Vref is less than 0.01 pu and the phase margin is at least 80?
% . Plot the
% compensated system (that is, the controller in series with the system) frequency response
% vs the uncompensated system frequency response to verify your design. List the gain and
% phase margins and the corresponding crossover frequencies. You will most probably find
% that the gain margin is very small. (The gain margin will be improved by the damping
% controller in the second part of the design.)

%D_C=133*(1.35*s+1)/(179.1*s+1);

% D_C=133*(6.75*s+1)/(900*s+1)
%D_C=10003*(6.75*s+1)/(900*s+1)

D_C=39*(s+0.38)/(s+0.0558);
DC_num=133*(6.75*s+1);
DC_den=(900*s+1);
figure;
bode(D_C*u2V)
[gm,pm] = margin(D_C*u2V)
%% Task 3
% Close the voltage regulator loop. Simulate and plot the response of the closed-loop system
% for a 0.1 pu Vref step input. Use the tstats function to compute the rise time, the
% overshoot, and the steady-state error. Compare the performance of the controller K`(s)
% to that of the PI controller you obtained in Design Project Part I. List the numerical
% values of the dominant K`(s) closed-loop poles in a table
% figure;
% u2Vfb=feedback(u2V,1);
% step(0.1*u2Vfb);
% title('closed-loop');
% stepinfo(u2Vfb);
figure;
u2VDfb=feedback(D_C*u2V,1);
step(0.1*u2VDfb);
title('closed-loop with comp')
stepinfo(u2VDfb)
% poles of closed-loop system with compensator
polesDC = pole(feedback(D_C*u2V,1))
%% Task 4
% Connect Q(s) in series with the washout and torsional filters to form F(s). Plot the
% frequency response of F(s), using a frequency range of 1 to 100 rad/s. Note that the EM
% mode has been left out of F(s).
u2w = tf(num2,den2);

testSys = feedback(D_C*u2V,1);
testSys = minreal(testSys, .01)
originalPoles = pole(u2V);
newPoles = pole(testSys);
zpk(testSys);

K = place(A,B,newPoles);

A1 = (A-B*K);
% State Space Model
a_23=A1(2,3:7);
a_31=A1(3:7,1);
a_32=A1(3:7,2);
a_33=A1(3:7,3:7);

[num_a,den_a]=ss2tf(a_33,a_31,a_23,0);
sysQ=tf(num_a,den_a);
sysF=sysQ*Gwo*Gtor;


bode(sysF,{1,100})

%% Task 5
% Design a phase-lead controller K?(s) such that the phase of F(s)K?(s) is between 6?
% to ?150 from 2 to 10 rad/s (approximately 0.3 to 1.6 Hz). The rational for this design is
% that if the phase of F(s)K?(s) is close to zero, the speed feedback loop will add mostly
% damping. As a further refinement, if the phase of F(s)K?(s) is slightly negative, then the
% root-locus departure direction of the EM mode with positive imaginary part is to the left
% and up, improving the so-called “synchonizing torque”. A frequency range is specified
% so that improved damping is possible as long as the frequency of the EM mode, which
% changes due to operating conditions, falls in this range. For this part, show the frequency
% response of F(s)Kd(s) in that frequency range, where Kd(s) is the damping controller
% designed in Design Project Part I. In the design for K?(s), you can again use the lead-lag
% controller frequency-response charts. Limit the number of lead sections to 2 because you
% have a limited hardware budget. Points will be deducted if you use more than 2 sections.
% After you have obtained K?(s), plot the frequency response of F(s)K?(s) to show that
% you have satisfied the design specifications

%%
% Use typical lead compensator formulation and tinker with gains until
% specification is met

Wmax = 20
theta = 56
theta = theta/180 * pi;
a = (1-sin(theta))/(1+sin(theta))
zero = Wmax*(a)^.5
pole = Wmax/(a)^.5
D_K = ((s/zero+1)/(s/pole+1))^2;
D_K = D_K*37;
bode(sysF*D_K,{1,100})


%% Task 6
% Connect G?(s) in series with K?(s) and the washout and torsional filters. Use this system
% to perform a root-locus analysis to select the gain of K?(s) to achieve a damping ratio
% ? = 15% for the EM mode. Mark on the root-locus plot the gain selection and the location
% of the closed-loop poles. You will also notice that there is a pair of complex poles going to
% the RHP. Find the gain of K?(s) to achieve a damping ratio ? = 15% for this mode. Your
% final gain of K?(s) is the smaller of these two. List the numerical values of the dominant
% closed-loop poles in the table of Task 3.

figure()
rlocus(Gwo*Gtor*D_K*u2W*-1)
% Apply gain
D_K= D_K/13

%% Task 7
[Damping_Controller_Num,Damping_Controller_Den]=tfdata(D_K,'v');
[PI_Controller_Num,PI_Controller_Den]=tfdata(D_C,'v');
[tempNum,tempDen,tfSysFinal]=simulinkToTF('problem1');

figure()
step(.1*tfSysFinal,10)




