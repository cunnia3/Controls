%% Reference error
G = 1/((s+1)*(5*s+1));
% pOutput sys is the transfer function of the system
% PErrorSys is the error of the system in tracking reference input
[pOutputSys, pErrorSys] = getRefSysFromPID(19,0,0,G)
[pdOutputSys, pdErrorSys] = getRefSysFromPID(19,0,4/19,G)
[pidOutputSys, pidErrorSys] = getRefSysFromPID(19,.5,4/19,G)

figure
hold on;
step(pErrorSys,400)
step(pErrorSys/s,400)
title('P Reference Error (step and ramp)')

figure
hold on;
step(pdErrorSys,400)
step(pdErrorSys/s,400)
title('PD Reference Error (step and ramp)')

figure
hold on;
step(pidErrorSys,400)
step(pidErrorSys/s,400)
title('PID Reference Error (step and ramp)')
%% Disturbance errror
G = 1/((s+1)*(5*s+1));
% pOutput sys is the transfer function of the system
% PErrorSys is the error of the system in tracking reference input
[pOutputSys, pErrorSys] = getDisturbSysFromPID(19,0,0,G)
[pdOutputSys, pdErrorSys] = getDisturbSysFromPID(19,0,4/19,G)
[pidOutputSys, pidErrorSys] = getDisturbSysFromPID(19,.5,4/19,G)

figure
hold on;
step(pErrorSys,400)
step(pErrorSys/s,400)
title('P Disturbance Error (step and ramp)')

figure
hold on;
step(pdErrorSys,400)
step(pdErrorSys/s,400)
title('PD Disturbance Error (step and ramp)')

figure
hold on;
step(pidErrorSys,400)
step(pidErrorSys/s,400)
title('PID Disturbance Error (step and ramp)')