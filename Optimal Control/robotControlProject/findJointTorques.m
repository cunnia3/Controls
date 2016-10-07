function [ u ] = findJointTorques(t, qd, q, qstar)
    nL = 5;
    muTraj = [0 .8 .8 .8 0];
    lambdaMax = 1;

    % Offset from 0 starting location
    q=q+.01

    mdl_puma560;

    % PARAMETERIZE PATHS
    syms l mu u li lf
    f1(l) = l; f2(l) = l;
    f3(l) = l; f4(l) = l;
    f5(l) = l; f6(l) = l;
    if1(l) = finverse(f1); 
    f(l) = [f1;f2;f3;f4;f5;f6];
    dfdl(l) = diff(f,l);
    lCurrent = double(if1(q(1)));

    % GET BOUNDARY CONDITIONS
    mu0 = muTraj(floor(lCurrent/lambdaMax * nL)+1);
    mu1 = muTraj(ceil(lCurrent/lambdaMax * nL)+1);
    lk = floor(lCurrent/lambdaMax * nL)*lambdaMax+1;
    lkp1 = ceil(lCurrent/lambdaMax * nL)*lambdaMax+1;
    mu(l) = sqrt(((lkp1-l)*(mu0-1)^2+(l+lk)*(mu1-1)^2)/(lkp1-lk));

    % CALCULATE ROBOT DYNAMICS
    M = p560.inertia(q); G = p560.gravload(q);
    C = zeros(size(M));
    Mi=M*dfdl; Qi = C*dfdl; 

    % CALCULATE JOINT TORQUES
    u = Qi*mu(lCurrent)^2+G'+Mi*((mu1-1)^2-(mu0-1)^2)/(2*(lkp1-lk));
    u = double(u)'
end
