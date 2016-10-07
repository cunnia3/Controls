mdl_puma560;
lambdaMax = 1;
muMax = 1;

% define parametric paths
syms l mu u li lf
% f1 = -2*l^3+3*l^2; f2 = -2*l^3+3*l^2;
% f3 = -2*l^3+3*l^2; f4 = -2*l^3+3*l^2;
% f5 = -2*l^3+3*l^2; f6 = -2*l^3+3*l^2;
f1(l) = l; f2(l) = l;
f3(l) = l; f4(l) = l;
f5(l) = l; f6(l) = l;
if1 = finverse(f1); if2 = finverse(f2);
if3 = finverse(f3); if4 = finverse(f4);
if5 = finverse(f5); if6 = finverse(f6);

f(l) = [f1;f2;f3;f4;f5;f6];
dfdl(l) = diff(f,l);
inversef(l) = [if1;if2;if3;if4;if5;if6];

% define cost function
L = 1/mu;
costFunction(li,lf) = int(L,l,li,lf);

% divide mu,lambda phase space
% nMu is # of mu divisions
% nL is # of lambda divisions
nMu = 5; nL = 5;

% initialize cost matrices to infinity
% set last cost matrix entry to 0
costMatrix = ones(nMu,nL)*100000;
costMatrix(1,nL) = 0;

% initialize pointer matrices
pointerMatrix = ones(nMu,nL)*-1;

% do dynamic programming!
for columnCounter=nL:-1:2
    columnCounter
    % Find best way to get from columnCounter-1 to columnCounter
    for muki=1:nMu %muki is index for mu_i
        for mukp1i=1:nMu %mukp1i is index for mu_i+1
            muk = indexToMu(muki,nMu,muMax);
            mukp1 = indexToMu(mukp1i,nMu,muMax);
            lkp1 = indexToLambda(columnCounter,nL,lambdaMax);
            lk = indexToLambda(columnCounter-1,nL,lambdaMax);
            
            % Get robot dynamics parameters
            q = f(lk); q = double(q)';
            M = p560.inertia(q); G = p560.gravload(q);
            C = zeros(size(M));
            Mi=M*dfdl; Qi = C*dfdl; 
            
            tempMu = sqrt(((lkp1-l)*(muk-1)^2+(l+lk)*(mukp1-1)^2)/(lkp1-lk));
            tempu = Qi*mu^2+G'+Mi*((mukp1-1)^2-(muk-1)^2)/(2*(lkp1-lk));
            
            incrementalCost = int(tempMu,l,lk,lkp1);
            
            newCost = double(incrementalCost + costMatrix(mukp1i,columnCounter));
            
            if newCost < costMatrix(muki,columnCounter-1)
                costMatrix(muki,columnCounter-1) = newCost;
                pointerMatrix(muki,columnCounter-1) = mukp1i;
            end  
            
        end
    end
end

indexTraj = getControlTraj(pointerMatrix)
muTraj = [0]
for i=1:size(indexTraj,2)-1
    muTraj = [muTraj indexToMu(indexTraj(i),nMu,muMax)];
end