A = [-3 1 -1; 2 -4 -2; 1 -1 -5];
B = [1;1;2];
C = [1 1 -1]; 
D = [0];
e = [B A*B A^2*B];
o = [C; C*A; C*A^2];
rangeE = orth(e);
nullO = null(o);

%% Part A
Tc = [1 -4 0; 1 -6 2; 2 -10 0];
%T = [orth(e) [1;0;0]]
Ac = inv(Tc)*A*Tc;
Bc = inv(Tc)*B;
Cc = C*Tc;

%% Part B
To = [1 1 0; 0 0 1; 0 1 1];
Ao = inv(To)*A*To;
Bo = inv(To)*B;
Co = C*To;

%% Part C
% Kalman Decomposition
T2 = orth(e)*null(null(o)-orth(e));
E1 = rangeE(:,1) - (rangeE(:,1)'*T2)/(T2'*T2)*T2;
T1 = E1/norm(E1);
E2 = nullO(:,1) - (nullO(:,1)'*T2)/(T2'*T2)*T2;
T4 = E2/norm(E2);
Tk = [T1 T2 T4];
Ak = inv(Tk)*A*Tk
Bk = inv(Tk)*B
Ck = C*Tk

% test to see if T1 and T4 are correct
% T1 U T2 is a basis for range(e)
testR = [T1 T2];
null(rangeE-testR)
