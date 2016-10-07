%% Part A
syms a q r k
% Form state-space representation of system
A = a; B = 1; C = 1; D = 0;
% Form performance measure
Q = q; R = r; N = 0;
% Solve for K
eqn= k^2 - 2*k*a*r - q*r == 0;
solk=solve(eqn,k)

% original pole is s=a for (1/(s-a))

% new pole = new A which is newA = a-k
newPole = a-solk(2)

%% Part B
A = [0 1; -4 -4]; B = [0;1];
C = eye(2); D = [0;0];
sysB = ss(A,B,C,D);
Q = [20 0; 0 5]; R = 1; N =0;
[K,S,e]=lqr(sysB,Q,R,N)

% poles before control law
polesOriginal=eig(A)

% poles after control law
polesModified=eig(A-B*K)


