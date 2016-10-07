A = [0 1 0; 0 0 1; 0 -6 -20.3];
B = [0;0;100];
C = [1 0 0];
D = 0;

sys1 = ss(A,B,C,D);
e = ctrb(sys1);

% checks out!
[num,den] = ss2tf(A,B,C,D);
[A1,B1,C1,D1]=tf2ss(num,den)

%% Part C
syms k1 k2 k3
tn = [0 0 1]*inv(e);
T = inv([tn*A^2; tn*A; tn]);
Ac = inv(T)*A*T;
Bc = inv(T)*B;
Cc = C*T;
k = [139.7 8494 250000]
eig(Ac-Bc*k)

%% Part D
syms k1 k2 k3
k = [2500 84.94 1.397];
eig(A-B*k);

%% Observer
Ao = [-20.3 1 0; -6 0 1; 0 0 0];
Bo = [0;0;100];
Co = [1 0 0];
syms l1 l2 l3
L = [l1;l2;l3];
charpoly(Ao-L*Co)