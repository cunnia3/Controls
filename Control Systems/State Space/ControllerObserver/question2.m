syms tau
A = [-1 1 -1; 1 -1 -2; 1 0 -5];
B = [1;1;2];
C = [1 1 -1];
D = 0;
sys1 = ss(A,B,C,D);
e = ctrb(sys1);
o = obsv(sys1);
Wc = gram(sys1,'c');
Wo = gram(sys1,'o');

[V,D] = eig(A);
myMatrix = V*(D*exp(tau))*inv(V)