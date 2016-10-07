A = [-2 1; 1 0];
B = [1; 0];
C = [1 2];
D = 0;
[num,den] = ss2tf(A,B,C,D)

K = [1 3];

det([C;C*(A-B*K)]);

A1 = A-B*K;
[num1,den1] = ss2tf(A1,B,C,D)
