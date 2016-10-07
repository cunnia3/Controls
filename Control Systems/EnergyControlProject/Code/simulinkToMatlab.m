sys = linmod('problem1')
[numTemp,denTemp]= ss2tf(sys.a,sys.b,sys.c,sys.d)
sysTF = tf(numTemp,denTemp);
stepinfo(sysTF)
pzplot(sysTF)