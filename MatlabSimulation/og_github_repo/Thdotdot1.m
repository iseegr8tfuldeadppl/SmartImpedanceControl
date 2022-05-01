function eqn1 = Thdotdot1(I1,I2,T1,T2,d1,d2,g,l1,m1,m2,th1,th2,thdot1,thdot2)
%THDOTDOT1
%    EQN1 = THDOTDOT1(I1,I2,T1,T2,D1,D2,G,L1,M1,M2,TH1,TH2,THDOT1,THDOT2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    24-Apr-2022 15:13:08

t2 = sin(th1);
t3 = sin(th2);
t4 = d1.^2;
t5 = d2.^2;
t6 = d2.^3;
t7 = l1.^2;
t8 = m2.^2;
t9 = th2.*2.0;
t10 = thdot1.^2;
t11 = thdot2.^2;
eqn1 = ((I2.*T1-I2.*T2+T1.*m2.*t5-T2.*m2.*t5+(t5.*t7.*t8.*t10.*sin(t9))./2.0+I2.*d1.*g.*m1.*t2+I2.*g.*l1.*m2.*t2+(g.*l1.*t2.*t5.*t8)./2.0+l1.*t3.*t6.*t8.*t10+l1.*t3.*t6.*t8.*t11-T2.*d2.*l1.*m2.*cos(th2)-(g.*l1.*t5.*t8.*sin(t9+th1))./2.0+l1.*t3.*t6.*t8.*thdot1.*thdot2.*2.0+I2.*d2.*l1.*m2.*t3.*t10+I2.*d2.*l1.*m2.*t3.*t11+d1.*g.*m1.*m2.*t2.*t5+I2.*d2.*l1.*m2.*t3.*thdot1.*thdot2.*2.0).*2.0)./(I1.*I2.*2.0+I2.*m1.*t4.*2.0+I1.*m2.*t5.*2.0+I2.*m2.*t7.*2.0+t5.*t7.*t8+m1.*m2.*t4.*t5.*2.0-t5.*t7.*t8.*cos(t9));
