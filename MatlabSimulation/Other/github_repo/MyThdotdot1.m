function eqn1 = MyThdotdot1(Fdx,Fdy,I1,I2,T1,g,l1,l2,m1,m2,th1,th2,thdot1,thdot2)
%MYTHDOTDOT1
%    EQN1 = MYTHDOTDOT1(FDX,FDY,I1,I2,T1,G,L1,L2,M1,M2,TH1,TH2,THDOT1,THDOT2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    12-Mar-2022 00:37:34

t2 = cos(th1);
t3 = cos(th2);
t4 = sin(th1);
t5 = sin(th2);
t6 = l1.^2;
t7 = l2.^2;
t8 = l2.^3;
t9 = m2.^2;
t10 = th2.*2.0;
t11 = thdot1.^2;
t12 = -th2;
t13 = -t10;
t14 = t12+th1;
t15 = t13+th1;
t16 = sin(t14);
eqn1 = (Fdy.*I2.*-4.0+T1.*m2.*t7.*4.0+g.*t3.*t8.*t9.*2.0+Fdx.*m2.*t5.*t8.*4.0-Fdy.*m2.*t3.*t8.*4.0+g.*l1.*t7.*t9.*sin(t10)-t6.*t7.*t9.*t11.*cos(t15)+Fdy.*l1.*l2.*m2.*t16.*2.0-g.*l1.*t7.*t9.*sin(th1.*2.0)+Fdx.*l1.*m2.*t4.*t7.*4.0-Fdy.*l1.*m2.*t2.*t7.*4.0+g.*l1.*t2.*t7.*t9.*4.0+I2.*g.*l2.*m2.*cos(th1+th2).*4.0+t2.*t6.*t7.*t9.*t11-l1.*t8.*t9.*t16.*thdot2.^2.*2.0-l1.*t8.*t9.*t16.*thdot1.*thdot2.*4.0+I2.*l1.*l2.*m2.*t5.*t11.*4.0+g.*l1.*m1.*m2.*t2.*t7.*2.0)./(l2.*m2.*(I1.*l2.*4.0-I2.*l1.*t3.*4.0+l2.*m2.*t4.*t6+l1.*m2.*t7.*t16.*2.0+l2.*m2.*t6.*sin(t15)));
