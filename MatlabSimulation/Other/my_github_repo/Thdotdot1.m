function EQthdotdot1 = Thdotdot1(Fdx,Fdy,L1,L2,T1,T2,g,m1,m2,th1,th2,thdot1,thdot2)
%THDOTDOT1
%    EQTHDOTDOT1 = THDOTDOT1(FDX,FDY,L1,L2,T1,T2,G,M1,M2,TH1,TH2,THDOT1,THDOT2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    19-Apr-2022 14:42:41

t2 = sin(th1);
t3 = sin(th2);
t4 = th1+th2;
t5 = L1.^2;
t6 = L2.^2;
t7 = th2.*2.0;
t8 = cos(t4);
t9 = t4+th2;
EQthdotdot1 = -(L2.*T1.*2.0-L2.*T2.*2.0-Fdx.*t6.*sin(t4).*2.0+Fdy.*t5.*cos(th1-th2)+Fdy.*t5.*t8+Fdy.*t6.*t8.*2.0-L1.*T2.*cos(th2).*2.0+Fdy.*L1.*L2.*cos(t9)+Fdy.*L1.*L2.*cos(th1).*3.0-Fdx.*L1.*L2.*t2.*2.0-L1.*L2.*g.*m1.*t2.*2.0-L1.*L2.*g.*m2.*t2+L1.*L2.*g.*m2.*sin(t9)-L1.*m2.*t3.*t6.*thdot2.^2.*2.0+L2.*m2.*t5.*thdot1.*thdot2.*sin(t7)-L1.*m2.*t3.*t6.*thdot1.*thdot2.*2.0)./(L2.*t5.*(m1.*2.0+m2-m2.*cos(t7)));