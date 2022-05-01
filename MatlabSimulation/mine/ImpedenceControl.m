function Ta = ImpedenceControl(Kd,Kp,l1,l2,th1,th2,thdot1,thdot2,xdott,xt,ydott,yt)
%IMPEDENCECONTROL
%    TA = IMPEDENCECONTROL(KD,KP,L1,L2,TH1,TH2,THDOT1,THDOT2,XDOTT,XT,YDOTT,YT)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    23-Apr-2022 14:18:54

t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t9 = -yt;
t5 = l1.*t2;
t6 = cos(t4);
t7 = l1.*t3;
t8 = sin(t4);
t10 = l2.*t6;
t11 = l2.*t8;
t12 = t10.*thdot2;
t13 = t11.*thdot2;
t14 = t5+t10;
t15 = t7+t11;
t16 = t15+xt;
t17 = t14.*thdot1;
t18 = t15.*thdot1;
t20 = t9+t14;
t19 = Kp.*t16;
t21 = Kp.*t20;
t23 = t12+t17+xdott;
t24 = t13+t18+ydott;
t22 = -t21;
t25 = Kd.*t23;
t26 = Kd.*t24;
t27 = t19+t25;
t28 = t22+t26;
Ta = [-t14.*t27+t15.*(t21-t26);-t10.*t27+t11.*(t21-t26)];
