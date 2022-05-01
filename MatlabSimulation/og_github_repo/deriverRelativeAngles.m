%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Derive double pendulum dynamics %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Derive double pendulum equations of motion, write to Thdotdot1 and
%Thdotdot2 matlab function files. THIS VERSION DERIVES IT WITH THE ELBOW
%ANGLE RELATIVE TO THE UPPER ARM ANGLE.

%Parameters symbolically.
syms m1 m2 I1 I2 g l1 l2 d1 d2 th1 th2 thdot1 thdot2 thdotdot1 thdotdot2 er1 er2 eth1 eth2 T1 T2 real

%Unit vectors, cartesian
i = [1 0 0]';
j = [0 1 0]';
k = [0 0 1]';

%Rotating reference frames
er1 = [-sin(th1), cos(th1), 0]'; %Just changed it so 0 is upright.
er2 = [-sin(th2+th1), cos(th2+th1), 0]';

eth1 = [-cos(th1),-sin(th1),0]';
eth2 = [-cos(th2+th1),-sin(th2+th1),0]';

%Vectors to significant points
ra_c1 = d1*er1; %A is fixed point, B is the elbow, c1 and c2 are COMs, e end effector
rb_c2 = d2*er2;
rb_e = l2*er2;
ra_b = l1*er1;
ra_c2 = ra_b + rb_c2;
ra_e = ra_b + rb_e;

matlabFunction(ra_e, 'file', 'ForwardKin');

%Velocities
VB = l1*thdot1*eth1;
Vc2 = VB + d2*(thdot2+thdot1)*eth2;
Ve = VB + l2*(thdot2+thdot1)*eth2;





syms Fdx Fdy real

Fd = [Fdx, Fdy];

P1 = [ - l2*cos(th1 + th2) - l1*cos(th1) ;
       - l2*sin(th1 + th2) - l1*sin(th1) ];
   
P2 = [ - l2*cos(th1 + th2) ;
       - l2*sin(th1 + th2) ];
 
M = [ I2 + I1 + 2*m2*d2*l1*cos(th2) + m2*l1^2 + m2*d2^2 + m1*d1^2  , I2 + m2*d2^2  + m2*d2*l1*cos(th2) ;
      I2 + d2*m2*l1*cos(th2) + m2*d2^2                             , I2 + m2*d2^2                      ];
          
B = [ - d2*m2*l1*thdot2^2*sin(th2) - 2*m2*d2*l1*thdot1*thdot2*sin(th2) ;
        d2*m2*l1*sin(th2)*thdot1^2                                     ];
              
G = [ - g*m2*d2*sin(th1 + th2) - (m2*l1 + d1*m1)*g*sin(th1) ;
      - d2*g*m2*sin(th1 + th2)                              ];
              
F = [ Fd*P1 ;
      Fd*P2 ];

T = [ T1 ;
      T2 ];
  
  
Zero = M * [thdotdot1;thdotdot2] + B + G - [T1;T2]% - F;

%Zero =  M^-1 * ( [thdotdot1;thdotdot2] + B + G - [T1;T2] - F );

eqn1forthdotdot1 = solve(Zero(1), thdotdot1);
eqn2forthdotdot2 = solve(Zero(2), thdotdot2);

eqn1 = simplify(solve(subs(eqn1forthdotdot1, thdotdot2, eqn2forthdotdot2)-thdotdot1,thdotdot1));
eqn2 = simplify(solve(subs(eqn2forthdotdot2, thdotdot1, eqn1forthdotdot1)-thdotdot2,thdotdot2));









%Create matlab functions for thdotdot1 and 2:
matlabFunction(eqn1, 'file', 'Thdotdot1');
matlabFunction(eqn2, 'file', 'Thdotdot2');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Gravity Compensation %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T2Eq = simplify(solve((solve(eqn1,T1)-solve(eqn2,T1)),T2));

T1Eq = simplify(subs(solve(eqn1,T1),T2,T2Eq));

matlabFunction(T1Eq, 'file', 'GravityCompT1');
matlabFunction(T2Eq, 'file', 'GravityCompT2');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Impedance control?   %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Jacobian relating end effector velocity to joint space vel
% ie Ve = J*qv

J = jacobian(Ve,[thdot1 thdot2]);

syms Kp Kd xt yt xdott ydott real

zt = [xt yt 0 ]'; %Trajectory tracked
ztdot = [xdott ydott 0]'; %velocity tracked

Ta = J'*(Kp*(zt - ra_e) + Kd*(ztdot - J*[thdot1 thdot2]'));

matlabFunction(Ta, 'file', 'ImpedenceControl');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Energy eqns %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Turned this off after making sure stuff worked:

% syms k1 k2 real
% 
% PE = g*dot(ra_c2,j)*m2 + g*dot(ra_c1,j)*m1;
% KE = 1/2*I1*thdot1^2 + 1/2*I2*(thdot2+thdot1)^2 + 1/2*m1*dot(Vc1,Vc1) + 1/2*m2*dot(Vc2,Vc2);
% 
% Etot = PE + KE;
% 
% matlabFunction(Etot, 'file', 'TotEnergy');

