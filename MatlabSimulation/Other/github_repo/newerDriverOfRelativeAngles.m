%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Derive double pendulum dynamics %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Derive double pendulum equations of motion, write to Thdotdot1 and
%Thdotdot2 matlab function files. THIS VERSION DERIVES IT WITH THE ELBOW
%ANGLE RELATIVE TO THE UPPER ARM ANGLE.

%Parameters symbolically.
syms m1 m2 g l1 l2 th1 th2 thdot1 thdot2 thdotdot1 thdotdot2 real % this is reating the varaibles seen here, they are variables to be used in formulas

B = [ -m2*l1*l2*(2*thdot1*thdot2 + thdot2^2) * sin(th2) ;
        m2*l1*l2*thdot1^2*sin(th2) ];
    
G = [ (m1 + m2)*g*l1*cos(th1) + m2*g*l2*cos(th1 + th2) ;
        m2*g*l2*cos(th1 + th2) ];

h = m2*l2^2 + (m1 + m2)*l1^2 + 2*m2*l1*l2*cos(th2);
i = m2*l2^2 + m2*l1*l2*cos(th2);
j = i;
k = m2*l2^2;

M = [ h i ;
       j k];

%Force at end effector
syms Fdx Fdy real    
Fd = [Fdx; Fdy];

leftSide = M * [thdotdot1; thdotdot2] + B + G;
eqn2forthdotdot1 = solve(leftSide(1) - Fd(1),thdotdot1);
eqn2forthdotdot2 = solve(leftSide(2) - Fd(2),thdotdot2);

eqn2 = simplify(solve(subs(eqn2forthdotdot2, thdotdot1, eqn1forthdotdot1)-thdotdot2,thdotdot2)); % subs will take where ever there is thdotdot1 in eqn2forthdotdot2 and replaces it with its actual formula eqn1forthdotdot1
eqn1 = simplify(solve(subs(eqn1forthdotdot1, thdotdot2, eqn2forthdotdot2)-thdotdot1,thdotdot1));

matlabFunction(eqn1, 'file', 'MyThdotdot1');
matlabFunction(eqn2, 'file', 'MyThdotdot2');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Impedance control?   %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Velocities
eth1 = [-cos(th1),-sin(th1),0]';
eth2 = [-cos(th2+th1),-sin(th2+th1),0]';

VB = l1*thdot1*eth1; % ANGULAR VELOCITY
Ve = VB + l2*(thdot2+thdot1)*eth2; % ANGULAR VELOCITY

J = jacobian(Ve,[thdot1; thdot2]); % 3ala9a between Ve and qdot

syms Kp Kd xt yt xdott ydott real

zt = [xt yt 0 ]'; %Trajectory tracked (desired?)
ztdot = [xdott ydott 0]'; %velocity tracked (desired?)
Ta = J' * ( Kp*(zt - ra_e) + Kd*(ztdot - J*[thdot1 thdot2]') );

matlabFunction(Ta, 'file', 'MyImpedenceControl');
