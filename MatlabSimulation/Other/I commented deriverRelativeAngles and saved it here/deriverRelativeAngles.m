%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Derive double pendulum dynamics %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Derive double pendulum equations of motion, write to Thdotdot1 and
%Thdotdot2 matlab function files. THIS VERSION DERIVES IT WITH THE ELBOW
%ANGLE RELATIVE TO THE UPPER ARM ANGLE.

%Parameters symbolically.
syms m1 m2 I1 I2 g l1 l2 d1 d2 th1 th2 thdot1 thdot2 thdotdot1 thdotdot2 er1 er2 eth1 eth2 T1 T2 real % this is reating the varaibles seen here, they are variables to be used in formulas

%Unit vectors, cartesian, these are used just to do some fancy math but at
%the very end, M_A and M_B and all those formulas, they're gonna have their
%final results in a one dimentional format in the third column of the
%matrix kinda neat stuff but way too complicated, he could've just
%hardcoded the selections
i = [1 0 0]';
j = [0 1 0]';
k = [0 0 1]';

%Rotating reference frames
    % error of the first arm along x axis and y axis
er1 = [-sin(th1), cos(th1), 0]'; %Just changed it so 0 is upright.
    % 
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

matlabFunction(ra_e, 'file', 'ForwardKin'); % by converting the function ra_e into a matlabFunction file, we can call it and give it parameters that we want and it would return the result instead of having to modify the variables in this code that ra_e depends on

%Velocities
Vc1 = d1*thdot1*eth1; % ANGULAR VELOCITY
VB = l1*thdot1*eth1; % ANGULAR VELOCITY
Vc2 = VB + d2*(thdot2+thdot1)*eth2; % ANGULAR VELOCITY
Ve = VB + l2*(thdot2+thdot1)*eth2; % ANGULAR VELOCITY

%Accelerations
Ac1 = d1*thdotdot1*eth1 - d1*thdot1^2*er1;
AB = l1*thdotdot1*eth1 - l1*thdot1^2*er1;
Ac2 = d2*(thdotdot2+thdotdot1)*eth2 - d2*(thdot1 + thdot2)^2*er2  + AB;

%Force at end effector
syms Fdx Fdy real    
Fd = [Fdx, Fdy, 0]';

%AMB for just link 2:
    % maybe this is just the addition of all forces acting on it???
    % notice that the first term of this formula is a force applied to the
    % center of mass of the second arm, while the second term of this formula
    % is a force applied at the very end of the second arm a.k.a the end
    % effector
    
% this is kinda jam3 el 9iwa el mo2athira,
%
% the first term is literally just
% multiplying the Y projection of the vector from point B to center of mass
%  of second link by the force of gravity coming from down below
%
% and second term is impedence control's tork + it's gravity compensation
M_B = cross(rb_c2,-m2*g*j) + T2*k     + cross(rb_e,Fd); %Last term is force at end effector.

Hdot2 = I2*(thdotdot2+thdotdot1)*k + cross(rb_c2, m2*Ac2); % this is more like majmou3 el 9iwa da2iriya, idk what it's called, tork formula

eqn2forthdotdot2 = solve(dot(Hdot2 - M_B,k),thdotdot2); % this is extracting thdotdot2 from the final formula of dot(Hdot2 - M_B,k) = 0 (the solve function itself puts the given formula as = 0 ans solves for the wanted variable)

%AMB for whole thing:
M_A = cross(ra_c2,-m2*g*j) + cross(ra_c1,-m1*g*j) + T1*k + cross(ra_e,Fd); %Gravity for both, plus a control torque. Last term is force at end effector
Hdot1 = I2*(thdotdot2+thdotdot1)*k + cross(ra_c2, m2*Ac2) + I1*thdotdot1*k + cross(ra_c1, m1*Ac1);

eqn1forthdotdot1 = solve(dot(Hdot1 - M_A,k),thdotdot1);

%One equation for thdotdot1, one for thdotdot2.
eqn2 = simplify(solve(subs(eqn2forthdotdot2, thdotdot1, eqn1forthdotdot1)-thdotdot2,thdotdot2)); % subs will take where ever there is thdotdot1 in eqn2forthdotdot2 and replaces it with its actual formula eqn1forthdotdot1
    % we are adding -thdotdot2 just because we know the solve function will
    % put the entire thing as = 0 and solves for our chosen variable but
    % during that process it'll set the thdotdot2 on the other side to
    % zero, so we just substracting it from the formula bfr using solve()
eqn1 = simplify(solve(subs(eqn1forthdotdot1, thdotdot2, eqn2forthdotdot2)-thdotdot1,thdotdot1));

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

J = jacobian(Ve,[thdot1; thdot2]);

syms Kp Kd xt yt xdott ydott real

zt = [xt yt 0 ]'; %Trajectory tracked (desired?)
ztdot = [xdott ydott 0]'; %velocity tracked (desired?)

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

