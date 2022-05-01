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
    
     % so all these four are rotation matrices found from the homogenous transformation matrices T01 and T12 and T02
     % but he flipped the x axis coordinate stuff with the y axis stuff i
     % assume so 0 is considered up, he also multiplied eth1 an eth2 by -
     % for some reason
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
    
% this is kinda jam3 el 9iwa el mo2athira,
%
% the first term is literally just
% multiplying the Y projection of the vector from point B to center of mass
%  of second link by the force of gravity coming from down below
%
% and second term is impedence control's tork + it's gravity compensation
M_B = cross(rb_c2,-m2*g*j) + T2*k     + cross(rb_e,Fd); %Last term is force at end effector.
% cross(rb_c2,-m2*g*j) is literally G(th) for theta 1 with cosine replaced with sine

% M_B is the external forces from the environment

% HDOT2 is the inertia matrix of the manipulator maybe?
Hdot2 = I2*(thdotdot2+thdotdot1)*k + cross(rb_c2, m2*Ac2); % 

eqn2forthdotdot2 = solve(dot(Hdot2 - M_B,k),thdotdot2); % this is extracting thdotdot2 from the final formula of dot(Hdot2 - M_B,k) = 0 (the solve function itself puts the given formula as = 0 ans solves for the wanted variable)

%AMB for whole thing:
% in this research paper https://pdf.sciencedirectassets.com/314898/1-s2.0-S1474667016X60624/1-s2.0-S1474667016415016/main.pdf?X-Amz-Security-Token=IQoJb3JpZ2luX2VjEJz%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FwEaCXVzLWVhc3QtMSJIMEYCIQDgK43kDpM2b%2Bq%2B4%2BHFnKQLU5joPJWJ73SzaddepR2yCAIhANg7bPtrmU2Q3TQut94lvqFlEDy5FeyTOU5lf6%2Ba8eE0KvoDCBUQBBoMMDU5MDAzNTQ2ODY1IgwUUO7TZGusQg6nFfEq1wMEoqYHvDQGRzvUbstS1AGo0ngNGzcXEoaw91fkW4Sabl7MSWMa41i%2FHtc82sb6cXuwdyCsmj0tAefHPrbWfEukw2SoTwjfTrn0H8hg7aZd%2Fh2EzLpWXScNRpLIOl4XFOi9NklXdlxO1wgX3OmIIQ83Q2MDs8bFqj11zJ2kttDLkpDLB72nGcnPz9EHKZor%2BjwU0cPmI58WCbYqJ4E%2BH6Xb0a4Ig27Q2jw%2F3FYRnCc6x2Wx25mT42bZdlrLzTN9bD3mdZOulIAOQBbPD0jjUOQ10Xegep5FcG3SdjG2IrgaPqKiMfJ9WoC5LnSa%2FWiL95zlIAcHPDKvzq%2FtpmlX5AKmKXPSg0Ys%2BcU8xQiA9lQ9lMEiszUS%2FyTf7QDsGZZVtMgzJW8Q1hdr4N1gcYSVmeRmP7dGuaj3zFkBvqU3JmJetZoS6jAn%2Bw3Do%2BUbGA6kZ8%2FzGM57HU4Xr0%2FnyGgLbBQNLZbmLGd8liKgTOIJp9QlALw1s4Mx%2BjbreUf7gXvwWNMOA%2BpVzZIAeF2lF4MlwbUhvZ9%2FLKZpR%2F4uln3XD1OImItxhWcIk9wPhy0wyWdRP9S2vtA60ZsqXbbU9G8fk8pyYlOcohtyPiSxrb6FfvzmrQQQMF3Inc8w9uWskQY6pAEkA5VW44W1Kz9K1au6Tw4gwsApx5s6eq13Bs2cbYpenkFFRTUU%2FtI%2FmYwOHvZhUc6vtZENYf7K3FvfPPMqQjCp8If6AciTW1QruB8eZruOTyf1M6FTmgFoHPMXd7ogM3NoMrChXNCzA5Z6m09jGHo4q6lpgsFE2cPCyeAig055sYMoHo58L8gfGps%2B1nKEvYVafZhefLEracnAgYjC8iCU%2BxgP1A%3D%3D&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Date=20220311T124335Z&X-Amz-SignedHeaders=host&X-Amz-Expires=300&X-Amz-Credential=ASIAQ3PHCVTYTKLM27OE%2F20220311%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Signature=a0a42e1bf48bec8d4720d69f43579243d28e2eedc7f7dbd81fab3685c771d6d2&hash=8d4dd0919c2cc06d03181a430f853a185990db7cb530d18ba9400fd376e31784&host=68042c943591013ac2b2430a89b270f6af2c76d8dfd086a07176afe7c76c2c61&pii=S1474667016415016&tid=spdf-7378119e-37e2-479b-9432-5564a62f90db&sid=0ce70cd71531544d739b11a2289d7b69543dgxrqb&type=client&ua=53020057525353595000&rr=6ea460677b555385
%   they ignored the tork resulted from the motor turning, but in this code
%   he did not ignore it

M_A = cross(ra_c2,-m2*g*j) + cross(ra_c1,-m1*g*j) + T1*k + cross(ra_e,Fd); %Gravity for both, plus a control torque. Last term is force at end effector
% cross(ra_c2,-m2*g*j) + cross(ra_c1,-m1*g*j) is literally G(th) for theta
% 2 with cosine replaced with sine

Hdot1 = I2*(thdotdot2+thdotdot1)*k + I1*thdotdot1*k      +       cross(ra_c2, m2*Ac2) + cross(ra_c1, m1*Ac1);

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

% this jacobian here is not the analytic jacobian but instead the geometric
% jacobian
J = jacobian(Ve,[thdot1; thdot2]); % 3ala9a between Ve and qdot

syms Kp Kd xt yt xdott ydott real

zt = [xt yt 0 ]'; %Trajectory tracked (desired?)
ztdot = [xdott ydott 0]'; %velocity tracked (desired?)

% maybe the second term in this line is the direct kinematics
% differentiated? because look it's both a delta between desired and
% actual, so kind of df(q)/dq

% J*[thdot1 thdot2]' this here is literally re-obtaining velocity Ve again from
% Jacobian and angular velocities qdot a.k.a v = J(q) * qdot
Ta = J' * ( Kp*(zt - ra_e) + Kd*(ztdot - J*[thdot1 thdot2]') );

matlabFunction(Ta, 'file', 'ImpedenceControl');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Energy eqns %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  these r just majmou3 el energies acting on end effector, supposedly we
%  want the total energy to be zero?

%Turned this off after making sure stuff worked:

% syms k1 k2 real
% 
% PE = g*dot(ra_c2,j)*m2 + g*dot(ra_c1,j)*m1;
% KE = 1/2*I1*thdot1^2 + 1/2*I2*(thdot2+thdot1)^2 + 1/2*m1*dot(Vc1,Vc1) + 1/2*m2*dot(Vc2,Vc2);
%  
%  
% Etot = PE + KE;
% 
% matlabFunction(Etot, 'file', 'TotEnergy');

