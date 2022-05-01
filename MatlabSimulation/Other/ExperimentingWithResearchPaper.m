% reference: https://www.ijeert.org/papers/v6-i11/3.pdf

syms th1 th2 thdot1 thdot2 thdotdot1 thdotdot2 L1 L2 g m1 m2 T1 T2 real

T01 = [cos(th1), -sin(th1), 0, L1*cos(th1) ; 
        sin(th1), cos(th1), 0, L1*sin(th1); 
        0, 0, 1, 0; 
        0, 0, 0 1];
T12 = [cos(th2), -sin(th2), 0, L2*cos(th2) ; 
        sin(th2), cos(th2), 0, L2*sin(th2); 
        0, 0, 1, 0; 
        0, 0, 0 1];

%T02 = simplify(T01 * T12);
T02 = T01 * T12;

Px = T02(1, 4);
Py = T02(2, 4); % this is forward kinematics we should use it later




% we also need the jacobian, to get that we first:
% get the cartesian speed of the end effector which
% is just the derivative of Px,y
V = [ -L2*(thdot1+thdot2)*sin(th1+th2) - L1*thdot1*sin(th1) ;
      L2*(thdot1+thdot2)*cos(th1+th2) + L1*thdot1*cos(th1);
      0 ];

  
%Jacobian relating end effector velocity to joint space vel
% ie Ve = J*qv
J = jacobian(V,[thdot1; thdot2]);


syms Fdx Fdy real
% inverse kinematics
%solving_for_cos_th2 = -cos(solve(simplify(Px^2+Py^2), th2));
%solving_for_sin_th2 = sqrt(1 - solving_for_cos_th2.^2);
%th2_using_geometrical_inv_kinematics = atan(solving_for_cos_th2./solving_for_sin_th2);

M = [(m1+m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(th2),    m2*L2^2 + m2*L1*L2*cos(th2) ;
      m2*L2^2 + m2*L1*L2*cos(th2),                    m2*L2                       ];
  
B = [ -m2*L1*L2*(2*thdot1*thdot2+thdot2^2)*sin(th2)   ;
       m2*L1*L2*thdot1^2*sin(th2)                    ];

G = [ (m1+m2)*g*L1*cos(th1) + m2*g*L2*cos(th1+th2)    ;
       m2*g*L2*cos(th1+th2)                          ];
   
F = [Fdx;
    Fdy];


% since we know the jacobian, we can now determine thetadotdot
% using the lagrangian: M + B + G = F - J'*Imp
% Imp being the impedence control's suggested actions
% F being the force applied on the end effector
% M being the inertia matrix
% B being the centrifugal and coriolis effects
% G being the gravity effects
Lagrangian = M * [thdotdot1;thdotdot2] + B + G - [T1;T2] - F.*[Px; Py]; %- (F - J'*[T1, T2, 0]'); % we pulled everything to one side because the solve() function will set the other side to zero to obtain thetadotdot1 and thetadotdot2


% SOLVING TWO FORMULAS OF TWO UNKNOWNS THAT'S ALL HERE
% brought thdotdot to one side and left everything else on the other along
% with the other thdotdot
eqn1forthdotdot1 = solve(Lagrangian(1), thdotdot1);
eqn2forthdotdot2 = solve(Lagrangian(2), thdotdot2);


% now we need to replace the other thdotdot with its formula
EQthdotdot1 = simplify(solve(subs(eqn1forthdotdot1, thdotdot2, eqn2forthdotdot2)-thdotdot1,thdotdot1));
EQthdotdot2 = simplify(solve(subs(eqn2forthdotdot2, thdotdot1, eqn1forthdotdot1)-thdotdot2,thdotdot2));


matlabFunction(EQthdotdot1, 'file', 'Thdotdot1');
matlabFunction(EQthdotdot2, 'file', 'Thdotdot2');


% Gravity compensation
T2Eq = simplify(solve((solve(EQthdotdot1,T1)-solve(EQthdotdot2,T1)),T2));
T1Eq = simplify(subs(solve(EQthdotdot1,T1),T2,T2Eq));


matlabFunction(T1Eq, 'file', 'GravityCompT1');
matlabFunction(T2Eq, 'file', 'GravityCompT2');





syms desiredP desiredPdot Kp Kd real

% the cartesian force applied by our impedance control model (will be used
% in lagrangian up above)
currentP = ForwardKin(L1, L2, th1, th2);

Imp = Kp*(desiredP - currentP) + Kd*(desiredPdot - J*[thdot1 thdot2]'); % J*[thdot1 thdot2]' must be recalculated for every iteration of the system because they change

matlabFunction(Imp, 'file', 'ImpedenceControl');