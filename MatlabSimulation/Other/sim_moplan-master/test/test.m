%Initial conditions:
g = 9.81; 
m1 = 1.0; % mass of link 1
m2 = 1.0; % mass of link 2
l1 = 1.0; % length of link 1
l2 = 1.0; % length of link 2

d1 = arm.l1/2; % center of mass distance along link 1 
d2 = arm.l2/2; % center of mass distance along link 2 

% for various inertial ethetauations, see: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
% used inertial around center of mass: I_center = m*L^2/3
I1 = (1/3)*arm.m1*arm.l1^2; % moment of inertia of link 1 about center of mass     
I2 = (1/3)*arm.m2*arm.l2^2; % moment of inertia of link 2 about center of mass

% list of goal locations (cartesian)
 xy_goals = [-1, 1.5;
            -1, 1;
            -1, 0.5;
            -1, 0;
            -1, -0.5;
            -1, -1.0;
            -1, -1.5];
for i=1:7
    q = inv_kin(l1,l2,xy_goals(i,1),xy_goals(i,2))
end

% create all necessary code
%gen_functions()
    
% call open loop controller
%open_loop_control()

function gen_functions()
    syms x y m1 m2 I1 I2 g l1 l2 d1 d2 theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2 tau
    %%% generate forward kinematics
    ee_kin = [l1*cos(theta1) + l2*cos(theta1+theta2);  
              l1*sin(theta1) + l2*sin(theta1+theta2);
              0]

    matlabFunction(ee_kin, 'file', 'forward_kin');

    %%% generate inverse kinematics
    c2 = (x^2+y^2-l1^2-l2^2)/(2*l1*l2);
    s2 = sqrt(1 - c2^2);
    t2 = atan2(s2, c2);
    k1 = l1 + l2*c2;
    k2 = l2*s2;
    t1 = atan2(y,x) - atan2(k2, k1);
    ee_inv_kin = [t1; t2];

    matlabFunction(ee_inv_kin, 'file', 'inv_kin');

    %%% generate open loop controller
    theta = [theta1; theta2];  
    dtheta = [dtheta1; dtheta2]; 
    ddtheta = [ddtheta1; ddtheta2];
    % inertial matrix 
    M = getM(theta, l1, l2, m1, m2)
    % coriolis forces matrix
    C = getC(theta,dtheta, l1, m2, d2)
    % get normal forces
    %N = getN(theta,dtheta, l1, l2, m1, m2)
    % create open-loop control function
    tau_eqn = M*ddtheta + C*dtheta %+ N
    matlabFunction(tau_eqn, 'file', 'open_loop_control');
end

function M = getM(theta, l1, l2, m1, m2)
    M11 = (m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(theta(2));
    M12 = m2*l2^2 + m2*l1*l2*cos(theta(2));
    M21 = m2*l2^2 + m2*l1*l2*cos(theta(2));
    M22 = m2*l2^2;
    M = [M11, M12;
         M21, M22]; 
end

function C = getC(theta,dtheta, l1, m2, d2)
    C11 = m2*l1*d2*sin(theta(2))*dtheta(2);
    C12 = -m2*l1*d2*sin(theta(2))*(dtheta(1) + dtheta(2));
    C21 = m2*l1*d2*sin(theta(2))*dtheta(1);
    C22 = 0.0;
    C = [C11, C12;
         C21, C22]; 
end

% does linear function parabolic blending
function LFPB(theta_d)
    
end
