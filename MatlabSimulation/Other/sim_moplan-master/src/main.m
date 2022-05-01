%Initial conditions:
arm.init = [pi/4;    % theta1
            0.0;     % dtheta1
            pi/4;    % theta2
            0.0];    % dtheta2

arm.g = 9.81; 
arm.m1 = 1.0; % mass of link 1
arm.m2 = 1.0; % mass of link 2
arm.l1 = 1.0; % length of link 1
arm.l2 = 1.0; % length of link 2

arm.d1 = arm.l1/2; % center of mass distance along link 1 
arm.d2 = arm.l2/2; % center of mass distance along link 2 

% for various inertial equations, see: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
% used inertial around center of mass: I_center = m*L^2/3
arm.I1 = (1/3)*arm.m1*arm.l1^2; % moment of inertia of link 1 about center of mass     
arm.I2 = (1/3)*arm.m2*arm.l2^2; % moment of inertia of link 2 about center of mass

% starting forces at end-effector
arm.Fx = 0;
arm.Fy = 0;

% controller gains
arm.Kp = 10; % stiffness
arm.Kd = 6;  % damping

% get end effector initial position in world space
ee_state = forward_kin(arm.l1,arm.l2,arm.init(1),arm.init(3))
x0 = ee_state(1); 
y0 = ee_state(2);

% goal location in world space
arm.xtarget = x0 
arm.ytarget = y0

% run simulation
plot_sim(arm)