
% Human-Robot Interfacing software prototype
%   using impedance Control & Trajectory Optimization techniques

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% references
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   - https://drive.google.com/drive/folders/1OpBOQRvFL2EBWvRAA2XpjNUty3MaOqUA?usp=sharing


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trajactory Adaptation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constant system paramters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = []; % real current trajectory of the robot
Fh = []; % force applied to robot

M1 = 1; % paramter to adjust Alpha1
K1 = 1; % parameter to regulate the position error e
Mu1 = 1; % parameter to regulate the forgetting rate
v1 = 1; % parameter to adjust the adaptation of the interaction force Fh with Mu1

deltaT = 5; % paramter for sampling time of the system


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Operation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

i = size(xd) - 1; % number of sampling times

e = xd - x; % array of errors between all positions of x and xd

Epsilon1 = K1 * e^2; % performance evaluation index

% Index function J
J = 0;
for j=1:i
    J = J + Epsilon1(j) * exp( - Mu1 * (i - j) * deltaT ) ;
end

Alpha1 = 1 - exp( -v1 * J ); % variable to be designed according to a specific system to adjust the user's interaction profile
Beta1 = M1 * Alpha1; % an open parameter to regulate the interaction force

% final relation between x, xd and Fh
xd_trajectory_adaptation = x + Beta1 * Fh; % oh we experienced disturbing force Fh, let's see where we should move next to reduce Fh, we find xd



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Adaptive impedance Control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constant system paramters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

K2 = 1; % paramter to adjust the weight of the tracking error e
M2 = 1; % parameter to adjust Alpha2
Mu2 = 1; % paramter to regulate the forgetting rate
v2 = 1; % paramter to adjust the adaptation of the interaction force

e_derivative = diff(e);
Beta2 = M2 * Alpha2; % paramter to regulate the stiffness of the impedence controller
deltaK = Beta2 * abs(Fh);
Kd = 1; % paramter of damping can be variable too if we want
Kc0 = 1; % paramter of nominal stiffness, must be designed according to the experimental results
Kc = Kc0 + deltaK; % variable stiffness
Fim = Kc * e + Kd * e_derivative; % it's u, input of the impedance control


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Operation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Epsilon2 = K2 * e^2; % performance evaluation index

% Index function J2
J2 = 0;
for j=1:i
    J2 = J2 + Epsilon2(j) * exp( - Mu2 * (i - j) * deltaT ) ;
end

Alpha2 = exp( -v2 * J2 ); % variable to be designed according to a specific system to adjust the user's interaction profile
Beta2 = M2 * Alpha2; % an open parameter to regulate the interaction force

% final relation between x, xd and Fh
xd_impedence_control = x + Beta2 * Fh; % oh we experienced disturbing force Fh, let's see where we should move next to reduce Fh, we find xd


