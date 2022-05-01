
%% Linear Segments with Parabolic Blends (PSPB)
% From: http://www.ee.nmt.edu/~wedeward/EE570/SP09/gentraj.html

clear all;

% intial time, joint value and joint velocity
t0 = 0; 
tf = 7;
% final time, joint value and joint velocity
q0 = -0.7; 
qf = 0.5;
% constant velocity and blend time
V = 0.2; 
tb = (q0 - qf + V*tf)/V;

% sanity check that V is within limits
Vmin = (qf - q0)/tf;
if (V < Vmin || V > 2*Vmin) % this check assumes V positive
    display(['V = ',num2str(V), ' is not within limits',...
            '(',num2str(Vmin),', ',num2str(2*Vmin),')']);
    display('LSPB will not be correct!');
end;

% coefficients for linear and parabolic blending
a = q0; b = 0; c = (V-b)/(2*tb);
d = (q0+qf-V*tf)/2;
e = qf - (V*tf^2)/(2*tb); f = V*tf/tb; g = -V/(2*tb); %(V*tf + q0 - qf + 2*tb*(f-V))/(2*tb^2);
% b(1) = qf - (V*tf^2)/(2*tb); b(2) = V*tf/tb; b(3) = -V/(2*tb);

% generate trajectory for time interval
t = t0:(tf-t0)/500:tf;
q = (a + b*t + c*t.^2).*(t<=tb) + ...
    (d + V*t).*((t>tb)-(t>=(tf-tb))) + ...
    (e + f*t + g*t.^2).*(t>(tf-tb));
qdot = (b + 2*c*t).*(t<=tb) + ...
       V.*((t>tb)-(t>=(tf-tb))) + ...
       (f + 2*g*t).*(t>(tf-tb));
qddot = 2*c*(t<=tb) + ...
        0*((t>tb)-(t>=(tf-tb))) + ...
        2*g*(t>(tf-tb));

% a(1) = q0; a(2) = 0; a(3) = V/(2*tb);
% b(1) = qf - (V*tf^2)/(2*tb); b(2) = V*tf/tb; b(3) = -V/(2*tb);
% 
% % generate traj for time interval
% t = t0:(tf-t0)/500:tf;
% q = (a(1) + a(2)*t + a(3)*t.^2).*(t<=tb) + ...
%     ((qf + q0 - V*tf)/2 + V*t).*((t>tb)-(t>=(tf-tb))) + ...
%     (b(1) + b(2)*t + b(3)*t.^2).*(t>(tf-tb));
% qdot = (a(2) + 2*a(3)*t).*(t<=tb) + ...
%        V.*((t>tb)-(t>=(tf-tb))) + ...
%        (b(2) + 2*b(3)*t).*(t>(tf-tb));
% qddot = 2*a(3)*(t<=tb) + ...
%         0*((t>tb)-(t>=(tf-tb))) + ...
%         2*b(3)*(t>(tf-tb));
    
plot(t,q,'b-',t,qdot,'g--',t,qddot,'r-.','LineWidth',2);
legend('position','velocity','acceleration');
xlabel('time (sec)'); ylabel('joint trajectory');
title('Trajectory using LSPB');
