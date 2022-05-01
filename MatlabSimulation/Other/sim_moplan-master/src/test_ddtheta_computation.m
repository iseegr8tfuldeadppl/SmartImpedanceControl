syms theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2 m1 m2 l1 l2 d1 d2 g I1 I2 tau1 tau2 Fx Fy

Ve = [-(l1*sin(theta1) + l2*sin(theta1+theta2))*dtheta1 - l2*sin(theta1+theta2)*dtheta2;
        (l1*cos(theta1) + l2*cos(theta1+theta2))*dtheta1 + l2*cos(theta1+theta2)*dtheta2;
        0];

J = jacobian(Ve,[dtheta1 dtheta2])

tau_a = J'*[Fx; Fy; 0]

H11 = m1*d1^2 + I1 + m2*(l1^2 + d2^2 + 2*l1*d2*cos(theta2)) + I2;
H22 = m2*d2^2 + I2;
H12 = m2*(d2^2+l1*d2*cos(theta2)) + I2;
h = m2*l1*d2*sin(theta2);
G1 = m1*d1*g*cos(theta1) + m2*g*(d2*cos(theta1+theta2)+l1*cos(theta1));
G2 = m2*g*d2*cos(theta1+theta2);

eqn1 = H11*ddtheta1 + H12*ddtheta2 - h*dtheta2^2 - 2*h*dtheta1*dtheta2 + G1 == tau1 + tau_a(1);
eqn2 = H22*ddtheta2 + H12*ddtheta1 + h*dtheta1^2 + G1 == tau2 + tau_a(2);

sol = solve([eqn1, eqn2], [ddtheta1, ddtheta2]);
ddtheta1Sol = simplify(sol.ddtheta1)
ddtehta2Sol = simplify(sol.ddtheta2)

%%%%%%% test grav_comp_tau %%%%%%%

grav_comp_tau1 = g*((1/2)*m1+m2)*l1*cos(theta1) + (1/2)*m2*l2*cos(theta1 + theta2)
grav_comp_tau2 = g*((1/2)*m2*l2*cos(theta1+theta2))

F_g1 = -g*m1
F_g2 = -g*m2
F_q = J(1)'*F_g1 + J(2)'*F_g2
