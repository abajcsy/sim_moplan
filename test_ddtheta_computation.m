syms theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2 m1 m2 l1 l2 d1 d2 g I1 I2 tau1 tau2

H11 = m1*d1^2 + I1 + m2*(l1^2 + d2^2 + 2*l1*d2*cos(theta2)) + I2;
H22 = m2*d2^2 + I2;
H12 = m2*(d2^2+l1*d2*cos(theta2)) + I2;
h = m2*l1*d2*sin(theta2);
G1 = m1*d1*g*cos(theta1) + m2*g*(d2*cos(theta1+theta2)+l1*cos(theta1));
G2 = m2*g*d2*cos(theta1+theta2);

eqn1 = H11*ddtheta1 + H12*ddtheta2 - h*dtheta2^2 - 2*h*dtheta1*dtheta2 + G1 == tau1;
eqn2 = H22*ddtheta2 + H12*ddtheta1 + h*dtheta1^2 + G1 == tau2;

sol = solve([eqn1, eqn2], [ddtheta1, ddtheta2]);
ddtheta1Sol = simplify(sol.ddtheta1)
ddtehta2Sol = simplify(sol.ddtheta2)