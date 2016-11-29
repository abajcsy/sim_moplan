function tau2Eq = grav_comp_tau2(Fdx,Fdy,d1,d2,dtheta1,g,l1,l2,m1,m2,theta1,theta2)
%GRAV_COMP_TAU2
%    TAU2EQ = GRAV_COMP_TAU2(FDX,FDY,D1,D2,DTHETA1,G,L1,L2,M1,M2,THETA1,THETA2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    29-Nov-2016 00:37:39

t2 = theta1+theta2;
t3 = cos(t2);
t4 = cos(theta1);
tau2Eq = -Fdy.*l2.*t3+Fdx.*l2.*sin(t2)+d1.*g.*m1.*t4+d2.*g.*m2.*t3+g.*l1.*m2.*t4+d2.*dtheta1.^2.*l1.*m2.*sin(theta2);