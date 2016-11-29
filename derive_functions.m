% Derive the kinematics and dynamics equationss with the elbow angle
% relative to the upper arm angle
% Reference: http://summerschool.stiff-project.org/fileadmin/pdf/1804_C19.pdf

syms x y m1 m2 I1 I2 g l1 l2 d1 d2 theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2 er1 er2 eth1 eth2 tau1 tau2 real

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Fwd & Inv Kinematics %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ee_kin = [l1*cos(theta1) + l2*cos(theta1+theta2);  
          l1*sin(theta1) + l2*sin(theta1+theta2);
          0]

matlabFunction(ee_kin, 'file', 'forward_kin');

c2 = (x^2+y^2-l1^2-l2^2)/(2*l1*l2);
s2 = sqrt(1 - c2^2);
t2 = atan2(s2, c2);
k1 = l1 + l2*c2;
k2 = l2*s2;
t1 = atan2(y,x) - atan2(k2, k1);
ee_inv_kin = [t1; t2];

matlabFunction(ee_inv_kin, 'file', 'inv_kin');

%%%%%%%%%%%

% force at end effector
syms Fdx Fdy real
Fd = [Fdx; 
      Fdy; 
      0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Acceleration computation %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

H11 = m1*d1^2 + I1 + m2*(l1^2 + d2^2 + 2*l1*d2*cos(theta2)) + I2;
H22 = m2*d2^2 + I2;
H12 = m2*(d2^2+l1*d2*cos(theta2)) + I2;
h = m2*l1*d2*sin(theta2);
G1 = m1*d1*g*cos(theta1) + m2*g*(d2*cos(theta1+theta2)+l1*cos(theta1));
G2 = m2*g*d2*cos(theta1+theta2);

tau_eqn1 = H11*ddtheta1 + H12*ddtheta2 - h*dtheta2^2 - 2*h*dtheta1*dtheta2 + G1 == tau1;
tau_eqn2 = H22*ddtheta2 + H12*ddtheta1 + h*dtheta1^2 + G1 == tau2;

sol = solve([tau_eqn1, tau_eqn2], [ddtheta1, ddtheta2]);
eqn1 = sol.ddtheta1;
eqn2 = sol.ddtheta2;

matlabFunction(eqn1, 'file', 'compute_ddtheta1');
matlabFunction(eqn2, 'file', 'compute_ddtheta2');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Gravity Compensation %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tau2Eq = simplify(solve((solve(eqn1,tau1)-solve(eqn2,tau1)),tau2));

tau1Eq = simplify(subs(solve(eqn1,tau1),tau2,tau2Eq));

matlabFunction(tau1Eq, 'file', 'grav_comp_tau1');
matlabFunction(tau2Eq, 'file', 'grav_comp_tau2');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Impedance control %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% rotating reference frames
% er1 = [-sin(theta1);
%        cos(theta1); 
%        0]; 
% er2 = [-sin(theta2+theta1);
%        cos(theta2+theta1);
%        0];
% 
% eth1 = [-cos(theta1);
%         -sin(theta1);
%         0];
% eth2 = [-cos(theta2+theta1);
%         -sin(theta2+theta1);
%         0];
% 
% % a = fixed base point, b = elbow point
% % c1 = link 1 center of mass, c2 = link 2 center of mass 
% % e = end-effector
% 
% ra_c1 = d1*er1  % rotation of link 1 center of mass around base point
% rb_c2 = d2*er2  % rotation of link 2 center of mass around elbow point
% rb_e = l2*er2   % rotation of end-effector around elbow point
% ra_b = l1*er1   % rotation of elbow point around base point
% ra_c2 = ra_b + rb_c2    % rotation of link 2 center of mass around base point 
% ra_e = ra_b + rb_e      % rotation of end-effector around base point

% jacobian relating end effector velocity to joint space vel
% i.e. x_dot = J(q)*dq
% i.e. Ve = J*qv

% velocities
% Vc1 = d1*dtheta1*eth1;
% VB = l1*dtheta1*eth1;
% Vc2 = VB + d2*(dtheta2+dtheta1)*eth2;
% Ve = VB + l2*(dtheta2+dtheta1)*eth2;

V_c1 = [-d1*dtheta1*sin(theta1);
        d1*dtheta1*cos(theta1);
        0];
V_c2 = [-(l1*sin(theta1) + d2*sin(theta1+theta2))*dtheta1 - d2*sin(theta1+theta2)*dtheta2;
        (l1*cos(theta1) + d2*cos(theta1+theta2))*dtheta1 + d2*cos(theta1+theta2)*dtheta2;
        0];
Ve = [-(l1*sin(theta1) + l2*sin(theta1+theta2))*dtheta1 - l2*sin(theta1+theta2)*dtheta2;
        (l1*cos(theta1) + l2*cos(theta1+theta2))*dtheta1 + l2*cos(theta1+theta2)*dtheta2;
        0];

J = jacobian(Ve,[dtheta1 dtheta2])

syms Kp Kd xt yt xdott ydott real

% tracked trajectory 
zt = [xt; 
      yt;
      0]; 
% tracked velocity 
ztdot = [xdott;
         ydott;
         0]; 

tau_a = J'*(Kp*(zt - ee_kin) + Kd*(ztdot - J*[dtheta1 dtheta2]'));

matlabFunction(tau_a, 'file', 'impedence_control');