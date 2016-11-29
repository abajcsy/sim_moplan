% computes full dynamics for the 2 link arm robot
% uses autogenerated ddtheta1 and ddtheta2, impedence_control, and
% grav_comp_tau
function [zdot, tau1, tau2] = full_dynamics( t,z,arm )

theta1 = z(1);
theta2 = z(3);
dtheta1 = z(2);
dtheta2 = z(4);

% Fdx = 0;
% Fdy = 0;

% current disturbance force on end effector
Fx_curr = arm.Fx;
Fy_curr = arm.Fy;

% current Target
x_currTarget = arm.xtarget;
y_currTarget = arm.ytarget;

xdot_currTarget = 0;
ydot_currTarget = 0;

% get torque needed to track our desired point
tau = impedence_control(arm.Kd,arm.Kp,dtheta1,dtheta2,arm.l1,arm.l2,theta1,theta2,xdot_currTarget,x_currTarget,ydot_currTarget,y_currTarget);

% compute torques with gravity compensation
% tau1 = tau(1) + grav_comp_tau1(0,0,arm.d1,arm.d2,dtheta1,dtheta2,arm.g,arm.l1,arm.l2,arm.m1,arm.m2,theta1,theta2);
% tau2 = tau(2) + grav_comp_tau2(0,0,arm.d2,dtheta1,arm.g,arm.l1,arm.l2,arm.m2,theta1,theta2);
tau1 = tau(1) + grav_comp_tau1(arm.d1,arm.d2,dtheta1,dtheta2,arm.g,arm.l1,arm.m1,arm.m2,theta1,theta2);
tau2 = tau(2) + grav_comp_tau2(arm.d1,arm.d2,dtheta1,arm.g,arm.l1,arm.m1,arm.m2,theta1,theta2);

% compute acceleration 
% ddtheta1 = compute_ddtheta1(Fx_curr,Fy_curr,arm.I1,arm.I2,arm.d1,arm.d2,dtheta1,dtheta2,arm.g,arm.l1,arm.l2,arm.m1,arm.m2,tau1,tau2,theta1,theta2);
% ddtheta2 = compute_ddtheta2(Fx_curr,Fy_curr,arm.I1,arm.I2,arm.d1,arm.d2,dtheta1,dtheta2,arm.g,arm.l1,arm.l2,arm.m1,arm.m2,tau1,tau2,theta1,theta2);
ddtheta1 = compute_ddtheta1(arm.I1,arm.I2,arm.d1,arm.d2,dtheta1,dtheta2,arm.g,arm.l1,arm.m1,arm.m2,tau1,tau2,theta1,theta2);
ddtheta2 = compute_ddtheta2(arm.I1,arm.I2,arm.d1,arm.d2,dtheta1,dtheta2,arm.g,arm.l1,arm.m1,arm.m2,tau1,tau2,theta1,theta2);

%Assemble the state vector derivatives.
zdot = [dtheta1
        ddtheta1
        dtheta2
        ddtheta2];

end

