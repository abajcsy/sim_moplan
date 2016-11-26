%Initial conditions:
p.init = [pi/4    0.0    pi/4  0.0]';

p.g = 9.81;
p.m1 = 1; % mass of link 1
p.m2 = 1; % mass of link 2
p.l1 = 1; % length of link 1
p.l2 = 1; % length of link 2

p.d1 = p.l1/2; % center of mass distance along link 1 from the fixed joint
p.d2 = p.l2/2; % center of mass distance along link 2 from the fixed joint

p.I1 = 1/12*p.m1*p.l1^2; % moment of inertia of link 1 about COM
p.I2 = 1/12*p.m2*p.l2^2; % moment of inertia of link 2 about COM

%endZ = ForwardKin(p.l1,p.l2,p.init(1),p.init(3));
x0 = endZ(1); % end effector initial position in world space
y0 = endZ(2);
p.Fx = 0;
p.Fy = 0;

%%%%%%%% Control Parameters %%%%%%%%

% controller Gains
p.Kp = 10;
p.Kd = 8;

% single target:
p.xtarget = x0; % goal location in world space
p.ytarget = y0;