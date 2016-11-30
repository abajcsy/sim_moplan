function tau_a = impedence_control(Kd,Kp,dtheta1,dtheta2,l1,l2,theta1,theta2,xdott,xt,ydott,yt)
%IMPEDENCE_CONTROL
%    TAU_A = IMPEDENCE_CONTROL(KD,KP,DTHETA1,DTHETA2,L1,L2,THETA1,THETA2,XDOTT,XT,YDOTT,YT)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    29-Nov-2016 15:30:01

t2 = theta1+theta2;
t3 = sin(t2);
t4 = l2.*t3;
t5 = sin(theta1);
t6 = l1.*t5;
t7 = t4+t6;
t8 = cos(t2);
t9 = l2.*t8;
t10 = cos(theta1);
t11 = l1.*t10;
t12 = t9+t11;
t13 = dtheta1.*t7;
t14 = dtheta2.*l2.*t3;
t15 = t13+t14+xdott;
t16 = Kd.*t15;
t17 = t9+t11-xt;
t18 = t16-Kp.*t17;
t19 = t4+t6-yt;
t20 = Kp.*t19;
t21 = dtheta1.*t12;
t22 = dtheta2.*l2.*t8;
t23 = t21+t22-ydott;
t24 = Kd.*t23;
t25 = t20+t24;
tau_a = [-t7.*t18-t12.*t25;-l2.*t3.*t18-l2.*t8.*t25];
