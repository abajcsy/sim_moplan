function ee_inv_kin = inv_kin(l1,l2,x,y)
%INV_KIN
%    EE_INV_KIN = INV_KIN(L1,L2,X,Y)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    29-Nov-2016 15:29:57

t3 = l1.^2;
t4 = l2.^2;
t5 = x.^2;
t6 = y.^2;
t2 = t3+t4-t5-t6;
t7 = 1.0./l1.^2;
t8 = 1.0./l2.^2;
t9 = t2.^2;
t10 = t7.*t8.*t9.*(-1.0./4.0)+1.0;
t11 = sqrt(t10);
t12 = 1.0./l1;
ee_inv_kin = [angle(x+y.*1i)-angle(l1+l2.*t11.*1i-t2.*t12.*(1.0./2.0));angle(t11.*2.0i-(t2.*t12)./l2)];
