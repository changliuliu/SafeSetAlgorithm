function [ tau ] = rbt_inv_dyna_frc( x,param )
%RBT_INV_DYNA inverse dynamics of 2 link robot
%   x=[theta1,theta2,theta1_d,theta2_d,theta1_dd,theta2_dd]
alpha=param(1);
beta=param(2);
delta=param(3);
fc1=param(4);
fc2=param(5);
fv1=param(6);
fv2=param(7);
M=[alpha+2*beta*cos(x(2)),delta+beta*cos(x(2));...
    delta+beta*cos(x(2)),delta];
C=beta*[-(2*x(3)+x(4))*x(4);x(3)^2]*sin(x(2));
f=[fc1*sign(x(3))+fv1*x(3);fc2*sign(x(4)-x(3))+fv2*(x(4)-x(3))];
tau=M*[x(5);x(6)]+C+f;


end

