function [ dx ] = rbt_fwd_dyna_frc( t,x,param,tau )
%RBT_FWD_DYNA_ODE forward robot dynamics for 2 link robot, without friction
%and motor dynamics
%   x=[theta1,theta2,theta1_d,theta2_d]
alpha=param(1);
beta=param(2);
delta=param(3);
if length(param)<4
    fc1=0;fc2=0;fv1=0;fv2=0;
else
fc1=param(4);
fc2=param(5);
fv1=param(6);
fv2=param(7);
end
M=[alpha+2*beta*cos(x(2)),delta+beta*cos(x(2));...
    delta+beta*cos(x(2)),delta];
C=beta*[-(2*x(3)+x(4))*x(4);x(3)^2]*sin(x(2));
f=[fc1*sign(x(3))+fv1*x(3);fc2*sign(x(4)-x(3))+fv2*(x(4)-x(3))];
dx=[x(3);x(4);inv(M)*(tau-C-f)];
end

