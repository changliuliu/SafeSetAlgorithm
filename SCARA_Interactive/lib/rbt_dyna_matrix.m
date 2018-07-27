function [ M,C ] = rbt_dyna_matrix( x,param)
%RBT_FWD_DYNA_ODE forward robot dynamics for 2 link robot, without friction
%and motor dynamics
%   x=[theta1,theta2,theta1_d,theta2_d]
alpha=param(1);
beta=param(2);
delta=param(3);
M=[alpha+2*beta*cos(x(2)),delta+beta*cos(x(2));...
    delta+beta*cos(x(2)),delta];
C=beta*[-(2*x(3)+x(4))*x(4);x(3)^2]*sin(x(2));
end
