%%%%%%%%%%%%%%%%%%%%%
% 2014.6.21
% Changliu Liu
% This function updates the discrete nonlinear dynamics
%%%%%%%%%%%%%%%%%%%%%
function xnext=vehicle_dynamic_update(x,u,dt)

xnext(3)= x(3)+u(1)*dt;
xnext(4)= x(4)+u(2)*dt; xnext(4)=mod(xnext(4)+pi,2*pi)-pi;
if abs(u(2))<0.1
    xnext(1)=x(1)+cos(x(4))*(u(1)*dt^2/2+x(3)*dt);
    xnext(2)=x(2)+sin(x(4))*(u(1)*dt^2/2+x(3)*dt);
else
xnext(1)= x(1)+(xnext(3)*sin(xnext(4))-x(3)*sin(x(4)))/u(2)+u(1)/u(2)^2*(cos(xnext(4))-cos(x(4)));
xnext(2)= x(2)-(xnext(3)*cos(xnext(4))-x(3)*cos(x(4)))/u(2)+u(1)/u(2)^2*(sin(xnext(4))-sin(x(4)));
end
end