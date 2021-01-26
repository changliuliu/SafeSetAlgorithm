function F=GetFeature(x,t)
F=[];
dt=0.05;vavg=30;vfront=x(3,t);
% Longitudinal accelaration
if t>1
    F(1)=(x(3,t-1)-x(3,t))/dt;
else
    F(1)=0;
end
% Deccelaration light
F(2)=0;
% Turn signal
F(3)=0;
% Speed relative to the traffic flow
F(4)=x(3,t)-vavg;
% Speed relative to the front vehicle
F(5)=x(3,t)-vfront;
% Current lane id
if x(2,t)-floor(x(2,t))>0.85 || x(2,t)-floor(x(2,t))<0.15
    F(6)=-1;
else
    F(6)=ceil(x(2,t));
end
% Current lane clearance
F(7)=0;
% Lateral velocity
F(8)=x(3,t)*sin(x(4,t));
% Lateral deviation from the center of its current lane
if F(6)~=-1
    F(9)=x(2,t)-F(6)+0.5;
else
    F(9)=-1;
end
% Lateral deviation from the center of its target lane
F(10)=F(9);
end