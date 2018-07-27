
function [status,agentnew]=agentmove(t,agent)


status=0;

if status==0
    horizon=5;
    goal=[agent.Goal(1,1);agent.Goal(2,1);0;0];
     u=agent.K*(agent.x(1:4,t)-goal);%*(log(d/3));%(agent.obs.xstar(1:2:3,end)-agent.obs.xself(1:2:3,end))/norm(agent.obs.xstar(1:2:3,end)-agent.obs.xself(1:2:3,end))^2;
     if norm(u)>agent.umax
         u=u.*(agent.umax/norm(u));
     end
     agent.x(1:4,t+1)=agent.A*agent.x(1:4,t)+agent.B*u;
     agent.Astar{t}=agent.A+agent.B*agent.K;
     agent.Bstar{t}=[zeros(4,4) -agent.B*agent.K];

end
agentnew=agent;
end