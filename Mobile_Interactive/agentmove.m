
function [status,agentnew]=agentmove(t,agent)


status=0;
% if agent.x(1:2:3,t)~=[-1;-1]
% agent.x(:,t+1)=agent.x(:,t)-[1;0;1;0];
% else
%    agent.x(:,t+1)=agent.x(:,t);
% end
% agentnew=agent;
% return
% check if a goal is reached
for i=1:agent.nG
    if norm(agent.x(1:2:3,t)-agent.Goal(:,i),2)<0.2
        agent.Goal=deleteGoal(agent.Goal,i);
        agent.nG=agent.nG-1;
        agent.score=agent.score+1;
        break
    end
end
%if the agent has reached all goals and is back to origin, then stop
if agent.nG==0 && agent.flag==1
    agentnew=agent;
    status=1;
    return
end
%if the agent has reached all goals but is not at the original
%position, then go back
if agent.nG==0 && agent.flag==0
    agent.nG=1;
    agent.Goal=agent.x(1:2:3,1);
    agent.flag=1;
    status=0;
end
%check if the current goal is the nearest or not
for i=1:agent.nG
    for j=i+1:agent.nG
        if norm(agent.obs.xself(1:2:3,end)-agent.Goal(:,j))<norm(agent.obs.xself(1:2:3,end)-agent.Goal(:,i))-1
            g=agent.Goal(:,i);
            agent.Goal(:,i)=agent.Goal(:,j);
            agent.Goal(:,j)=g;
        end
    end
end
if status==0
    horizon=5;
    goal=[agent.Goal(1,1);0;agent.Goal(2,1);0];
%     cvx_begin
%     cvx quite
%     variable x(4,horizon)
%     variable u(2,horizon)
%     minimize norm(x(1:4,horizon)-goal,2);
%     subject to 
%     x(1:4,1)==agent.A*agent.obs.xself(1:4,end)+agent.B*u(:,1);
%     for i=2:horizon
%         x(1:4,i)==agent.A*x(1:4,i-1)+agent.B*u(:,i);
%     end
%     for i=1:horizon
%         norm(u(:,i),2)<=agent.umax
%     end
%     cvx_end
%     agent.x(1:4,t+1)=x(:,1);
    status=0;
%     if t==500
%         agent.K=agent.K-2*[1 0 0 0;0 0 1 0];
%     end
    d=norm(agent.obs.xstar(1:2:3,end)-agent.obs.xself(1:2:3,end));
    agent.K=[-1 -2 0 0;0 0 -1 -2];
     u=(agent.K)*(agent.x(1:4,t)-goal)*(log(d/3));%(agent.obs.xstar(1:2:3,end)-agent.obs.xself(1:2:3,end))/norm(agent.obs.xstar(1:2:3,end)-agent.obs.xself(1:2:3,end))^2;
     agent.x(1:4,t+1)=agent.A*agent.x(1:4,t)+agent.B*u;
     agent.Astar{t}=agent.A+agent.B*agent.K;
     agent.Bstar{t}=[zeros(4,4) -agent.B*agent.K];

end
agentnew=agent;
end