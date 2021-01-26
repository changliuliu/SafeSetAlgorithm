%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test on the freeway scenario 
% Denso: Road Project
% Feature: Multiple-Vehicles, Learning and Decision Making
%
% Changliu Liu
% 2015.12
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function robotnew=robotmove(t,robot,agent)
Q1=[1 1;1 10];
Q2=[4 1; 1 1];
goal=robot.Goal;
dmin=25;
kdot=50;

if goal(2)-0.5 > ceil(robot.x(2,end))
    goal(2) = ceil(robot.x(2,end))+0.5
end

% Efficiency Controller
% if robot.x(2,end)-goal(2)>0.5
%     goal(2)=ceil(robot.x(2,end));
% end
%u(1)=0.1*norm(robot.x(1:2,end)-goal(1:2))*cos(atan((robot.x(2,end)-goal(2))/(robot.x(1,end)-goal(1)))-robot.x(4,end))-0.4*(robot.x(3,end)-goal(3));
if robot.x(3,end)-goal(3)<0
    u(1)=-0.07*(robot.x(1:2,end)-goal(1:2))'*[cos(robot.x(4,end));sin(robot.x(4,end))]-0.5*(robot.x(3,end)-goal(3));
else
    u(1)=-0.5*(robot.x(3,end)-goal(3));
end
if robot.x(2,end)-goal(2)<-0.5
    u(2)=-0.3*(robot.x(2,end)-goal(2))+0.1*(-robot.x(4,end));
else
    u(2)=-0.1*(robot.x(2,end)-goal(2))+3*(-robot.x(4,end));
end
u(2)=mod(u(2)+pi,2*pi)-pi;
%consider for the satuation
if u(1)>2
    u(1)=2;end
if u(1)<-3
    u(1)=-3;end
if norm(u(2))>0.05
    u(2)=0.05*u(2)/norm(u(2));end

newstate=vehicle_dynamic_update(robot.x(:,t),u,robot.delta_t);

% Learning
% ua=[0;0];
% for i=1:size(agent,2)
%     if t==1
%         
%         PreviousInfer = [1;0;0];
%         robot.inference{i}(:,1)=[1;0;0];
%         robot.predict{i}.B{1}.par=[1;1;1];
%         robot.predict{i}.B{2}.par=[1;1;1;1;1];
%         robot.predict{i}.B{3}.par=[1;1;1;1;1];
%         ua = [0;0];
%         
%     else
%         PreviousInfer = robot.inference{i}(:,t-1);
%         
%         [prob, PreviousAction] = max(robot.inference{i}(:,t-1));
%         Features = GetFeature(agent{i}.x,t);
%         if Features(6)==-1
%             [~,leftright] = max(PreviousInfer(2:3));
%             robot.inference{i}(:,t) = [0;0;0];
%             robot.inference{i}(leftright+1,t) = 1;
%         else
%             if PreviousInfer(1) ==0
%                 robot.inference{i}(:,t)=[1;0;0];
%             else
%                 robot.inference{i}(:,t) = infer(PreviousInfer,Features);
%             end
%         end
%         [prob, Action] = max(robot.inference{i}(:,t));
%         if Action == PreviousAction
%             robot.predict{i} = PAA(robot.predict{i},Action,agent{i}.x,t);
%         else
%             robot.predict{i}.B{Action}.par(:,end+1) =  robot.predict{i}.B{Action}.par(:,end);
%             robot.predict{i}.B{Action}.par(end,end) =  t;
%         end
%         
%         par = robot.predict{i}.B{Action}.par(1:end-1,end);
%         if Action == 1
%             ua(1) = par'*[Features(4);Features(5)];
%             ua(2) = 0;
%         else
%             ua(1) = par(1:2)'*[Features(4);Features(5)];
%             ua(2) = par(3:4)'*[Features(8);Features(10)];        
%         end
%         
%     end
%     robot.predict{i}.x(:,t+1)= vehicle_dynamic_update(agent{i}.x(:,t),ua,robot.delta_t);
% end


% Safety Controller
Lstack=[];Sstack=[];flag=0;
for i=1:size(agent,2)
    obstacle=agent{i}.x;
    theta_rel=robot.x(4,t)-angle(obstacle(1:2,t),robot.x(1:2,t));
    L=[cos(theta_rel),-robot.x(3,end)*sin(theta_rel)];
    if cos(theta_rel)<0.1
        L(1)=0;
    end
    
    d=norm(robot.x(1:2,t)-obstacle(1:2,t));
    d=max([d,1])
    ddot=((robot.x(1,t)-obstacle(1,t))*(robot.x(3,t)*cos(robot.x(4,t))-...
        obstacle(3,t)*cos(obstacle(4,t)))+...
        (robot.x(2,t)-obstacle(2,t))*(robot.x(3,t)*sin(robot.x(4,t))-...
        obstacle(3,t)*sin(obstacle(4,t))));
    
    drel=norm(robot.x(2,t)-obstacle(2,t));
    vrel=robot.x(3,t)*sin(robot.x(4,t))-obstacle(3,t)*sin(obstacle(4,t));
    

    dis(t)=norm(robot.x(1,t)-obstacle(1,t));
    ua = [0;0];
    thres=-2+2*d*ddot-ddot^2/d+vrel^2/d-(robot.x(1:2,t)-obstacle(1:2,t))'*ua/d;
    %first, check if the safety index is positive   
    
    if dmin^2-d^2-kdot*ddot>0 && (drel<0.7 || norm(goal(2)-agent{i}.x(2,t))<0.5)
        flag=1;
        %thres=2*drel*vrel-kdot*vrel^2/d;
        Lstack=[Lstack; L];
        Sstack=[Sstack;thres];
        if norm(goal(2)-obstacle(2,t))<0.9 && drel>0.5
            if obstacle(3,t)>robot.x(3,t)-1
                goal(3)=25;
            else
                goal(3)=35;
            end
        end
    end
end
robot.ssa(t)=0;
if flag==1
    if norm(robot.x(2,t)-goal(2))<0.5
        Q=Q1;
    else 
        Q=Q2;
    end
    Lsat=[1 0;-1 0; 0 1; 0 -1];
    if (robot.x(2,t)<0.9) && (robot.x(4,t)<=0)
        Ssat=[2;3;0.05;0];
    else
        Ssat=[2;3;0.05;0.05];
    end
    %unew = quadprog(Q,-u',[Lstack],[Sstack]);
    %unew = quadprog(eye(2),-unew',[Lsat],[Ssat]);
    unew = quadprog(Q,u',[Lstack;Lsat],[Sstack;Ssat]);
    aug = 100;
    while size(unew,1)<1
        Q=Q+Lstack'*Lstack;
        f=u+Sstack'*Lstack;
        Sstack = Sstack + aug
        unew = quadprog(Q,u',[Lstack;Lsat],[Sstack;Ssat]);
        %unew = quadprog(Q,f',Lsat,Ssat);
    end
    if norm(unew-u')>0.01
        robot.ssa(t)=1;
    end
    u=unew';
end


if dmin^2-d^2-100*ddot<0 && goal(3)<30
    goal(3)=30;
end


robot.u(1:2,t)=u;

robot.x(:,t+1)=vehicle_dynamic_update(robot.x(:,t),u,robot.delta_t);
robot.goal=goal;
robotnew=robot;
end