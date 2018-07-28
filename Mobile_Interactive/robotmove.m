
function [status,robotnew]=robotmove(t,robot)


status=0;

%check if a goal is reached
for i=1:robot.nG
    goal=[robot.Goal(1,i);0;robot.Goal(2,i);0];
    if t>1
        x=robot.x(:,t-1)-goal;
    else
        x=robot.x(:,t)-goal;
    end
    if x'*robot.const.Q2*x<0
        time=min([robot.delta_t,-x'*robot.const.Q2*x/(x'*robot.const.Q3*x)]);
        d=x'*robot.const.Q1*x+time*2*(x'*robot.const.Q2*x)+time^2*(x'*robot.const.Q3*x);
    else
        d=x'*robot.const.Q1*x;
    end
    if norm(robot.x(1:2:3,t)-robot.Goal(:,i),2)<0.2 || sqrt(d)<0.2
        robot.Goal=deleteGoal(robot.Goal,i);
        robot.nG=robot.nG-1;
        robot.score=robot.score+1;
        break
    end
end
%if the robot has reached all goals and is back to origin, then stop
if robot.nG==0 && robot.flag==1
    robotnew=robot;
    status=1;
    return
end
%if the robot has reached all goals but is not at the original
%position, then go back
if robot.nG==0 && robot.flag==0
    robot.nG=1;
    robot.Goal=robot.x(1:2:3,t);
    robot.flag=1;
    status=0;
end
%check if the current goal is the nearest or not
for i=1:robot.nG
    goal1=[robot.Goal(1,i);0;robot.Goal(2,i);0];
    x1=robot.obs.xself(:,end)-goal1;
    time1=-x1'*robot.const.Q2*x1/(x1'*robot.const.Q3*x1);
    if time1<0
        time1=-5*time1;
    end
    for j=i+1:robot.nG
        
        goal2=[robot.Goal(1,j);0;robot.Goal(2,j);0];
        
        x2=robot.obs.xself(:,end)-goal2;
        
        time2=-x2'*robot.const.Q2*x2/(x2'*robot.const.Q3*x2);
        
        if time2<0
            time2=-5*time2;
        end
        if time2+0.3<time1
            g=robot.Goal(:,i);
            robot.Goal(:,i)=robot.Goal(:,j);
            robot.Goal(:,j)=g;
        end
    end
end


if status==0
    %if mod(t,3)~=0
    horizon=3;
    
    goal=[robot.Goal(1,1);0;robot.Goal(2,1);0];
%     if t>10
%         robot=inference(robot,10);
%     else
%         robot.inf.xstar=[10*ones(1,10);zeros(1,10);10*ones(1,10);zeros(1,10)];
%     end
    


    alfa=0.2;
    forget=0.98;
    if t==1
        robot.inf.xstar=[10;0;10;0];
        robot.inf.xself=[-10;0;-10;0];
    else
        fi=[robot.inf.xstar(:,t-1)' robot.obs.xself(:,t-1)' robot.obs.goal(:,t-1)']';
        C1=[robot.inf.A{t-1} robot.inf.B{t-1}];
        robot.inf.xstar(1:4,t)=(1-alfa).*robot.obs.xstar(:,t)+alfa.*C1*fi;
        robot.inf.F{t}=1/forget.*(robot.inf.F{t-1}-robot.inf.F{t-1}*fi*fi'*robot.inf.F{t-1}/(forget+fi'*robot.inf.F{t-1}*fi));
        C2=C1+(robot.inf.xstar(:,t)-C1*fi)*fi'*robot.inf.F{t};
        robot.inf.A{t}=C2(:,1:4);
        robot.inf.B{t}=C2(:,5:10);
    end
    
    %calculate horizon
    v=norm(robot.obs.xself(2:2:4,end));
    p=norm(robot.Goal(:,1)-robot.obs.xself(1:2:3,end));
    amax=robot.umax;
    tmin=2*sqrt(p/amax-0.5*(v/amax)^2)-v/amax;
    if tmin<0
        tmin=2*p/v;
    end
    horizon=max([1 floor(tmin/robot.delta_t)]);
    
    %calculate optimal control
    A=[];B=[];q=[];R=eye(2*horizon);Binc=[];G=[];
    for i=1:horizon
        A=[A;robot.A^i];
        Binc=[robot.A^(i-1)*robot.B Binc];
        B=[B zeros(4*(i-1),2);Binc];
        G=[G;goal];
        if i<horizon
        q=[q 1 0 1 0];
        else
            q=[q 100 100 100 100];
        end
    end
    
    Q=diag(q);
    
    %U=inv(R+B'*Q*B)*(B'*Q*G-B'*Q*A*robot.obs.xself(:,end));
    U=[-1 -2 0 0; 0 0 -1 -2]*(robot.obs.xself(:,end)-goal);
    D=(robot.A-robot.inf.B{t}(:,1:4))*robot.obs.xself(:,end)-robot.inf.A{t}*robot.obs.xstar(:,end)-robot.inf.B{t}(:,5:6)*robot.obs.goal(:,end);
    dnext=sqrt(D(1)^2+D(3)^2);
    thres=((robot.margin^2*dnext-dnext^3)-D(1)*D(2)-D(3)*D(4))/robot.delta_t;
     %thres=((robot.margin^2*dnext-dnext^3)/robot.delta_t-D(1)*D(2)-D(3)*D(4))/robot.delta_t;
    

    
%     
%     if d>robot.margin
% if mod(t,10)==1
%         cvx_begin
%         cvx quite
%         variable x(4,horizon)
%         variable u(2,horizon)
%         minimize norm(x(1:4,horizon)-goal,2)+thres-[D(1) D(3)]*u(:,1);
%         subject to
%         x(1:4,1)==robot.A*robot.obs.xself(1:4,end)+robot.B*u(:,1);
%         for i=2:horizon
%             x(1:4,i)==robot.A*x(1:4,i-1)+robot.B*u(:,i);
%         end
%         for i=1:horizon
%             norm(u(:,i),2)<=robot.umax;
%         end
%         %D(1:2:3)'*u(:,1)>=thres;
%         cvx_end
%         robot.ref.x(1:4,t+1:t+horizon)=x;
%         robot.ref.u(1:2,t:t+horizon-1)=u;
% end
    for i=1:1

        u=U((i-1)*2+1:i*2);%robot.ref.u(:,t);
        if norm(u)>robot.umax
            u=u*robot.umax/norm(u);
        end
        if (D(1)*u(1)+D(3)*u(2))<thres
            change=thres-(D(1)*u(1)+D(3)*u(2));
            u=u+D(1:2:3)*(change/norm(D(1:2:3))^2)+[D(3);-D(1)]/norm(D(1:2:3))^2;
        end
        robot.u(1:2,t)=u;

        robot.x(1:4,t+1)=robot.A*robot.x(:,t)+robot.B*robot.u(:,t);
    end
%     else
%     
%     status=2;
%     robot=inference(robot,10);
%     robot=solvelqr(robot,10);
%     
%     u=-robot.K{1}*robot.obs.xself(1:4,end)-robot.d{1};%inv(robot.R+robot.B'*robot.P*robot.B)*robot.B'*robot.P*goal;
%     u=[0 1;-1 0]*robot.obs.xstar(2:2:4,end)*(robot.margin/d)^2;
%     if norm(u)>robot.umax
%         u=u./norm(u).*robot.umax;
%     end
%     robot.x(1:4,t+1)=robot.A*robot.x(1:4,t)+robot.B*u;
%     %         if norm(robot.x(1:2:3,t+1)-robot.inf.xstar(1:2:3,1))<2
%     %             robot.x(1:4,t+1)=robot.x(1:4,t);
%     %             robot.x(1,t+1)=2*robot.x(1,t)-robot.obs.xstar(1,end);
%     %             robot.x(3,t+1)=2*robot.x(3,t)-robot.obs.xstar(3,end);
%     %         end
%     end
end


% if t>9 && mod(t,10)==0
%     A1=[];
%     A2=[];
%     for i=1:t-2
%         A1=[A1 robot.obs.xstar(:,i+1)];
%         A2=[A2 [robot.obs.xstar(:,i);robot.obs.goal(1,i+1);0;robot.obs.goal(2,i+1);0]];
%     end
%     [n,m]=size(A2);
%     if rank(A2)<8
%         A2=A2+eye(n,m)*0.0001;
%     end
% 
%     C=A1/A2;
%     
%     robot.inf.A=C(:,1:4);
%     robot.inf.B=[zeros(4) C(:,5:2:7)];
% end

%goal inference






%%%%%%%%%%%%%%%%%%
% EM
%%%%%%%%%%%%%%%%%%
% if mod(t,100)==0
% for i=1:50 % 50 is an arbitrary number of EM iterations, but is reasonable for this example
% 	[xfilt_me, xpred_me, Vfilt_me, loglik_me, xsmooth_me, Vsmooth_me, robot.inf.A,robot.inf.B,robot.inf.W] = kf_smooth_abcd(robot.obs.xstar, robot.inf.A, robot.inf.B, robot.C, zeros(4,1), [robot.obs.xself;robot.obs.goal], robot.inf.W, robot.outnoisestar*eye(4), [10,0,10,0], eye(4)); % YOURS to implement
%     margin=max(abs(eig(robot.inf.A)))-1;
%     if margin>=0
%         robot.inf.A=robot.inf.A-(margin+0.1)*eye(4);
%     end
% 	ll(i) = loglik_me;
% end
% if t==500
%     ssss=0;
% end
% end
robotnew=robot;
end