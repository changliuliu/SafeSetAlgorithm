%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The robot control algorithm
% by Changliu Liu
% 2015.5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [status,robotnew]=robotmove(t,robot)

status=0;

if status==0
    goal=[robot.Goal(1,1);robot.Goal(2,1)];
    robot.goalhis{t}=goal;
    %PAA
    %     alfa=0.2;
    %     forget=0.98;
    %     if t==1
    %         robot.inf.xstar=[10;0;10;0];
    %         robot.inf.xself=[-10;0;-10;0];
    %     else
    %         fi=[robot.inf.xstar(:,t-1)' robot.obs.xself(:,t-1)' robot.obs.goal(:,t-1)']';
    %         C1=[robot.inf.A{t-1} robot.inf.B{t-1}];
    %         robot.inf.xstar(1:4,t)=(1-alfa).*robot.obs.xstar(:,t)+alfa.*C1*fi;
    %         robot.inf.F{t}=1/forget.*(robot.inf.F{t-1}-robot.inf.F{t-1}*fi*fi'*robot.inf.F{t-1}/(forget+fi'*robot.inf.F{t-1}*fi));
    %         C2=C1+(robot.inf.xstar(:,t)-C1*fi)*fi'*robot.inf.F{t};
    %         robot.inf.A{t}=C2(:,1:4);
    %         robot.inf.B{t}=C2(:,5:10);
    %     end
    
    %calculate space trans
    
    [Je,He]=Jacobi(robot.x(1:robot.nlink,end),robot.x(robot.nlink+1:2*robot.nlink,end),robot.l,robot.nlink,robot.l(end),robot.base,robot.DH);
    [M,C]=rbt_dyna_matrix(robot.x(:,end),robot.param);
    
    %desired control
    
    %     if norm(robot.wx(1:2,end)-goal)>0.1
    %         U=-Je'*(robot.wx(1:2,end)-goal+2*robot.wx(4:5,end))*50-0.5*robot.x(robot.nlink+1:2*robot.nlink,end);
    %         robot.u(1:2,t)=U./max(abs(U)).*robot.umax;
    %         if robot.x(3,end)*robot.u(1,t)/norm(robot.u(1,t))>=robot.wmax
    %             robot.u(1,t)=0;
    %         end
    %         if robot.x(4,end)*robot.u(2,t)/norm(robot.u(2,t))>=robot.wmax
    %             robot.u(2,t)=0;
    %         end
    %     else
    U=-Je'*(robot.wx(1:2,end)-goal+1*robot.wx(4:5,end))*100-robot.x(robot.nlink+1:2*robot.nlink,end);
    robot.u(1:2,t)=U;
    tic
    %calculate closest point
    [linkid,ratio,robot.profile{t}]=closest_2D(robot.obs.xstar(1:2,t),robot.x(:,t),robot.pos(:,t),robot.l);
    robot.profile{t}.dtime=toc;
    tic;
    robot.profile{t}.ssa=0;
    if linkid==1 && ratio==0
        if robot.profile{t}.rmin<0.1
            ratio=0.001;
        end
    end
    
    if linkid==1 && ratio==0
        robot.u(1:2,t)=U;
    else
        [Jm,Hm,robot.mx(1:3,t),robot.mx(4:6,t)]=Jacobi(robot.x(1:robot.nlink,end),robot.x(robot.nlink+1:2*robot.nlink,end),robot.l,linkid,ratio,robot.base,robot.DH);
        
        % Discrete safety index
        BJ=robot.B*Jm*inv(M);
        %D=(robot.A-robot.inf.B{t}(:,1:4))*robot.wx([1,2,4,5],end)-robot.inf.A{t}*robot.obs.xstar(:,end)-robot.inf.B{t}(:,5:6)*robot.obs.goal(:,end)+robot.B*Hm;
        
        D=robot.A*robot.mx([1,2,4,5],end)+robot.B*(Hm-Jm*inv(M)*C)-robot.obs.xstar(:,end);
        
        [thres,vet]=safety(D,BJ,robot.margin);
        
        
        
        if (vet*U)<thres
            change=thres-vet*U;
            U=U+M*vet'*inv(vet*M*vet')*change;
            robot.profile{t}.ssa=1;
        end
        
        dx=robot.mx([1,2,4,5],end)-robot.obs.xstar(:,end);
        dmin=robot.profile{t}.rmin;
        kd=0.01;
        % Continuous safety index
        if -dmin^2-kd*dx'*robot.const.P2*dx/dmin>=0
            vet=dx(1:2)'*Jm*inv(M)./dmin;
            vcirc=dx(3:4)-dx(1:2)*(dx(3:4)'*dx(1:2)/dmin);
            thres=robot.margin-2*dx'*robot.const.P2*dx+kd*(dx(1:2)'*(Jm*inv(M)*C-Hm)-norm(vcirc))/dmin;
            if (vet*U)<thres
                change=thres-vet*U;
                U=U+M*vet'*inv(vet*M*vet')*change;
                robot.profile{t}.ssa=1;
            end
        end
    end
    
    robot.profile{t}.ssatime=toc;
    
    if abs(U(1))>robot.umax(1)
        U(1)=U(1)/abs(U(1))*robot.umax(1);
    end
    if abs(U(2))>robot.umax(2)
        U(2)=U(2)/abs(U(2))*robot.umax(2);
    end
    
    if robot.x(3,end)*robot.u(1,t)/norm(robot.u(1,t))>=robot.wmax
        robot.u(1,t)=0;
    end
    if robot.x(4,end)*robot.u(2,t)/norm(robot.u(2,t))>=robot.wmax
        robot.u(2,t)=0;
    end
    robot.u(1:2,t)=U;
    
    
    
end

[~,Q]=ode45(@(t,x)rbt_fwd_dyna_frc(t,x,robot.param,robot.u(:,end)), [t*robot.delta_t,(t+1)*robot.delta_t],robot.x(:,end));

robot.x(:,t+1)=Q(end,:);
robot.pos(:,t+1)=ArmPos(robot.base,robot.DH,robot.x(1:robot.nlink,t+1));%(x1,y1,z1,...,x(N+1),y(N+1),z(N+1))
robot.wx(1:3,t+1)=robot.pos(end-2:end,t+1);
robot.wx(4:5,t+1)=Je*robot.x(robot.nlink+1:2*robot.nlink,t+1);

robotnew=robot;
end