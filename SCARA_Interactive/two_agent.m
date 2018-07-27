%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Main Code in SCARA dynamics    %
%  Changliu Liu                   %
%  2015.5                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
addpath('lib');
%% Initialization
robot=robotproperty(1);
agent=agentproperty(1);

flag=0;%sign that the game is down
flag1=0;%sign that robot has finished all the task
flag2=0;%sign that human has finished all the task

use_cursor=1;%use defined agent or cursor

t=1;%count time step

%% initialize the control plot
generate_animation_plot;
calibmark.xy = [0.5; 0.5];
calibmark.handle = plot(calibmark.xy(1),calibmark.xy(2),'o','linewidth',3,'color','b','markersize',14);
set(calibmark.handle,'XDataSource','calibmark.xy(1)');
set(calibmark.handle,'YDataSource','calibmark.xy(2)');

%% calibrate the control before the test

calibmark.xy = [0; 0];
refreshdata([calibmark.handle],'caller');
drawnow;
pause(3)
cursor_pos_center = get(0,'PointerLocation');

calibmark.xy = [0.5,0.5];
refreshdata([calibmark.handle],'caller');
drawnow;
pause(3)
cursor_pos_URcorner = get(0,'PointerLocation');

calibmark.xy = [10; 10];
refreshdata([calibmark.handle],'caller');
drawnow;

%% begin testing
set(text1handle,'string','Test runing...')

J = eye(2); % [-1 2; 10 5];
goalhis=[];

while flag==0

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  robot observe agent        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [robot,agent]=observe(robot,agent,[1,1]);
    %goal inference
    mini=100;ii=1;
    for i=1:agent.nG
        goal=[agent.Goal(1,i);0;agent.Goal(2,i);0];
        dx=robot.obs.xstar(:,end)-goal;
        score=-dx'*robot.const.P1*dx/(dx'*robot.const.P2*dx);
        if score>0 && score<mini
            mini=score;
            ii=i;
        end
    end
    robot.obs.goal(1:2,t)=agent.Goal(:,ii);
            
            
    
%%%%%%%%%%%%%%%%%
%  robot move   %
%%%%%%%%%%%%%%%%%
    if flag1==0
        %robot=solvelqr(robot);
        [flag1,robot]=robotmove(t,robot);
        if flag1~=1
            ie=min([size(robot.x,2),t+1]);
            %plot(robot.x(1,1:ie),robot.x(3,1:ie));
            %drawagent(robot.x(1,ie),robot.x(3,ie),0.5,'k');
            robotposition=robot.wx(1:2:3,ie);
        end
        if flag1==2
            flag1=0;

        end
    end
    

    if flag1==1
        %plot(robot.x(1,:),robot.x(3,:));
        %drawagent(robot.x(1,end),robot.x(3,end),0.5,'k');
        robotposition=robot.wx(1:2:3,end);
    end

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% update the agent position   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cursor_pos_current = get(0,'PointerLocation');
    u = (cursor_pos_current - cursor_pos_center)./(cursor_pos_URcorner - cursor_pos_center); % normalized
    agent.x(1,end+1) = u(1)/2;
    agent.x(2,end) = u(2)/2;
    if size(agent.x,2)>1
        agent.x(3,end)=(agent.x(1,end)-agent.x(1,end-1))/robot.delta_t;
        agent.x(4,end)=(agent.x(2,end)-agent.x(2,end-1))/robot.delta_t;
    end
    
    refreshdata([robot.handle, agent.handle, rtrace.handle, atrace.handle],'caller');
    
    t=t+1;
    if t>2
        delete(chandle);
    end
    chandle=capsule(robot.nlink,robot.pos(:,t),robot.x(:,t),[0.07,0.06],'r',0.1,agent.x(:,t),'b');


    if flag1==0 && flag2==0
        refreshdata([rgoal.handle, agoal.handle],'caller');
    end
    
    output=strcat('timestep:',int2str(t),'  score:',int2str(robot.score+agent.score));
    set(text1handle,'string',output)
    drawnow;
    
    if norm(robot.wx(1:2,end)-robot.Goal(:,1))<0.02
        robot.Goal=randn(2,1)./5;
        if norm(robot.Goal-robot.base(1:2))<norm(robot.l(1)-robot.l(2))
            robot.Goal=(robot.Goal-robot.base(1:2))/norm(robot.Goal-robot.base(1:2))*norm(robot.l(1)-robot.l(2))+robot.base(1:2);
        end
        if norm(robot.Goal-robot.base(1:2))>norm(robot.l(1)+robot.l(2))
            robot.Goal=(robot.Goal-robot.base(1:2))/norm(robot.Goal-robot.base(1:2))*norm(robot.l(1)+robot.l(2))+robot.base(1:2);
        end
    end
    
    agent.goalhis{t}=agent.Goal;
    if norm(agent.x(1:2,end)-agent.Goal(:,1))<0.01
        agent.Goal=randn(2,1)./5;
        if norm(agent.Goal-robot.base(1:2))<0.25
            agent.Goal=robot.base(1:2)+(agent.Goal-robot.base(1:2))*0.25/norm(agent.Goal-robot.base(1:2));
        end
        if norm(agent.Goal(1))>0.4
            agent.Goal(1)=0.4*agent.Goal(1)/norm(agent.Goal(1));
        end
        if norm(agent.Goal(2))>0.4
            agent.Goal(2)=0.4*agent.Goal(2)/norm(agent.Goal(2));
        end
    end
    
    

    pause(0.05); 

end

set(text1handle,'string','Test ended')
