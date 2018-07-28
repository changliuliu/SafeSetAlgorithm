%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Main Code in two agent game    %
%  Changliu Liu                   %
%  2013.11                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
addpath('lib')

flag=0;%sign that the game is down
flag1=0;%sign that robot has finished all the task
flag2=0;%sign that human has finished all the task

interact=[1 1];%interaction type

use_cursor=1;%use defined agent or cursor

horizon=10;

robot=robotproperty(interact(1));
agent=agentproperty(interact(2));

%robot=generate_reference_traj(robot);

t=1;%count time step

fighandle = figure;%simulation plane
set(gcf,'Position',get(0,'ScreenSize'),'color','w')

%% initialize the animation plot
subplot1handle = subplot(1,2,1);
hold on;
axis equal
axis([-10 10 -10 10]);
grid on;
text1handle = text(min(xlim),max(ylim)+0.1,' ');

robot.handle = plot(robot.x(1,end),robot.x(3,end),'o','linewidth',3,'color','r','markersize',50);
set(robot.handle,'XDataSource','robot.x(1,end)');
set(robot.handle,'YDataSource','robot.x(3,end)');

agent.handle = plot(agent.x(1,end),agent.x(3,end),'o','linewidth',3,'color','b','markersize',50);
set(agent.handle,'XDataSource','agent.x(1,end)');
set(agent.handle,'YDataSource','agent.x(3,end)');

rgoal.handle = plot(robot.Goal(1,:),robot.Goal(2,:),'o','linewidth',1,'color','r','markersize',50);
set(rgoal.handle,'XDataSource','robot.Goal(1,:)');
set(rgoal.handle,'YDataSource','robot.Goal(2,:)');

agoal.handle = plot(agent.Goal(1,:),agent.Goal(2,:),'o','linewidth',1,'color','b','markersize',50);
set(agoal.handle,'XDataSource','agent.Goal(1,:)');
set(agoal.handle,'YDataSource','agent.Goal(2,:)');

rtrace.handle = plot(robot.x(1,max([1,size(robot.x,2)-100]):end),robot.x(3,max([1,size(robot.x,2)-100]):end),'linewidth',1,'color','r');
set(rtrace.handle,'XDataSource','robot.x(1,max([1,size(robot.x,2)-100]):end)');
set(rtrace.handle,'YDataSource','robot.x(3,max([1,size(robot.x,2)-100]):end)');

atrace.handle = plot(agent.x(1,:),agent.x(3,:),'linewidth',1,'color','b');
set(atrace.handle,'XDataSource','agent.x(1,max([1,size(agent.x,2)-100]):end)');
set(atrace.handle,'YDataSource','agent.x(3,max([1,size(agent.x,2)-100]):end)');

%% initialize the control plot
subplot2handle = subplot(1,2,2);
hold on;
grid on;
axis equal
axis([-10 10 -10 10]);
plot([-1 1],[0 0],':b','linewidth',4)
plot([0 0],[-1 1],':b','linewidth',4)
text2handle = text(-1,1.2,' ');

calibmark.xy = [10; 10];
calibmark.handle = plot(calibmark.xy(1),calibmark.xy(2),'o','linewidth',3,'color','r','markersize',14);
set(calibmark.handle,'XDataSource','calibmark.xy(1)');
set(calibmark.handle,'YDataSource','calibmark.xy(2)');

%% calibrate the control before the test
set(fighandle, 'currentaxes', subplot2handle)
set(text2handle,'string','Please calibrate cursor')

calibmark.xy = [0; 0];
refreshdata([calibmark.handle],'caller');
drawnow;
pause(3)
cursor_pos_center = get(0,'PointerLocation');

calibmark.xy = [10; 10];
refreshdata([calibmark.handle],'caller');
drawnow;
pause(3)
cursor_pos_URcorner = get(0,'PointerLocation');

set(text2handle,'string',' ')

%% begin testing
set(fighandle, 'currentaxes', subplot2handle)
set(text1handle,'string','Test runing...')

J = eye(2); % [-1 2; 10 5];

while flag==0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Check if #goal < 3         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if robot.nG<3
        robot.Goal(:,3:5)=(rand(2,3)-0.5)*20;
        robot.nG=5;
    end
    
    if agent.nG<1
        agent.Goal(:,1)=(rand(2,1)-0.5)*20;
        agent.nG=1;
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  robot observe agent        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [robot,agent]=observe(robot,agent,interact);
    %goal inference
    mini=100;ii=1;
    for i=1:agent.nG
        goal=[agent.Goal(1,i);0;agent.Goal(2,i);0];
        dx=robot.obs.xstar(:,end)-goal;
        score=-dx'*robot.const.Q1*dx/(dx'*robot.const.Q2*dx);
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
            robotposition=robot.x(1:2:3,ie);
        end
        if flag1==2
            flag1=0;

        end
    end
    

    if flag1==1
        %plot(robot.x(1,:),robot.x(3,:));
        %drawagent(robot.x(1,end),robot.x(3,end),0.5,'k');
        robotposition=robot.x(1:2:3,end);
    end

%     if flag2==0
%         [flag2,agent]=agentmove(t,agent);
%         if flag2==0
%         %plot(agent.x(1,1:t+1),agent.x(3,1:t+1));
%         %drawagent(agent.x(1,t+1),agent.x(3,t+1),0.5,'r');
%         agentposition=agent.x(1:2:3,t+1);
%         end
%     end
    
%     if flag2==1
%         %plot(agent.x(1,:),agent.x(3,:));
%         %drawagent(agent.x(1,end),agent.x(3,end),0.5,'r');
%         agentposition=agent.x(1:2:3,end);
%     end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% update the agent position   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cursor_pos_current = get(0,'PointerLocation');
    u = (cursor_pos_current - cursor_pos_center)./(cursor_pos_URcorner - cursor_pos_center)*J; % normalized
    agent.x(1,end+1) = u(1)*10;
    agent.x(3,end) = u(2)*10;
    if size(agent.x,2)>1
        agent.x(2,end)=agent.x(1,end)-agent.x(1,end-1);
        agent.x(4,end)=agent.x(3,end)-agent.x(3,end-1);
    end
    agentposition=agent.x(1:2:3,end);
    %check if a goal is reached
    for i=1:agent.nG
        if norm(agentposition-agent.Goal(:,i))<0.2
            agent.Goal=deleteGoal(agent.Goal,i);
            agent.nG=agent.nG-1;
            agent.score=agent.score+1;
            break
        end
    end
    
    if agent.nG==0
        %flag2=1;
        agent.Goal(:,1)=(rand(2,1)-0.5)*20;
        agent.nG=1;
    end
        
    if flag1==1 && flag2==1
        flag=1;
        break
    end
    t=t+1;
    
    refreshdata([robot.handle],'caller');
    refreshdata([agent.handle],'caller');
    refreshdata([rtrace.handle],'caller');
    refreshdata([atrace.handle],'caller');
    
    if flag1==0
        refreshdata([rgoal.handle],'caller');
    end
    if flag2==0
        refreshdata([agoal.handle],'caller');
    end
    output=strcat('timestep:',int2str(t),'  score:',int2str(robot.score+agent.score));
    set(text1handle,'string',output)
    drawnow;
    if norm(agentposition-robotposition)<2
        stat='collide!'
        break
    end
    
    pause(0.1);
end

set(text1handle,'string','Test ended')
