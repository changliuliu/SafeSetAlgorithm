%% initialize the animation plot
fighandle = figure(1);
clf;
set(gcf,'Position',get(0,'ScreenSize'),'color','w')
set(gcf,'renderer','opengl') 
axis equal
axis([-0.5 0.5 -0.5 0.5]);
%grid on;
box on;
hold on;
xlabel('m');
ylabel('m');
text1handle = text(min(xlim),max(ylim)+0.1,' ');

robot.handle = plot(robot.pos(1:3:(robot.nlink*3+1),end),robot.pos(2:3:(robot.nlink*3+3),end),'linewidth',10,'color','r','markersize',50);
set(robot.handle,'XDataSource','robot.pos(1:3:(robot.nlink*3+1),end)');
set(robot.handle,'YDataSource','robot.pos(2:3:(robot.nlink*3+2),end)');

agent.handle = plot(agent.x(1,end),agent.x(2,end),'.','linewidth',3,'color',[0.5,0.5,1],'markersize',200);
set(agent.handle,'XDataSource','agent.x(1,end)');
set(agent.handle,'YDataSource','agent.x(2,end)');
%%
rgoal.handle = plot(robot.Goal(1,:),robot.Goal(2,:),'.','linewidth',1,'color','r','markersize',50);
set(rgoal.handle,'XDataSource','robot.Goal(1,:)');
set(rgoal.handle,'YDataSource','robot.Goal(2,:)');
%%
agoal.handle = plot(agent.Goal(1,1),agent.Goal(2,1),'.','linewidth',1,'color','b','markersize',50);
set(agoal.handle,'XDataSource','agent.Goal(1,1)');
set(agoal.handle,'YDataSource','agent.Goal(2,1)');

rtrace.handle = plot(robot.wx(1,max([1,size(robot.x,2)-100]):end),robot.wx(2,max([1,size(robot.x,2)-100]):end),'linewidth',1,'color','r');
set(rtrace.handle,'XDataSource','robot.wx(1,max([1,size(robot.x,2)-100]):end)');
set(rtrace.handle,'YDataSource','robot.wx(2,max([1,size(robot.x,2)-100]):end)');

atrace.handle = plot(agent.x(1,max([1,size(agent.x,2)-100]):end),agent.x(2,max([1,size(agent.x,2)-100]):end),'linewidth',1,'color','b');
set(atrace.handle,'XDataSource','agent.x(1,max([1,size(agent.x,2)-100]):end)');
set(atrace.handle,'YDataSource','agent.x(2,max([1,size(agent.x,2)-100]):end)');
