%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test on the freeway scenario (Purely automatic)
% Denso: Road Project
% Feature: Multiple-Vehicles, Learning and Decision Making
%
% Changliu Liu
% 2016.2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Test Multiple Vehicle
addpath('Library');
robot=robotproperty(1,[-50;0.5;30;0]);
t=1;
nlane=4;

longit=[-50 250];
fighandle=initialize_figure_interact(1,longit,[0,nlane],[5,1,1]);
%[Center,URCorner]=calibration([1,1]);

%% Specify surrounding vehicles
set(fighandle(1), 'currentaxes', fighandle(2))
nagent=5;
agent={};
agent{1}.x=[-50; 1.5; 28;  0];
agent{2}.x(:,1)=[0; 1.5; 31; 0];
agent{3}.x=[-10;0.5;30;0];
agent{4}.x=[-90; 1.5; 28;  0];
agent{5}.x=[-80; 0.5; 28;  0];
dl=[-1 1];

agenthandle=zeros(nagent,1);

robothandle=plot(robot.x(1,t)+dl.*cos(robot.x(4,t)),robot.x(2,t)+dl.*sin(robot.x(4,t)),'r','LineWidth',10);
set(robothandle,'XDataSource','robot.x(1,t)+dl.*cos(robot.x(4,t))');
set(robothandle,'YDataSource','robot.x(2,t)+dl.*sin(robot.x(4,t))');
for i=1:nagent
    agenthandle(i)=plot(agent{i}.x(1,t)+dl.*cos(agent{i}.x(4,t)),agent{i}.x(2,t)+dl.*sin(agent{i}.x(4,t)),'b','LineWidth',10);
    set(agenthandle(i),'XDataSource',strcat('agent{',num2str(i),'}.x(1,t)+dl.*cos(agent{',num2str(i),'}.x(4,t))'));
    set(agenthandle(i),'YDataSource',strcat('agent{',num2str(i),'}.x(2,t)+dl.*sin(agent{',num2str(i),'}.x(4,t))'));
end


%%

robot.goal=[300;2.5;30;0];
radius=0.5;
ssa=[];

for t=1:500

    u=[0;0];
    
    for i=1:nagent
        agent{i}.x(:,t+1)=vehicle_dynamic_update(agent{i}.x(:,t),[0;0],robot.delta_t);
    end
    
    robot=robotmove(t,robot,agent);
    refreshdata(robothandle,'caller')
    for i=1:nagent
        refreshdata(agenthandle(i),'caller')
    end
    pause(0.05)
    
    if robot.x(1,end)>250
        break
    end
    
end
%% Plots
n=t;
start=longit(1);
ed=longit(2);
figure(1);clf;
subplot(511);hold on
for i=1:2
plot(agent{i}.x(1,:),agent{i}.x(2,:),'.','markersize',30);
end
plot(robot.x(1,1:n),robot.x(2,1:n),'LineWidth',3)
axis([start,ed,0,nlane]);
plot(start+1:ed,ones(ed-start,1),'--k')
title('Vehicle Trajectory')


subplot(512);hold on
bar(robot.x(1,1:n),robot.ssa(1,1:n),'FaceColor','g','EdgeColor','None')
axis([start,ed,0,1]);
title('Safety Controller Activity (On/Off)')

subplot(513);hold on
plot(robot.x(1,1:n),robot.u(1,1:n),'LineWidth',3)
title('Acceleration of the Vehicle')
axis([start,ed,-3,2]);
ylabel('m/s^2')

subplot(514)
plot(robot.x(1,1:n),robot.u(2,1:n),'LineWidth',3)
title('Turning Rate')
axis([start,ed,-0.06,0.06]);
ylabel('rad/s^2')

subplot(515)
plot(robot.x(1,1:n),robot.x(3,1:n),'LineWidth',3)
title('Vehicle Velocity')
axis([start,ed,20,31]);
ylabel('m/s')