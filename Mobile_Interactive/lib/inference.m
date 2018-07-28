function robot=inference(robot,horizon)
robot.inf.xstar(1:4,1)=robot.inf.A*robot.obs.xstar(1:4,end)+robot.inf.B*[zeros(4,1);robot.obs.goal(:,end)];
for i=2:horizon
robot.inf.xstar(1:4,i)=robot.inf.A*robot.inf.xstar(1:4,i-1)+robot.inf.B*[zeros(4,1);robot.obs.goal(:,end)];
end