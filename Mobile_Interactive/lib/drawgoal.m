function status=drawgoal(robot,agent)
for i=1:robot.nG
    drawagent(robot.Goal{i}(1),robot.Goal{i}(2),0.5,'k');
end
for i=1:agent.nG
    drawagent(agent.Goal{i}(1),agent.Goal{i}(2),0.5,'r');
end
status=1;
end
