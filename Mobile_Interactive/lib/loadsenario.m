function [robotGoal,agentGoal]=loadsenario(senario)
switch senario
    case 1
        %goal 
        agentGoal=[0.5,0.5]';robotGoal=[5,5]';
    case 2
        agentGoal=[4,6]';robotGoal=[6,4]';
    otherwise
        agentGoal=[5,5]';robotGoal=[5,5]';    
end
end