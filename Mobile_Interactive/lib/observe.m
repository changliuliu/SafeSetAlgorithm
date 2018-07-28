function [agent1,agent2]=observe(agent1,agent2,interact,t)
    agent1.obs.xself=[agent1.obs.xself agent1.x(:,end)+randn(4,1)*agent1.outnoiseself];
    agent2.obs.xself=[agent2.obs.xself agent2.x(:,end)+randn(4,1)*agent2.outnoiseself];
    if agent2.nG>0
    agent1.obs.goal=[agent1.obs.goal agent2.Goal(:,1)];
    else
        agent1.obs.goal=[agent1.obs.goal agent1.obs.goal(:,end)];
    end
    if agent1.nG>0
    agent2.obs.goal=[agent2.obs.goal agent1.Goal(:,1)];
    else
        agent2.obs.goal=[agent2.obs.goal agent2.obs.goal(:,end)];
    end
if interact(1)==1
    agent1.obs.xstar=[agent1.obs.xstar agent2.x(:,end)+randn(4,1)*agent1.outnoisestar];
end
if interact(2)==1
    agent2.obs.xstar=[agent2.obs.xstar agent1.x(:,end)+randn(4,1)*agent2.outnoisestar];
end