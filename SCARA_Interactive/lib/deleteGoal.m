function Goal=deleteGoal(G,index)
Goal=[];
for i=1:index-1
    Goal(1:2,i)=G(:,i);
end
for i=index+1:size(G,2)
    Goal(1:2,i-1)=G(:,i);
end