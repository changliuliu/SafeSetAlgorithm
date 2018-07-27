function status=CollisionCheck(robot,obstacle,range)
status=0;
if size(robot,2)>1
    for i=1:size(robot,2)
        [~,~,profile]=closestMultiple(robot{i}.l,robot{i}.pos(:,end),obstacle);
        if profile.rmin<range
            status=1;
        end
    end
else
    [~,~,profile]=closestMultiple(robot.l,robot.pos(:,end),obstacle);
    if profile.rmin<range
        status=1;
    end
end
end