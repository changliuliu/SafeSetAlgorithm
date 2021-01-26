function theta=angle(p1,p2)
if p1(1)-p2(1)==0
    if p1(2)>p2(2)
        theta=pi/2;
    else
        theta=3*pi/2;
    end
elseif p1(1)-p2(1)>0
    theta=atan((p1(2)-p2(2))/(p1(1)-p2(1)));
else
    theta=atan((p1(2)-p2(2))/(p1(1)-p2(1)))+pi;
end