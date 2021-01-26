function u=saturation_u(uold,sat1,sat2)
u=uold;
if u(1)>sat1(2)
    u(1)=sat1(2);
end

if u(1)<sat1(1)
    u(1)=sat1(1);
end
if u(2)>sat2(2)
    u(2)=sat2(2);
end

if u(2)<sat2(1)
    u(2)=sat2(1);
end
end