circle.center=[1;1];
circle.radius=1;
link=[2,2];
base=[0;0];
figure(1); clf
subplot(121)
hold on
for r=circle.radius:0.1:circle.radius
for theta=0:0.1:2*pi
    t=circle.center+r*[cos(theta);sin(theta)];
    p=zeros(2,3);
    p(:,1)=base;
    p(:,2)=base+link(1)*[cos(t(1));sin(t(1))];
    p(:,3)=p(:,2)+link(2)*[cos(t(2));sin(t(2))];
    plot(p(1,:),p(2,:))
end
end
for t1=-pi:0.2:pi+0.3
    for t2=-pi:0.2:pi+0.3
        t=[t1;t2];
        %if norm([1 -1]*t)>2
        if norm(mod(t-circle.center+[pi;pi],2*pi)-[pi;pi])>circle.radius
            p=zeros(2,3);
            p(:,1)=base;
            p(:,2)=base+link(1)*[cos(t(1));sin(t(1))];
            p(:,3)=p(:,2)+link(2)*[cos(t(2));sin(t(2))];
            plot(p(1,:),p(2,:))
        end
    end
end

plot(sum(link)*cos(0:0.1:2*pi+0.1),sum(link)*sin(0:0.1:2*pi+0.1))
axis equal
axis([-sum(link) sum(link) -sum(link) sum(link)])
title('Workspace for Planar Robot Arm')
%%
subplot(122)
hold on
plot(circle.radius*cos(0:0.1:2*pi+0.1)+circle.center(1),circle.center(2)+circle.radius*sin(0:0.1:2*pi+0.1))
fill(circle.radius*cos(0:0.1:2*pi+0.1)+circle.center(1),circle.center(2)+circle.radius*sin(0:0.1:2*pi+0.1),'r')
grid on
box on
axis equal
axis([-pi pi -pi pi])
title('Configuration Space for Planar Robot Arm')

