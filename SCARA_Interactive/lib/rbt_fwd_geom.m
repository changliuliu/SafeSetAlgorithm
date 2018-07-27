% pos: nx2
function pos = rbt_fwd_geom(q1, q2, l1, l2)

%  compute the end effector in task space
x = l1.*cos(q1)+l2.*cos(q1+q2);
y = l1.*sin(q1)+l2.*sin(q1+q2);

pos = [x; y];
