% Compute the inverse kinematic of two-link planar robot.
% [x, y] are the end-effector in task space
% [q1, q2] are the joint space position.

function [qd_mb] = rbt_inv_geom(len1, len2, xd, yd, flag)
l1 = len1; % 0.32;   
l2 = len2; % 0.215;

% make sure x, y have the same size
if length(xd)~=length(yd),
  error('length(xd)~=length(yd)');
end;
% make sure x, y are column vector.
[m,n]=size(xd);
if n == 1, xd=xd'; end
[m,n]=size(yd);
if n == 1, yd=yd'; end

% compute the ik.
n = length(xd);
dd = xd.^2 + yd.^2;

c2 = (xd.^2 + yd.^2 - l1.^2 - l2.^2)/(2*l1*l2);
s2 = sign(flag)*sqrt(1 - c2.^2);
qd2 = atan2(s2, c2);

s1 = ((l1 + l2.*c2).*yd - l2.*s2.*xd)./dd;
c1 = ((l1 + l2.*c2).*xd + l2.*s2.*yd)./dd;
qd1 = atan2(s1, c1);

qd_mb = [qd1; qd2];