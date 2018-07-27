%t:theta;dt:dtheta;
%the Jacobi is for the point on the k-th link and with lenght lk

function [J,H,vec,dvec]=Jacobi(t,dt,l,k,lk,base,DH)
n=length(t);
DH(k,3)=lk;
l(k)=lk;
for i=k+1:length(l)
    l(i)=0;
end
vec=0;dvec=[];
T={};
for i=1:k
    if i>1
        T{i}=T{i-1}*[cos(t(i)) -sin(t(i))*cos(DH(i,4)) sin(t(i))*sin(DH(i,4)) DH(i,3)*cos(t(i));
            sin(t(i)) cos(t(i))*cos(DH(i,4)) -cos(t(i))*sin(DH(i,4)) DH(i,3)*sin(t(i));
            0 sin(DH(i,4)) cos(DH(i,4)) DH(i,2);
            0 0 0 1];
    else
        T{i}=[cos(t(i)) -sin(t(i))*cos(DH(i,4)) sin(t(i))*sin(DH(i,4)) DH(i,3)*cos(t(i));
            sin(t(i)) cos(t(i))*cos(DH(i,4)) -cos(t(i))*sin(DH(i,4)) DH(i,3)*sin(t(i));
            0 sin(DH(i,4)) cos(DH(i,4)) DH(i,2);
            0 0 0 1];
    end
end
vec=base+T{k}(1:3,4);

switch n
    case 2
        H=[-l(1)*cos(t(1))*dt(1)^2-l(2)*cos(t(1)+t(2))*(dt(1)+dt(2))^2;l(1)*sin(t(1))*dt(1)^2+l(2)*sin(t(1)+t(2))*(dt(1)+dt(2))^2];
        J=[-l(1)*sin(t(1))-l(2)*sin(t(1)+t(2)),-l(2)*sin(t(1)+t(2));l(1)*cos(t(1))+l(2)*cos(t(1)+t(2)),l(2)*cos(t(1)+t(2))];
        dvec=[J*dt;0];
    case 3
        
        
        x=vec(1);y=vec(2);z=vec(3);
        
        J=[-y -(z-l(1))*cos(t(1)) -l(3)*cos(t(1))*sin(t(2)+t(3));
            x -(z-l(1))*sin(t(1)) -l(3)*sin(t(1))*sin(t(2)+t(3));
            0 l(2)*cos(t(2))+l(3)*cos(t(2)+t(3)) l(3)*cos(t(2)+t(3))];
        
        dvec=J*dt;
        dx=dvec(1);dy=dvec(2);dz=dvec(3);
        
        H=[-dy*dt(1)+(z-l(1))*sin(t(1))*dt(1)*dt(2)-dz*cos(t(1))*dt(2)+l(3)*sin(t(1))*sin(t(2)+t(3))*dt(1)*dt(3)-l(3)*cos(t(1))*cos(t(2)+t(3))*(dt(2)+dt(3))*dt(3);
            dx*dt(1)-(z-l(1))*cos(t(1))*dt(1)*dt(2)-dz*sin(t(1))*dt(2)-l(3)*cos(t(1))*sin(t(2)+t(3))*dt(1)*dt(3)-l(3)*sin(t(1))*cos(t(2)+t(3))*(dt(2)+dt(3))*dt(3);
            -l(2)*sin(t(2))*dt(2)^2-l(3)*sin(t(2)+t(3))*(dt(2)+dt(3))^2];
        
end
end