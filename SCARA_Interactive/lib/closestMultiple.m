% calculates the closest point on the arm under several obstacles
function [linkid,lstar,profile]=closestMultiple(l,pos,obs)
nlink=length(l);
nobs=size(obs,2);
rmin=100;r=zeros(nlink,nobs);S=zeros(nlink,nobs);rrl=zeros(nlink,nobs);
for k=1:nobs
for i=1:nlink
    r(i,k)=norm(pos(3*i+1:3*(i+1))-obs(1:3,k));
end
for i=1:nlink-1
    rrl(i+1,k)=abs(r(i,k)^2-r(i+1,k)^2)-l(i+1)^2;
    H=(r(i,k)+r(i+1,k)+l(i+1))/2;
    S(i+1,k)=sqrt(H*(H-r(i,k))*(H-r(i+1,k))*(H-l(i+1)));
    if rrl(i+1,k)>=0
        ro=min([r(i,k),r(i+1,k)]);
        if ro<=rmin
            rmin=ro;
            linkid=i+1;
            obsid=k;
            if r(i,k)==ro
                lstar=0;
            else
                lstar=1;
            end
            
        end
    else 
        ro=2*S(i+1,k)/l(i+1);
        if ro<=rmin
            rmin=ro;
            linkid=i+1;
            obsid=k;
            lstar=sqrt(r(i,k)^2-ro^2)/l(i+1);
        end
    end
end
end
lstar=lstar*l(linkid);
profile.r=r;
profile.S=S;
profile.rrl=rrl;
profile.rmin=rmin;
profile.obsid=obsid;
end