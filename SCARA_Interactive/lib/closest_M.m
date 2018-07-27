function [linkid,lstar,profile]=closest_M(l,pos,obs)
nlink=length(l);lold=l;
for i=1:nlink+1
    for j=1:3
    pos(3*(i-1)+j)=(pos(3*(i-1)+j)-obs(j));%/obs(j+3);
    end
end
for i=1:nlink
    l(i)=norm(pos(3*(i-1)+1:3*i)-pos(3*i+1:3*i+3));
end
rmin=100;r=zeros(nlink,1);S=zeros(nlink,1);rrl=zeros(nlink,1);
for i=1:nlink
    r(i)=norm(pos(3*i+1:3*(i+1)));
end
for i=1:nlink-1
    rrl(i+1)=abs(r(i)^2-r(i+1)^2)-l(i+1)^2;
    H=(r(i)+r(i+1)+l(i+1))/2;
    S(i+1)=sqrt(H*(H-r(i))*(H-r(i+1))*(H-l(i+1)));
    if rrl(i+1)>=0
        ro=min([r(i),r(i+1)]);
        if ro<=rmin
            rmin=ro;
            linkid=i+1;
            if r(i)==ro
                lstar=0;
            else
                lstar=1;
            end
            
        end
    else 
        ro=2*S(i+1)/l(i+1);
        if ro<=rmin
            rmin=ro;
            linkid=i+1;
            lstar=sqrt(r(i)^2-ro^2)/l(i+1);
        end
    end
end
lstar=lstar*lold(linkid);
profile.r=r;
profile.S=S;
profile.rrl=rrl;
profile.rmin=rmin;
profile.linkid=linkid;
profile.lstar=lstar;
end