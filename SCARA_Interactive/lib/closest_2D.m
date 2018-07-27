
function [k,r,profile]=closest_2D(obs,rangle,robotpos,link)
n=length(link);
theta=0;
record=1e10;
k=2;r=link(2);
for i=1:n
    x=robotpos(i*3-2);
    y=robotpos(i*3-1);
    theta=mod(theta+rangle(i),2*pi);
    dmin=abs((x-obs(1))*sin(theta)-(y-obs(2))*cos(theta));
    if dmin<record
        rx=obs(1)-x;
        ry=obs(2)-y;
        relangle=abs(mod(dangle(rx,ry)-theta+pi,2*pi)-pi);
        if relangle<=atan(dmin/link(i))
            dmin=sqrt((obs(1)-robotpos(i*3+1))^2+(obs(2)-robotpos(i*3+2))^2);
            rr=1;
        else
            if relangle>=pi/2
                dmin=sqrt((obs(1)-x)^2+(obs(2)-y)^2);
                rr=0;
            else
                rr=dmin/tan(relangle)/link(i);
            end
            
        end
        if dmin<record
            record=dmin;
            k=i;
            r=rr;
        end
    end
end
r=r*link(k);
profile.rmin=record;
profile.linkid=k;
profile.lstar=r;
end