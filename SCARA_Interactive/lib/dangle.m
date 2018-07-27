function relangle=dangle(rx,ry)
if rx==0
    if ry>0
        relangle=pi/2;
    else
        relangle=-pi/2;
    end
else
    if rx>0
        relangle=atan(ry/rx);
    else
        relangle=atan(ry/rx)+pi;
    end
end
end