function pos=ArmPos(base,DH,x)
nlink=length(x);
pos=[];
pos(1:3,1)=base;
M={}; M{1}=eye(4);
for i=1:nlink
    R=[cos(x(i)) -sin(x(i))*cos(DH(i,4)) sin(x(i))*sin(DH(i,4));
        sin(x(i)) cos(x(i))*cos(DH(i,4)) -cos(x(i))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(x(i));DH(i,3)*sin(x(i));DH(i,2)];
    M{i+1}=M{i}*[R T; zeros(1,3) 1];
    pos(1+3*i:3+3*i,1)=M{i+1}(1:3,4)+base;
end
end