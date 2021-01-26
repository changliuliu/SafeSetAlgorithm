function new=PAA(old,Action,x,t)
new=old;
f=GetFeature(x,t-1);
if Action==1
    data=[f(4);f(5)];
else
    data=[f(4);f(5);f(8);f(10)];
end
fnew=GetFeature(x,t);
Gain=eye(size(data,2))./t;
new.B{Action}.par(:,end+1)=[old.B{Action}.par(1:end-1,end)+...
    Gain*data*(fnew(1)-old.B{Action}.par(1:end-1,end)'*data);t];
end