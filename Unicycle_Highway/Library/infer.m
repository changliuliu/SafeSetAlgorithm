function new=infer(old,F)
new=zeros(3,1);
A=[0.8 0.1 0.1;
    0.1 0.8 0.1;
    0.1 0.1 0.8];
old=A*old;
new(1)=old(1)*(exp(-(F(8))^2-4*F(9)^2));
if F(8)>0
    new(2)=old(2)*(1-exp(-(F(8))^2-4*F(9)^2));
    new(3)=0;
else
    new(3)=old(3)*(1-exp(-(F(8))^2-4*F(9)^2));
    new(2)=0;
end
normfactor=sum(new);
new=new./normfactor;
end