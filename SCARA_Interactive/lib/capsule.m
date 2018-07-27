function handle=capsule(n,pos,x,offset,color1,sbound,agent,color2)
handle=[];
for i=2:n
    x(i)=x(i)+x(i-1);
end
for i=0:n-1
    % two parallel lines
handle(i+1,1)=plot(pos(3*i+1:3:3*i+4)-offset(i+1)*sin(x(i+1)),pos(3*i+2:3:3*i+5)+offset(i+1)*cos(x(i+1)),'-.','color',color1);
handle(i+1,2)=plot(pos(3*i+1:3:3*i+4)+offset(i+1)*sin(x(i+1)),pos(3*i+2:3:3*i+5)-offset(i+1)*cos(x(i+1)),'-.','color',color1);
    % half circles
handle(i+1,3)=plot(pos(3*i+1)-offset(i+1)*sin(x(i+1):0.01:x(i+1)+pi),pos(3*i+2)+offset(i+1)*cos(x(i+1):+0.01:x(i+1)+pi),'-.','color',color1);
handle(i+1,4)=plot(pos(3*i+4)-offset(i+1)*sin(x(i+1):-0.01:x(i+1)-pi),pos(3*i+5)+offset(i+1)*cos(x(i+1):-0.01:x(i+1)-pi),'-.','color',color1);
end

 %sbound=0.1;
 %h=fill(agent(1)+sbound*cos(0:0.01:2*pi),agent(2)+sbound*sin(0:0.01:2*pi),color2,'EdgeColor','None');
 %alpha(h,0.1);
end