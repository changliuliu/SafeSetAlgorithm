function robot=robotproperty(id)
switch id
    case 1
        %the constants
        robot.nlink=2;
        robot.umax=[5,2]; %m/s^2
        robot.margin=0.05;
        robot.delta_t=0.05;
        robot.wmax=1; %rad/s
        
        %The moments, lengths of the links and DH parameter and base       
        robot.I=[0.123 0.028]; %Moments of inertia (kg.m^2)
        robot.l=[0.32 0.215]; %Length of the links (m)
        robot.m=[6.83 3.29]; %Mass of the links (Kg)
        robot.M=[5.56 1.05]; %Mass of the second motor and the end-effector
        robot.Kt=[22.36 2.42]; %Torque constant of the motors (N.m/V)
        robot.DH=[0 0 robot.l(1) 0;pi/4 0 robot.l(2) 0]; %theta,d,a,alpha
        robot.base=[-30;-30;0]./100; %origin
        
        %Parameters in dynamics
        robot.param=[];
        robot.param(1)=robot.I(1)+robot.I(2)+(robot.m(1)/4+robot.m(2))*robot.l(1)^2+...
            robot.m(2)*robot.l(2)^2/4+robot.M(1)*robot.l(1)^2+robot.M(2)*(robot.l(1)^2+robot.l(2)^2);
        robot.param(2)=robot.I(2)+robot.m(2)*robot.l(2)^2/4+robot.M(2)*robot.l(2)^2;
        robot.param(3)=robot.m(2)*robot.l(1)*robot.l(2)/2+robot.M(2)*robot.l(1)*robot.l(2);

end


%The kinematic matrices
robot.A=[eye(robot.nlink) robot.delta_t*eye(robot.nlink);zeros(robot.nlink) eye(robot.nlink)];
robot.B=[0.5*robot.delta_t^2*eye(robot.nlink);robot.delta_t*eye(robot.nlink)];
robot.C=eye(2*robot.nlink);
robot.D=zeros(2*robot.nlink,robot.nlink);
robot.Q=diag([ones(1,robot.nlink) zeros(1,robot.nlink)]);%[1 robot.delta_t 0 0;robot.delta_t robot.delta_t^2 0 0;0 0 1 robot.delta_t;0 0 robot.delta_t robot.delta_t^2];
robot.R=eye(robot.nlink);
robot.Goal=[-0.2,-0.1]';
robot.nG=size(robot.Goal,2);

robot.x(1:2*robot.nlink,1)=[robot.DH(:,1);zeros(robot.nlink,1)];%(theta1,...,thetaN,theta1dot,...,thetaNdot)
robot.pos=ArmPos(robot.base,robot.DH,robot.x(1:robot.nlink,1));%(x1,y1,z1,...,x(N+1),y(N+1),z(N+1))
robot.wx(1:3*robot.nlink,1)=[robot.pos(end-2:end);0;0;0];%endpoint state(x(N+1),y(N+1),z(N+1),x(N+1)dot,y(N+1)dot,z(N+1)dot)
robot.mx=robot.wx;%closest point state

robot.ref.x=robot.x;
robot.innoise=0;
robot.outnoiseself=0;
robot.outnoisestar=0;
robot.obs.xself=[];
robot.obs.xstar=[];
robot.obs.goal=[];
robot.obs.A=robot.A;
robot.obs.B=robot.B;
robot.obs.C=robot.C;
robot.obs.D=robot.D;
robot.obs.Q=robot.Q;
robot.obs.R=robot.R;
robot.score=0;
robot.inf.A={};
robot.inf.B={};
robot.inf.F={};
robot.inf.A{1}=robot.A;
robot.inf.B{1}=[eye(4) robot.B];
robot.inf.F{1}=eye(10);
robot.flag=0;

%For SSA
robot.const.P1=[eye(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) zeros(robot.nlink)];
robot.const.P2=[zeros(robot.nlink) 0.5*eye(robot.nlink);0.5*eye(robot.nlink) zeros(robot.nlink)];
robot.const.P3=[zeros(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) eye(robot.nlink)];

%For collision check
robot.profile={};





