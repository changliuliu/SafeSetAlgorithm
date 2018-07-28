function robot=robotproperty(interact)
robot.umax=10;
robot.margin=3;
robot.delta_t=0.1;
robot.A=[1 robot.delta_t 0 0 ;0 1 0 0;0 0 1 robot.delta_t;0 0 0 1];
robot.B=[0.5*robot.delta_t^2 0;robot.delta_t 0;0 0.5*robot.delta_t^2;0 robot.delta_t];
robot.C=eye(4);
robot.D=zeros(4,2);
robot.Q=diag([1;0;1;0]);%[1 robot.delta_t 0 0;robot.delta_t robot.delta_t^2 0 0;0 0 1 robot.delta_t;0 0 robot.delta_t robot.delta_t^2];
robot.R=eye(2);
robot.Goal=[-1,-1;-1,3]';
robot.nG=size(robot.Goal,2);
robot.x(1:4,1)=[-10;0;-10;0];
robot.ref.x=robot.x;
robot.innoise=0;
robot.outnoiseself=0;
robot.outnoisestar=0;
robot.obs.xself=[];
robot.obs.xstar=[];
robot.obs.goal=[];
if interact==1
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
end
robot.const.Q2=[0 0.5 0 0;0.5 0 0 0;0 0 0 0.5;0 0 0.5 0];
robot.const.Q1=diag([1;0;1;0]);
robot.const.Q3=diag([0;1;0;1]);


