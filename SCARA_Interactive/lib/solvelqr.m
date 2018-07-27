function robot=solvelqr(robot,horizon)

    P={};K={};b={};d={};
    P{horizon+1}=100*eye(4);
    goal=[robot.Goal(1,1);0;robot.Goal(2,1);0];
    b{horizon+1}=goal';
    
    for i=horizon:-1:1
        K{i}=inv(robot.R+robot.B'*P{i+1}*robot.B)*robot.B'*P{i+1}*robot.A;
        d{i}=inv(robot.R+robot.B'*P{i+1}*robot.B)*robot.B'*b{i+1}';
        P{i}=-robot.const.Q2+K{i}'*robot.R*K{i}+(robot.A-robot.B*K{i})'*P{i+1}*(robot.A-robot.B*K{i});
        b{i}=1*(-robot.inf.xstar(:,i)'*robot.Q+d{i}'*robot.R*K{i}-d{i}'*robot.B'*P{i+1}*(robot.A-robot.B*K{i})+b{i+1}*(robot.A-robot.B*K{i}));
    end
    robot.K=K;
    robot.P=P;
    robot.d=d;
    robot.b=b;
end