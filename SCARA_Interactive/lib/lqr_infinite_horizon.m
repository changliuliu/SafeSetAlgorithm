function [K, P] = lqr_infinite_horizon(A, B, Q, R)

%% find the infinite horizon K and P through running LQR back-ups
%%   until norm(K_new - K_current, 2) <= 1e-4  
flag=1;
K=zeros(size(R,1),size(Q,2));
P=zeros(size(A,1));
while flag>0
P_new=Q+K'*R*K+(A+B*K)'*P*(A+B*K);
K_new=-inv(R+B'*P_new*B)*B'*P_new*A;
if norm(K_new-K,2)<=1e-4
    flag=0;
end
K=K_new;
P=P_new;
end