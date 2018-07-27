function [xfilt,varargout] = kf_smooth(y, A, B, C, d, u, Q, R, init_x, init_V)
%
% function [xfilt, xpred, Vfilt, loglik, xsmooth, Vsmooth, Q, R] = 
%           kf_smooth(y, A, B, C, d, u, Q, R, init_x, init_V)
%
%
% Kalman filter
% [xfilt, xpred, Vfilt] = ekf_smooth(y_all, A, B, C, d, Q, R, init_x, init_V);
%
% Kalman filter with Smoother
% [xfilt, xpred, Vfilt, loglik, xsmooth, Vsmooth] = ekf_smooth(y_all, A, B, C, d, Q, R, init_x, init_V);
%
% Kalman filter with Smoother and EM algorithm
% [xfilt, xpred, Vfilt, loglik, xsmooth, Vsmooth, Q, R] = ekf_smooth(y_all, A, B, C, d, Q, R, init_x, init_V);
%
%
% INPUTS:
% y - observations
% A, B, C, d:  x(:,t+1) = A x(:,t) + B u(:,t) + w(:,t) 
%              y(:,t)   = C x(:,t) + d        + v(:,t)
% Q - covariance matrix of system x(t+1)=A*x(t)+w(t) , w(t)~N(0,Q)
% R - covariance matrix of output y(t)=C*x(t)+v(t) , v(t)~N(0,R)
% init_x -
% init_V -
%
%
% OUTPUTS:
% xfilt = E[X_t|t]
% varargout(1) = xpred - the filtered values at time t before measurement
% at time t has been accounted for
% varargout(2) = Vfilt - Cov[X_t|0:t]
% varargout(3) = loglik - loglikelihood
% varargout(4) = xsmooth - E[X_t|0:T]
% varargout(5) = Vsmooth - Cov[X_t|0:T]
% varargout(6) = Q - estimated system covariance according to 1 M step (of EM)
% varargout(7) = R - estimated output covariance according to 1 M step (of EM)

nargout=10;
n_var_out = max(nargout,1)-1; % number of variable number of outputs

T = size(y,2);
ss = size(Q,1); % size of state space
n=size(u,2);
%% Forward pass (Filter)

%YOUR code here
Vpred={};
Vpred{1}=init_V;
Vfilt={};
xfilt=zeros(size(A,1),size(u,2));
xpred=zeros(size(A,1),size(u,2));
xpred(:,1)=init_x;
Q_loglik=zeros(size(Q));
R_loglik=zeros(size(R));
p=0;
for t=1:size(u,2)
    %measurement update
    K=Vpred{t}*C'/(C*Vpred{t}*C'+R);
    xfilt(:,t)=xpred(:,t)+K*(y(:,t)-(C*xpred(:,t)+d));
    Vfilt{t}=(eye(size(K,1))-K*C)*Vpred{t};
    %dynamic update
    if t<size(u,2)
    xpred(:,t+1)=A*xfilt(:,t)+B*u(:,t);
    Vpred{t+1}=A*Vfilt{t}*A'+Q;
    end
    [~,definite]=chol(C*Vpred{t}*C'+R);
    if definite==1
        answer=t
    else
        if t<1000
        %p=p+log(mvnpdf(y(:,t),C*xpred(:,t)+d,C*Vpred{t}*C'+R));
        p=p-0.5*(y(:,t)-C*xpred(:,t)-d)'*inv(C*Vpred{t}*C'+R)*(y(:,t)-C*xpred(:,t)-d)-0.5*log(det(C*Vpred{t}*C'+R))-log(2*pi);
        end
    end
end
loglik=p;


if(n_var_out >= 1), varargout(1) = {xpred}; end
if(n_var_out >= 2), varargout(2) = {Vfilt}; end
if(n_var_out >= 3), varargout(3) = {loglik}; end


%% Backward pass (RTS Smoother and EM algorithm)
if(n_var_out >= 4)
    
    %YOUR code here
    xsmooth=zeros(size(A,1),n);
    Vsmooth={};
    L={};
    Vsmooth{n}=Vfilt{n};
    xsmooth(:,n)=xfilt(:,n);
    d_loglik=0;
    A1=zeros(size(A,1));
    A2=A1;
    B1=zeros(size(B,1),size(B,2));
    B2=zeros(size(u,1));
    C1=zeros(size(C,1),size(C,2));
    C2=A1;
    for t=size(u,2)-1:-1:1
        L{t}=Vfilt{t}*A'/Vpred{t+1};
        xsmooth(:,t)=xfilt(:,t)+L{t}*(xsmooth(:,t+1)-xpred(:,t+1));
        Vsmooth{t}=Vfilt{t}+L{t}*(Vsmooth{t+1}-Vpred{t+1})*L{t}';
    end
    for t=1:n
        if t>1
            A1=A1+xsmooth(:,t)*xsmooth(:,t-1)'+Vsmooth{t}*L{t-1}'-B*u(:,t-1)*xsmooth(:,t-1)';
            A2=A2+Vsmooth{t-1}+xsmooth(:,t-1)*xsmooth(:,t-1)';
            B1=B1+(xsmooth(:,t)-A*xsmooth(:,t-1))*u(:,t-1)';
            B2=B2+u(:,t-1)*u(:,t-1)';
        end
        C1=C1+(y(:,t)-d)*xsmooth(:,t)';
        C2=C2+Vsmooth{t}+xsmooth(:,t)*xsmooth(:,t)';
        d_loglik=d_loglik+(y(:,t)-C*xsmooth(:,t));
    end
    for t=1:n
        if t>1
            Q_loglik=Q_loglik+(xsmooth(:,t)-(A*xsmooth(:,t-1)+B*u(:,t-1)))*(xsmooth(:,t)-(A*xsmooth(:,t-1)+B*u(:,t-1)))'+A*Vsmooth{t-1}*A'+Vsmooth{t}-Vsmooth{t}*L{t-1}'*A'-A*L{t-1}*Vsmooth{t};
        end
    end
    d_me=d_loglik/(n);
    c_me=C1/C2;
    b_me=B1/B2;
    a_me=A1/A2;
    Q=Q_loglik./(n-1);
	varargout(4) = {xsmooth};
   if(n_var_out >= 5), varargout(5) = {Vsmooth}; end
   if(n_var_out >= 6), varargout(6) = {a_me}; end
   if(n_var_out >= 7), varargout(7) = {b_me}; end
   if(n_var_out >= 8), varargout(8) = {Q}; end
end





















