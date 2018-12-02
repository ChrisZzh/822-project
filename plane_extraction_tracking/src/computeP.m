function [n,d,lambda]= computeP(H,K,R,t)
%function [x]= computeP(H,K,R,t)

t1 = [t(1), t(1), t(1)];
t2 = [t(2), t(2), t(2)];
t3 = [t(3), t(3), t(3)];

T = [diag(t1); diag(t2); diag(t3)];

H = inv(K) * H * K;
%H = K * H * inv(K);

A = vertcat([T,H(:)]);
b = R(:);

x = A\b;

lambda = x(4);
np = x(1:3);
d = 1/norm(np);
n = d*np;
