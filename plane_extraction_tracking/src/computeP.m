%function [n,d,lambda]= computeP(H,K,R,t)
function [x]= computeP(H,K,R,t)

t1 = [t(1), t(1), t(1)];
t2 = [t(2), t(2), t(2)];
t3 = [t(3), t(3), t(3)];

T = [diag(t1); diag(t2); diag(t3)];

H = inv(K) * H * K;

A = vertcat([T,H(:)]);
b = R(:);

x = A\b;

lambda = x(4);
d = norm(x(1:3));
n = x(1:3);
