function [n,d,s]= computeP(T_C2,H,K,delta_T)
% output normal, d, and scale

R = delta_T(1:3,1:3);
t = delta_T(1:3,4);

s = median(svd(inv(K) * H * K));
nd = (1/s * (inv(K) * H * K) - R)./t;
nd = mean(nd,1); % this is n/d
d = 1/norm(nd); % distance
n = T_C2 * [nd,0]';
n = n/norm(n); % normal vector

%function [x]= computeP(H,K,R,t)
% 
% t1 = [t(1), t(1), t(1)];
% t2 = [t(2), t(2), t(2)];
% t3 = [t(3), t(3), t(3)];
% 
% T = [diag(t1); diag(t2); diag(t3)];
% 
% H = inv(K) * H * K;
% %H = K * H * inv(K);
% 
% A = vertcat([-T,H(:)]);
% b = R(:);
% 
% x = A\b;
% 
% lambda = x(4);
% np = x(1:3);
% d = 1/norm(np);
% n = d*np;

