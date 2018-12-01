function H2to1 = computeH(p1,p2)
% INPUTS:
% p1 and p2 - Each are size (2 x N) matrices of corresponding (x, y)'  
%             coordinates between two images
%
% OUTPUTS:
% H2to1 - a 3 x 3 matrix encoding the homography that best matches the linear 
%         equation
%% Construct A
% reorganize the entries to aviod for-loop
N = size(p1,2);
Xbef = p2(1,:);
Ybef = p2(2,:);
Xaft = p1(1,:);
Yaft = p1(2,:);
AugZeros = zeros(N,3);
AugIn = -[Xbef', Ybef', ones(N,1)];

Ay = [AugZeros, AugIn, Yaft'.*Xbef', Yaft'.*Ybef', Yaft'];
Ax = [AugIn, AugZeros, Xaft'.*Xbef', Xaft'.*Ybef', Xaft'];
A = [Ax;Ay]; 

%% Solve for H
[~,~,V] = svd(A);
H2to1 = reshape(V(:,end),[3,3])';
