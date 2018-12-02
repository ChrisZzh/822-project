function [bestH, index, NumInlier] = ransacH(locs1, locs2, nIter, tol)
% INPUTS
% locs1 and locs2 - matrices specifying point locations in each of the images
% nIter - number of iterations to run RANSAC
% tol - tolerance value for considering a point to be an inlier
%
% OUTPUTS
% bestH - homography model with the most inliers found during RANSAC
%% Initialization
p1 = locs1';
p2 = locs2';
% number of usable points
N = size(p1,2);
NumInlier = 0;
s = 8; % initial number of seeds
bestH = [];
%% RANSAC Iteration
for i = 1:1:nIter
    ind = randi([1 N],s,1);
    p1Tes = p1(:,ind);
    p2Tes = p2(:,ind);
    % estimate the transformation
    Htes = computeH(p1Tes, p2Tes);
    
    % count the number of inliers
    p1Gen = Htes * vertcat(p2,ones(1,N));
    p1Gen = p1Gen./p1Gen(3,:); % normalization
    p1Gen = p1Gen(1:2,:); % extract Cartisan Coordinate
    TotErr = (p1Gen(1,:)-p1(1,:)).^2+(p1Gen(2,:)-p1(2,:)).^2;
    InlierCount = size(TotErr(TotErr < tol),2);
    
    if InlierCount > NumInlier
        NumInlier = InlierCount;
        bestH = Htes;
        if NumInlier > 0.9 * N
            break;
        end
    end
end

if (isempty(bestH))
    fprintf('Cannot find H within the tol. Try increasing tol.\n');
    assert(1==0);
end
%% output the inlier indexs
p1Gen = bestH * vertcat(p2,ones(1,N));
p1Gen = p1Gen./p1Gen(3,:); % normalization
p1Gen = p1Gen(1:2,:); % extract Cartisan Coordinate
TotErr = (p1Gen(1,:)-p1(1,:)).^2+(p1Gen(2,:)-p1(2,:)).^2;
index = find(TotErr<tol);

bestH = bestH/bestH(3,3);
end