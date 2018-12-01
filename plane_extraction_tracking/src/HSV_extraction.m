%%
%
% Read in two images, extract dominant planes in HSV color space
% estimate homography, estimate n and d
%
%
function [f_sel, d_sel] = HSV_extraction(img_1, k)
%% convert to HSV
img_1_hsv = rgb2hsv(img_1);

row_sz = size(img_1_hsv,1);
col_sz = size(img_1_hsv,2);
pt_sz = row_sz * col_sz;

pts1_hs = reshape(img_1_hsv(:,:,1:2), [pt_sz,2]);
pts1_v = reshape(img_1_hsv(:,:,3), [pt_sz,1]);

%% do Kmeans
% idx_1 = reshape(kmeans(pts1,k),[row_sz,col_sz,1]) * (255/k);
% idx_1 = uint8(floor(idx_1));
% figure
% imshow(idx_1)
idx1_v = reshape(kmeans(pts1_v,k),[row_sz,col_sz,1]);

counts = zeros(k,1);
for i = 1:k
    counts(i) = sum(sum(idx1_v==i));
end
[~,dom_idx] = max(counts);

idx1_v_visualize = idx1_v;
idx1_v_visualize(idx1_v_visualize ~= dom_idx) = 0;
idx1_v_visualize = uint8(floor(idx1_v_visualize * 255));
figure
imshow(idx1_v_visualize)
title('value k-means')

%% do feature work
[f,d] = vl_sift(single(rgb2gray(img_1)));
locs = floor(f(1:2,:));
f_idx = diag(idx1_v(locs(2,:),locs(1,:)));

f_sel = f(:,f_idx == dom_idx);
d_sel = d(:,f_idx == dom_idx);





