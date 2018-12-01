%%
%
% Read in two images, extract dominant planes in HSV color space
% estimate homography, estimate n and d
%
%
%% read image
img_1 = imread('~/Desktop/822_test/images/right_0_1528404291835066602.jpg');
img_1_hsv = rgb2hsv(img_1);
img_2 = imread('~/Desktop/822_test/images/right_5_1528404292085050738.jpg');
img_2_hsv = rgb2hsv(img_2);


figure
imshow(img_1)

figure
imshow(img_1_hsv)

