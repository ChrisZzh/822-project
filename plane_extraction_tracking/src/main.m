%%
%
% Read in two images, extract dominant planes in HSV color space
% estimate homography, estimate n and d
%
%
clear all
close all
%% read image
img_1 = imread('../../822_test/images/left_330_1528404308395556281.jpg');
img_2 = imread('../../822_test/images/left_350_1528404309395055925.jpg');

[f_1, d_1] = HSV_extraction(img_1,4);
[f_2, d_2] = HSV_extraction(img_2,4);

%%
% THRESHHOLD: A descriptor D1 is matched to a descriptor D2 only if 
% the distance d(D1,D2) multiplied by THRESH is not greater than 
% the distance of D1 to all other descriptors.
[matches, scores] = vl_ubcmatch(d_1,d_2,1.5); 

loc_1 = f_1(1:2,matches(1,:));
loc_2 = f_2(1:2,matches(2,:));

%%
[bestH, index, NumInlier] = ransacH(loc_1',loc_2',2000,1);
% bestH transforms loc_2 to loc_1

figure; clf;
imagesc(cat(2, img_1, img_2));
hold on
h = line([loc_1(1,index) ; loc_2(1,index)+size(img_1,2)], [loc_1(2,index); loc_2(2,index)]);
set(h,'linewidth', 1, 'color', 'b');
plot([loc_1(1,index);(loc_2(1,index)+size(img_1,2))], [loc_1(2,index); loc_2(2,index)], 'r*');
% 
% vl_plotframe(f_1(:,matches(1,:)));
% f_2_v = f_2;
% f_2_v(1,:) = f_2_v(1,:) + size(img_1,2);
% vl_plotframe(f_2_v(:,matches(2,:)));
hold off

%% get poes
pose1 = [ -10.409359422989400201 0.64363922634339942874 -0.016079560710826553555 0.16140925110415874077 -0.17935746748085373836 0.62977950061038068252 0.7383466212611720092];
pose2 = [ -10.727217524362290035 0.45879971033240496414 -0.018033139705854673435 0.19446997480199282315 -0.13941221159040956268 0.79118088697298372125 0.56283076341705617907];

T_I1 = [quat2rotm([pose1(7),pose1(4:6)]),pose1(1:3)';[0,0,0,1]];
T_I2 = [quat2rotm([pose2(7),pose2(4:6)]),pose2(1:3)';[0,0,0,1]];

T_I_C = [quat2rotm([0.499079072301088, -0.505950871509703, -0.497347934968811, 0.497572936152999]),[-0.13537, -0.11358, 0.015839]';[0,0,0,1]];

T_C1 = T_I1 * T_I_C;
T_C2 = T_I2 * T_I_C;

%delta_T = inv(T_C1)*T_C2;
delta_T = inv(T_C2)*T_C1;

K = [498.1357145, 0, 351.726944;0,498.1357145,255.9642885;0,0,1];
%% Compute n d
R = delta_T(1:3,1:3);
t = delta_T(1:3,4);

%[n, d, lambda] = computeP(bestH, K, R, t);
x = computeP(bestH, K, R, t);

% T_C2*[n;0]

%syms lambda n1 n2 n3 d real
%[l,n1_,n2_,n3_,d_, val_] = fmincon(fun,[l_0,n1_0,n2_0,n3_0,d_0],A=[],b=[],Aeq=[],Beq=[],lb=[],ub=[],ceq);
% fun = @(lambda,n1,n2,n3,d) lambda * inv(K)*bestH*K - R + t * [n1,n2,n3]/d;
% fun = @(n1,n2,n3,d)  ( R - t * [n1,n2,n3]/d)./(inv(K)*bestH*K);
% ceq = @(n1,n2,n3)sqrt(n1^2+n2^2+n^3^2)-1;
l_0 = 1;
n1_0 = -0.924;
n2_0 = -0.017;
n3_0 = 0.3817;
d_0 = 1/10.7258;


0.0058
1.5883
0.8470
     0






