
im1 = imread('left.bmp');
im2 = imread('right.bmp');
% convert to gray-scale image
I1 = rgb2gray(im1);
I2 = rgb2gray(im2);

K = [350 0 960/2;
     0 350 540/2;
     0 0 1];
[x1, x2] = FindMatch(I1, I2);
%figure; showMatchedFeatures(im1,im2,x1,x2,'montage');
[F] = ComputeF(x1, x2);

%{
% visualizing epipolar lines
figure; subplot(121); imshow(im1);
title('Epipolar Lines in First Image'); hold on;
plot(x1(inliers,1),x1(inliers,2),'go')
epiLines = epipolarLine(F',x2(inliers,:));
points = lineToBorderPoints(epiLines,size(im1));
line(points(:,[1,3])',points(:,[2,4])');
subplot(122); imshow(im2);
title('Epipolar Lines in Second Image'); hold on;
plot(x2(inliers,1),x2(inliers,2),'go')
epiLines = epipolarLine(F,x1(inliers,:));
points = lineToBorderPoints(epiLines,size(im2));
line(points(:,[1,3])',points(:,[2,4])');
%}

% Compute four configurations of camera pose given F
[R1, C1, R2, C2, R3, C3, R4, C4] = ComputeCameraPose(F, K);

% Triangulate Points using four configurations
% e.g., P1: reference camera projection matrix at origin,
% P2: relative camera projection matrix with respect to P1

P1 = K * (R1 * [eye(3) zeros(3,1)]);
P2 = K * (R1 * [eye(3) -C1]);

X1 = Triangulation(P1, P2, x1, x2);
% X2 = Triangulation(P1, P2, x1, x2);
% X3 = Triangulation(P1, P2, x1, x2);
% X4 = Triangulation(P1, P2, x1, x2);

%{
% Disambiguate camera pose
[R,C,X] = DisambiguatePose(R1,C1,X1,R2,C2,X2,R3,C3,X3,R4,C4,X4);

im_w = WarpImage(im, H)
% Stereo rectification
[H1, H2] = ComputeRectification(K, R, C);
im1_w = WarpImage(im1, H1);
im2_w = WarpImage(im2, H2);

im1_w = imresize(im1_w, 0.5);
im2_w = imresize(im2_w, 0.5);
[disparity] = DenseMatch(im1_w, im2_w);

figure(1)
clf;
imagesc(disparity);
axis equal
axis off
colormap(jet);
%}



