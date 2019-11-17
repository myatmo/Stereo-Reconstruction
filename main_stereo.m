
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
%F = estimateFundamentalMatrix(x1, x2);

% Compute four configurations of camera pose given F
[R1, C1, R2, C2, R3, C3, R4, C4] = ComputeCameraPose(F, K);

% Triangulate Points using four configurations
% e.g., P1: reference camera projection matrix at origin,
% P2: relative camera projection matrix with respect to P1
P0 = K * [eye(3) zeros(3,1)];
P1 = K * (R1 * [eye(3) -C1]);
X1 = Triangulation(P0, P1, x1, x2);

P2 = K * (R2 * [eye(3) -C2]);
X2 = Triangulation(P0, P2, x1, x2);

P3 = K * (R3 * [eye(3) -C3]);
X3 = Triangulation(P0, P3, x1, x2);

P4 = K * (R4 * [eye(3) -C4]);
X4 = Triangulation(P0, P4, x1, x2);

% Disambiguate camera pose
[R,C,X] = DisambiguatePose(R1,C1,X1,R2,C2,X2,R3,C3,X3,R4,C4,X4);

% Stereo rectification
[H1, H2] = ComputeRectification(K, R, C);

im1_w = WarpImage(im1, H1);
im2_w = WarpImage(im2, H2);
subplot(1,2,1), imshow(im1_w)
subplot(1,2,2), imshow(im2_w)

im1_w = imresize(im1_w, 0.1);
im2_w = imresize(im2_w, 0.1);
im1_w = rgb2gray(im1_w);
im2_w = rgb2gray(im2_w);
[disparity] = DenseMatch(im1_w, im2_w);

figure(1)
clf;
imagesc(disparity);
axis equal
axis off
colormap(jet);
disp = disparity;
%save 'stereo.mat' x1, x2, F, X, H1, H2, im1_w, im2_w, disp

