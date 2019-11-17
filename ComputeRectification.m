function [H1, H2] = ComputeRectification(K, R, C)
%Input: The relative camera pose (R and C) and intrinsic parameter K.
%Output: H1 and H2 are homographies that rectify the left and right images such that the epipoles are at infinity.
%Description: Given the disambiguated camera pose, you can find the rectification rotation matrix,
%Rrect such that the x-axis of the images aligns with the baseline. Find the rectification homography
%H = KRrectRTK?1 where R is the rotation matrix of the camera. The rectified images are shown in Figure 6.
%This rectification sends the epipoles to infinity where the epipolar line becomes horizontal.

r_z_tilde = [0; 0; 1];
r_x = C/norm(C);
orth_proj = r_z_tilde - (dot(r_z_tilde, r_x) * r_x);
r_z = orth_proj / norm(orth_proj);
r_y = cross(r_z, r_x);
R_rect = [r_x'; r_y'; r_z'];

H1 = K * R_rect * inv(K);
H2 = K * R_rect * R' * inv(K);

