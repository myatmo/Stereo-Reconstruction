function [H1, H2] = ComputeRectification(K, R, C)
%Input: The relative camera pose (R and C) and intrinsic parameter K.
%Output: H1 and H2 are homographies that rectify the left and right images such that the epipoles are at infinity.
%Description: Given the disambiguated camera pose, you can find the rectification rotation matrix,
%Rrect such that the x-axis of the images aligns with the baseline. Find the rectification homography
%H = KRrectRTK?1 where R is the rotation matrix of the camera. The rectified images are shown in Figure 6.
%This rectification sends the epipoles to infinity where the epipolar line becomes horizontal.

