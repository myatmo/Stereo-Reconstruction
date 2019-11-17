function [F] = ComputeF(x1, x2)
%Input: x1 and x2 are n × 2 matrices that specify the correspondence.
%Output: F ? R3×3 is the fundamental matrix.
%Description: F is robustly computed by the 8-point algorithm within RANSAC. Note that
%the rank of the fundamental matrix needs to be 2 (SVD clean-up should be applied.).
%You can verify the validity of fundamental matrix by visualizing epipolar line as shown in Figure 3.

ransac_thr = 2;
ransac_iter = 100;

num_matches = size(x1,1);
n = 8; % n is the samplesize
u = x1(:,1);
v = x1(:,2);
u_prime = x2(:,1);
v_prime = x2(:,2);
inliers_index = {}; % to save the sets of inliers per iter; 
                 % each column is a set of indices of x1 and x2 (for inliers) for each iter
F_set = {};

for iter = 1:ransac_iter
    % random sampling
    idx = randsample(num_matches, n);
    
    % model building
    % computing A matrix; Ax = 0 ([8x9][f(9x1)] = 0
    A = zeros(8,9);
    for i = 1:size(idx,1)
        A(i,:) = [u_prime(idx(i)).*u(idx(i)) v_prime(idx(i)).*u(idx(i)) u(idx(i))...
            u_prime(idx(i)).*v(idx(i)) v_prime(idx(i)).*v(idx(i)) v(idx(i))...
            u_prime(idx(i)) v_prime(idx(i)) 1];
    end
    % compute Fundamental matrix
    [~,~,V] = svd(A);
    F_matrix = reshape(V(:,end),3,[])';
    
    % svd clean-up to make rank of F 2
    [U,D,V] = svd(F_matrix);
    F_matrix = U(:, 1) * D(1,1) * transpose(V(:, 1)) + U(:, 2) * D(2,2) * transpose(V(:, 2));
    F_set{end+1} = F_matrix;
    
    % thresholding and counting inliers
    idx = [];
    for i=1:num_matches
        %err = norm([A * reshape(F_matrix', [], 1)]); % calculate the error for each point
        a = [x1(i,:) 1];
        b = [x2(i,:) 1];
        %err = F_matrix * a' / sqrt(a^2 + b^2);
        err = sum((b .* (F_matrix * a')'),2);
        if err < ransac_thr
            idx = [idx; i];
        end
    end
    inliers_index{end+1} = idx;
    
end

% get max size of inliers from inliers set and get the indices
[max_size, max_idx] = max(cellfun('size',inliers_index,1));
inlier_x1 = x1(inliers_index{max_idx},:);
inlier_x2 = x2(inliers_index{max_idx},:);
F = F_set{max_idx};
inliers = inliers_index{max_idx};


