function [disparity] = DenseMatch(im1, im2)
%Input: two gray-scale rectified images with uint8 format.
%Output: disparity map disparity ? RH×W where H and W are the image height and width.
%Description: Compute the dense matches across all pixels. Given a pixel, u in the left image,
%sweep along its epipolar line, lu, and find the disparity, d, that produces the best match.

[H, W] = size(im1);
disparity = zeros(H, W);

% convert to single
im1 = single(im1);
im2 = single(im2);
step = 20;

for i = 1:H
    d = [];
    for j = 1:W
        % du_l: SIFT descriptors on left image at u
        %s1 = [j, window_size/2]; % start and end of window
        [fl, du_l] = vl_dsift(im1(i,j), 'Step', step);
        fl = fl'; du_l = du_l'; % get transpose of f and d
        %du_l = im1(i,j);
        
        for k = 1:W
            % du_r: SIFT descriptors on right image at u+(i,0) center
            [fr, du_r] = vl_dsift(im2(i,k), 'Step', step);
            fr = fr';  du_r = du_r'; % get transpose of f and d
            %du_r = im2(i,k);
            
            % calculate correlation
            d = [d; sum(sum(abs(du_l - du_r)))];
        end
        disparity(i,j) = min(d);
    end
end



