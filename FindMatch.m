function [x1, x2] = FindMatch(I1, I2)
%%Input: two input gray-scale images with uint8 format.
%Output: x1 and x2 are n × 2 matrices that specify the correspondence.
%Description: Each row of x1 and x2 contains the (x, y) coordinate of the point correspondence
%in I1 ad I2, respectively, i.e., x1(i,:) ? x2(i,:). This matching function is similar to HW#2
%except that bidirectional consistency check is mandatory.

match_ratio = 0.7;
% Assume that T1 and T2 have (nearly) same dimension
% change the images to single precision
im1 = im2single(I1);
im2 = im2single(I2);

% calculaTe SIFT frames (keypoints) and descriptors
% f1, f2 have column for each frame; a disk of center f(1:2), scale f(3) and orientation f(4)
[fa, da] = vl_sift(im1);
[fb, db] = vl_sift(im2);

% make the dimension of features to be the same
if size(fa,2) > size(fb,2)
    fa = fa(:,1:size(fb,2));
    da = da(:,1:size(db,2));
else
    fb = fb(:,1:size(fa,2));
    db = db(:,1:size(da,2));
end

% get transpose of f and d
f1 = fa'; f2 = fb'; d1 = da'; d2 = db';

% get the nearest neighbour points and euclidean distance in d1 for each
% descriptor in d2; Idx - index of the matched descriptor; D - distance
% this gives the match of d1(idx(i),:) <-> d2(i,:); I2 to I1
[Idx1, D1] = knnsearch(double(d1), double(d2),'K',2,'Distance','euclidean');

% this gives the match of d2(idx(i),:) <-> d1(i,:); I1 to I2
[Idx2, D2] = knnsearch(double(d2), double(d1),'K',2,'Distance','euclidean');

D = 0;
if length(D1)~=length(D2)
   D = min(D1,D2);
else
    D = D1;
end
% ratio test
x1 = []; x2 = [];
x1_1 = []; x2_1 = [];
x1_2 = []; x2_2 = [];
for i = 1:size(D,1)
    if D(i,1) < match_ratio * D(i,2)
        % I2 to I1
        x1_1 = [x1_1; f1(Idx1(i,1),1) f1(Idx1(i,1),2)];
        x2_1 = [x2_1; f2(i,1) f2(i,2)];
        
        % I1 to I2
        x1_2 = [x1_2; f1(i,1) f1(i,2)];
        x2_2 = [x2_2; f2(Idx2(i,1),1) f2(Idx2(i,1),2)];
    end
end

% bi-directional check
[~,id] = ismember(x1_1,x1_2,'row');
for i=1:length(id)
    if id(i) ~= 0
        if x2_1(i,:)==x2_2(id(i),:)
            x1 = [x1; x1_1(i,:)];
            x2 = [x2; x2_2(id(i),:)];
        end
    end
end


