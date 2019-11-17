function [R,C,X] = DisambiguatePose(R1,C1,X1,R2,C2,X2,R3,C3,X3,R4,C4,X4)
%Input: R1, C1, X1 · · · R4, C4, X4 are four sets of camera rotation, center, and 3D reconstructed points.
%Output: R, C, X are the best camera rotation, center, and 3D reconstructed points.
%Description: The 3D point must lie in front of the both cameras, which can
%be tested by: r3'(X-C)>0, where r3 is the 3rd row of the rotation matrix.

n = length(X1);
nValid = [0, 0, 0, 0];
Ri = {R1, R2, R3, R4};
Ci = {C1, C2, C3, C4};
Xi = {X1, X2, X3, X4};

for k = 1:length(Ri)
    for i = 1:n
        cheirality = Ri{k}(3,:) * (Xi{k}(i,:)' - Ci{k});
        if cheirality > 0
            nValid(k) = nValid(k) + 1;
        end
    end
end
[~,idx] = max(nValid);
R = Ri{idx};
C = Ci{idx};
X = Xi{idx};
nValid

