function [R,C,X] = DisambiguatePose(R1,C1,X1,R2,C2,X2,R3,C3,X3,R4,C4,X4)
%Input: R1, C1, X1 · · · R4, C4, X4 are four sets of camera rotation, center, and 3D reconstructed points.
%Output: R, C, X are the best camera rotation, center, and 3D reconstructed points.
%Description: The 3D point must lie in front of the both cameras, which can
%be tested by: r3'(X-C)>0, where r3 is the 3rd row of the rotation matrix.

n = length(X1);
nValid = [];
c1 = 0; c2 = 0; c3 = 0; c4 = 0;

for i = 1:n
    cheirality_X1 = R1(3,:) * (X1(i,:)'-C1);
    if cheirality_X1 > 0
        c1 = c1 + 1;
    end
    
    cheirality_X2 = R2(3,:) * (X2(i,:)'-C2);
    if cheirality_X2 > 0
        c2 = c2 + 1;
    end
    
    cheirality_X3 = R3(3,:) * (X3(i,:)'-C3);
    if cheirality_X3 > 0
        c3 = c3 + 1;
    end
    
    cheirality_X4 = R4(3,:) * (X4(i,:)'-C4);
    if cheirality_X4 > 0
        c4 = c4 + 1;
    end
end

nValid = [c1, c2, c3, c4];
[~,idx] = max(nValid);


