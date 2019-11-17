function [X] = Triangulation(P1, P2, x1, x2)
%Input: P1 and P2 are two camera projection matrices (R3×4). x1 and x2 are
%n × 2 matrices that specify the correspondence.
%Output: X is n × 3 where each row specifies the 3D reconstructed point.
%Description: Use the triangulation method by linear solve.

n = length(x1);
X = zeros(n,3);
for i = 1:n
    % u is for x1 points. get skew symmetric matrix for x1
    u = [x1(i,:)'; 1];
    skew_x1 = [0 -u(3) u(2);
               u(3) 0 -u(1);
               -u(2) u(1) 0];
    
    % v is for x2 points. get skew symmetric matrix for x2
    v = [x2(i,:)'; 1];
    skew_x2 = [0 -v(3) v(2);
               v(3) 0 -v(1);
               -v(2) v(1) 0];
           
    % stack x1 and x2 skew matrices multiplied by P matrix
    A = [skew_x1*P1; skew_x2*P2];
    
    % solve this in the form of Ax = 0
    [~,~,V] = svd(A);
    V_nor = V(:,end)/V(end,end);
    X(i,:) = V_nor(1:3,end);
end

