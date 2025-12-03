function [L] = determineTravelDistance(P1, P2)
%Used to determine euclidean travel distance.
delta_x = P1(1)-P2(1);
delta_y = P1(2)-P2(2);
L = sqrt(delta_x^2 + delta_y^2);
end