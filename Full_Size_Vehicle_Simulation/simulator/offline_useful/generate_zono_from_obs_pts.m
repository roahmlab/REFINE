function [c_out, g_out] = generate_zono_from_obs_pts(XY)
% Takes four XY coordinates corresponding to the corners of an offset and
% rotated rectangle and generates the center, generators of a zonotope for 
% the box.
if isnan(XY(1,end))
XY = XY(:,1:end-1);
end
% Choose one point as a temporary origin
pt0 = XY(:,1);
XY_offset = XY - pt0;

% Compute angles from origin to each of the other three points
thetas = angle(XY_offset(1,2:end) + 1i * XY_offset(2,2:end));
% thetas - atan2(XY_offset(2,2:end), XY_offset(1,2:end))

% Find two points that are 90 degrees apart (i.e. two diagonals) to find
% the rotation of the angle
delta1_2 = angdiff(thetas(1), thetas(2));
delta1_3 = angdiff(thetas(1), thetas(3));
delta2_3 = angdiff(thetas(2), thetas(3));
theta_eps = 1e-3;

if abs(abs(delta1_2) - pi/2) < theta_eps
    rz = rotz(-rad2deg(thetas(1)));
elseif abs(abs(delta1_3) - pi/2) < theta_eps
    rz = rotz(-rad2deg(thetas(1)));
else % corresponds to delta2_3 if all points are corners of a rectangle.
    rz = rotz(-rad2deg(thetas(2)));
end

% We are in 2D, so throw out z component of rotation matrix
rz = rz(1:2,1:2);

% Align the corners with the x and y axes
XY_offset_rot = rz * XY_offset;

% Compute center, generators for the box now that its corners are
% aligned with x, y
X_offset_rot = XY_offset_rot(1,:);
Y_offset_rot = XY_offset_rot(2,:);
max_x = max(X_offset_rot);
min_x = min(X_offset_rot);
max_y = max(Y_offset_rot);
min_y = min(Y_offset_rot);
range_x = max_x - min_x;
range_y = max_y - min_y;
center_pt = [range_x / 2 + min_x; range_y / 2 + min_y];
generators = [range_x/2  0;
              0          range_y/2];

% Transform back to the original obstacle coordinates and rotation
c_out = (rz' * center_pt) + pt0;
g_out = (rz' * generators);
end
