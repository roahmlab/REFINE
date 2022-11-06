function [gen1,gen2] = get_footprint_gen(gen,angle)
w = gen(1);h = gen(2);
vertices = [w -w w  -w;
            h h  -h -h];

points = [];
for theta = -angle:0.02:angle
points_iter = rotmat(theta)*vertices;
points = [points points_iter];
% scatter(points_iter(1,:),points_iter(2,:));
end
points = [points vertices rotmat(-angle)*vertices rotmat(angle)*vertices];
gen1 = (max(points(1,:))-min(points(1,:)))/2;
gen2 = (max(points(2,:))-min(points(2,:)))/2;
end