[x,y]=meshgrid(linspace(-10, 10),linspace(-10, 10))
X = reshape(x,1,[]);
Y = reshape(y,1,[]);
grid_plot = [X;Y]'
z_total = [grid_plot, repmat(pi/2,100*100,1)];
car1x = 0
car1y = 0;

dist_from_center = 1.4;
circle_radius    = sqrt(2);
car_size = 2*circle_radius;%proper +sqrt(2)%this takes into account of the size of the ego size and other car size
car_position_1_x = z_total(:,1) + cos(z_total(:,3))*dist_from_center;
car_position_1_y = z_total(:,2) + sin(z_total(:,3))*dist_from_center;
car_position_2_x = z_total(:,1) - cos(z_total(:,3))*dist_from_center;
car_position_2_y = z_total(:,2) - sin(z_total(:,3))*dist_from_center;

cons1 = (car_position_1_x - (car1x-1.4)).^2 + (car_position_1_y - car1y).^2 - car_size^2 > 0
cons2 = (car_position_1_x - (car1x+1.4)).^2 + (car_position_1_y - car1y).^2 - car_size^2 >0
cons3 = (car_position_2_x - (car1x-1.4)).^2 + (car_position_2_y - car1y).^2 - car_size^2 >0
cons4 = (car_position_2_x - (car1x+1.4)).^2 + (car_position_2_y - car1y).^2 - car_size^2 > 0
total_cons = reshape(cons1&cons2&cons3&cons4,[100,100]);


contour(x,y, double(total_cons))
axis equal