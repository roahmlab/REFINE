clear, close all, clc


bounds = [-500,0,-0.7,12.7] ;
goal_radius = 6;
world_buffer = 1 ; % this is used to make start and goal locations that are not too close to the robot or boundary
verbose_level = 0;
W = dynamic_car_world('bounds',bounds,'buffer',world_buffer,...
    'verbose',verbose_level,'goal_radius',goal_radius,'num_cars',12) ;







