function phaseout = car_highway_dynamics_gpops(input)
m = 1575;    
lf = 1.13;
lr = 1.67;
Izz = 3273; 
Cw = 3.41;%???
Caf1 = 1.716e5;
Caf2 = 1;  
Car1 = 2.9028e5;
Car2 = 1;     
Cbf  = 1e5;
Cbr  = 1e5;
grav_const = 9.8;

% 
% C_delta = 3;
% K_delta = 5; %should not be used, delta control gain, now using linear

A.m  = m;
A.lf = lf;
A.lr = lr;
A.l = lf+lr;
A.grav_const = grav_const;
%             A.Cf1 = 2*C1*A.lr/(A.lr+A.lf) ;
%             A.Cr1 = 2*C1*A.lf/(A.lr+A.lf) ;
A.Cbf = Cbf; A.Cbr = Cbr;
A.Caf1 = Caf1;
A.Caf2 = Caf2;
A.Car1 = Car1;
A.Car2 = Car2;
A.Cw   = Cw;
A.Izz  = Izz;

% t = input.phase.time
z_total = input.phase.state;
inputs_total = input.phase.control;
% params = input.parameter;
% dzdt = zeros(size(z_total));
% for i = 1:size(z_total,1)
%     z_total;
%     inputs_total;
    delta = inputs_total(:,1);
    w_cmd = inputs_total(:,2);
% 
    h = z_total(:,3);
    u = z_total(:,4);
    v = z_total(:,5);
    r = z_total(:,6);
    w = z_total(:,7);
                %             u = w;
% 
% 
    vf = v + A.lf*r;
    vr = v - A.lr*r;

%     
    u_wf = u .* cos(-delta) - vf .* sin(-delta) ; 
    v_wf = u .* sin(-delta) + vf .* cos(-delta) ; 
% %     uv_wf = rot_mat(-delta)*[;];
% % 
    alphar = atan( vr ./ u);
    Fywr = -A.Car1*tanh(A.Car2*alphar);
% % %     betaf = (w - uv_wf(1)) ./ max(sqrt(uv_wf(1).^2 +0.01), sqrt(w.^2 +0.01));
    betaf = (w - u_wf) ./ u_wf;
% %     %                 betar = (w - uv_wr(1)) ./ max(sqrt(uv_wr(1).^2 +0.01), sqrt(w.^2 +0.01));
% % 
    alphaf = atan(v_wf ./ u_wf);
% % % 
    Fxwf = A.Cbf*betaf;
    Fxwr = 0;%A.Cbr*betar;% front wheel drive, assume all force come from there
% % % 
    Fywf = -A.Caf1*tanh(A.Caf2*alphaf);
% % 
    au =  v.*r + (cos(delta).*Fxwf-sin(delta).*Fywf + Fxwr)/A.m;
    av = - u.*r + (sin(delta).*Fxwf+cos(delta).*Fywf + Fywr)/A.m ;
    torque = A.lf*(sin(delta).*Fxwf+cos(delta).*Fywf) - A.lr * Fywr;
    ar = torque/A.Izz;
% alphaf = atan( uv_wf(2) ./ uv_wf(1));
% Fywf = -A.Caf1*tanh(A.Caf2*alphaf);
% betaf = (w - uv_wf(1)) ./ sqrt(uv_wf(1).^2 +0.01);
%      Fxwf = A.Cbf*betaf;
%     au =  v*r + (cos(delta)*Fxwf-sin(delta)*Fywf + Fxwr)/A.m;
    dw = A.Cw*(w_cmd - w);
% 
param = input.phase.parameter;
% au = 0;
% av = 0;
% ar = 0;
% dzdt = [u.*cos(h)-v.*sin(h),u.*sin(h)+v.*cos(h),r,zeros(length(input.phase.time),1),zeros(length(input.phase.time),1),zeros(length(input.phase.time),1),dw];
dzdt = [u.*cos(h)-v.*sin(h),u.*sin(h)+v.*cos(h),r,au,av,ar,dw];
% xdot = u;
phaseout.dynamics = dzdt;
car1x = param(:,3) + input.phase.time.*param(:,5);
car1y = param(:,4);
car2x = param(:,6) + input.phase.time.*param(:,8);
car2y = param(:,7);
car3x = param(:,9) + input.phase.time.*param(:,11);
car3y = param(:,10);
car4x = param(:,12) + input.phase.time.*param(:,14);
car4y = param(:,13);
car5x = param(:,15) + input.phase.time.*param(:,17);
car5y = param(:,16);
car6x = param(:,18) + input.phase.time.*param(:,20);
car6y = param(:,19);
car_width = 2;
car_length = 4.8;
dist_from_center = 1.4;
circle_radius    =1.721;% sqrt(2);
car_size = 2*circle_radius;%proper +sqrt(2)%this takes into account of the size of the ego size and other car size
car_position_1_x = z_total(:,1) + cos(z_total(:,3))*dist_from_center;
car_position_1_y = z_total(:,2) + sin(z_total(:,3))*dist_from_center;
car_position_2_x = z_total(:,1) - cos(z_total(:,3))*dist_from_center;
car_position_2_y = z_total(:,2) - sin(z_total(:,3))*dist_from_center;

% each car will have 4 constraints, pos 1 - car1 left , pos 1 - car1 right,
% pos 2 - car 1 left, pos 2 - car 1 right
phaseout.path = [z_total(:,2), (car_position_1_x - (car1x-1.4)).^2 + (car_position_1_y - car1y).^2 - car_size^2,...
                               (car_position_1_x - (car1x+1.4)).^2 + (car_position_1_y - car1y).^2 - car_size^2,...
                               (car_position_2_x - (car1x-1.4)).^2 + (car_position_2_y - car1y).^2 - car_size^2,...
                               (car_position_2_x - (car1x+1.4)).^2 + (car_position_2_y - car1y).^2 - car_size^2,...
                               (car_position_1_x - (car2x-1.4)).^2 + (car_position_1_y - car2y).^2 - car_size^2,...
                               (car_position_1_x - (car2x+1.4)).^2 + (car_position_1_y - car2y).^2 - car_size^2,...
                               (car_position_2_x - (car2x-1.4)).^2 + (car_position_2_y - car2y).^2 - car_size^2,...
                               (car_position_2_x - (car2x+1.4)).^2 + (car_position_2_y - car2y).^2 - car_size^2,...
                               (car_position_1_x - (car3x-1.4)).^2 + (car_position_1_y - car3y).^2 - car_size^2,...
                               (car_position_1_x - (car3x+1.4)).^2 + (car_position_1_y - car3y).^2 - car_size^2,...
                               (car_position_2_x - (car3x-1.4)).^2 + (car_position_2_y - car3y).^2 - car_size^2,...
                               (car_position_2_x - (car3x+1.4)).^2 + (car_position_2_y - car3y).^2 - car_size^2,...
                               (car_position_1_x - (car4x-1.4)).^2 + (car_position_1_y - car4y).^2 - car_size^2,...
                               (car_position_1_x - (car4x+1.4)).^2 + (car_position_1_y - car4y).^2 - car_size^2,...
                               (car_position_2_x - (car4x-1.4)).^2 + (car_position_2_y - car4y).^2 - car_size^2,...
                               (car_position_2_x - (car4x+1.4)).^2 + (car_position_2_y - car4y).^2 - car_size^2,...
                               (car_position_1_x - (car5x-1.4)).^2 + (car_position_1_y - car5y).^2 - car_size^2,...
                               (car_position_1_x - (car5x+1.4)).^2 + (car_position_1_y - car5y).^2 - car_size^2,...
                               (car_position_2_x - (car5x-1.4)).^2 + (car_position_2_y - car5y).^2 - car_size^2,...
                               (car_position_2_x - (car5x+1.4)).^2 + (car_position_2_y - car5y).^2 - car_size^2,...
                               (car_position_1_x - (car6x-1.4)).^2 + (car_position_1_y - car6y).^2 - car_size^2,...
                               (car_position_1_x - (car6x+1.4)).^2 + (car_position_1_y - car6y).^2 - car_size^2,...
                               (car_position_2_x - (car6x-1.4)).^2 + (car_position_2_y - car6y).^2 - car_size^2,...
                               (car_position_2_x - (car6x+1.4)).^2 + (car_position_2_y - car6y).^2 - car_size^2];%put obstacles here
end
