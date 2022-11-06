%% description

% define reference, use binary search to find proper Ay to get to a proper lane change width; i.e 4m +- 0.05m
% above 28m/s there is a non-trivial phase offset of the v, and r achieved during simulation.
% therefore a different reference trajecotry is needed for that
close all; clear
warning('off','ALL')
load_const; plot_flag = 1;

x0 = 0;
y0 = 0;
h0 = 0;
u0_vec = 0.6:0.1:1.0;%above 28 ref is far far away from real, can come back to 0 in time
u0_gen = 0.5*(u0_vec(2)-u0_vec(1));
v0 = 0;
t0 = 0;
% au0 = 0;
r0 = 0;
t0_dt = 0.75;


Ay_vec= zeros(length(u0_vec),1);
num_Ay = 1;
sim_end_idx = 5;
r0v0_limit = zeros(length(u0_vec),1);
mode = 1; %mode 1 for  dir change, mode 2 for lane change
y_ideal = 0.2; %both dir change and lane change achieve 4 m of change.
% this means dir change needs a higher Ay

if mode == 1
%     t_f = tpk_dir +tbrk; %3+4  %have new adaptive braking time, not 4 sec anymore 
    end_idx = tpk_dir/t0_dt;
else
%     t_f = tpk +tbrk; %6+4
    end_idx = tpk/t0_dt;
end
r_value_vec = zeros(2,end_idx,num_Ay,length(u0_vec));%value of r at different time stage of a turn
v_value_vec = zeros(2,end_idx,num_Ay,length(u0_vec));%value of v at different time stage of a turn
for u0idx = 1:length(u0_vec)% starting speed
    u0 = u0_vec(u0idx)
    w0 = u0;
    %% automated from here
    A_go = highway_cruising_7_state_agent('use_rover_controller',false) ;
    
    z_0 = [x0;y0;h0;u0;v0;r0;w0] ;
    syms_flag = 0;
    done_flag = 0;
    

    Ay_upper =0.6*2;
    if mode == 2
        Ay_upper = Ay_upper*1.2;
    end
    Ay_lower= 0;
    
    while done_flag~= 1 %binary search
        Ay = mean([Ay_lower Ay_upper]);
        % if set to one will make t symbolic, for dynamics generation
        % syms t Au u0 %Ay av0 au0 v0 r0
        Au = u0; t= 0; scale_Ay_flag = 1; % enable lane change manuver by setting this to 1
        if mode == 1
            [T_go,U_go,Z_go] = gaussian_one_hump_parameterized_traj_with_brake(t0,Ay,Au,u0,t,syms_flag, scale_Ay_flag);
        else
            [T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(t0,Ay,Au,u0,t,syms_flag, scale_Ay_flag);
        end
        %% run iterative find vd rd exercise
        A_go.reset(z_0) ;
        % figure(2); hold on; axis auto
        if syms_flag
            %     ezplot(U_go(1,1), [0,tpk]);
            ezplot(Z_go(1,1), [0,tpk]);
            ezplot(U_go(2,1), [0,tpk]);
            ezplot(U_go(3,1), [0,tpk]);
            ezplot(U_ga(3,1), [0,tpk]);
            ezplot(U_ga(5,1), [0,tpk]);
            
            legend('int(vd)','vd','rd','vgaussian','rgaussian')
        else
            %     plot(T_go, U_go)
            %     plot(Z_go(1,:), Z_go(2,:))
        end
        if mode == 1
            A_go.move(tpk_dir,T_go,U_go,Z_go) ;
        else
            A_go.move(tpk,T_go,U_go,Z_go) ;
        end
        
        
        X = A_go.state';
        T = A_go.time';
        delta_y = X(end, 2);

        if true% abs(delta_y-y_ideal) <0.01 % found the right Ay
            done_flag = 1;
            
            Ay_vec(u0idx) = 0.6;Ay = 0.6;%TODO:DELETE this;
            del_y_arr = linspace(0,Ay,2*num_Ay+1);%    del_y_arr = -0.8:0.2:0; % look at the 4 bins to look at avtual v r values
            delta_y   = (del_y_arr(2)-del_y_arr(1));
            del_y_arr = del_y_arr(2:2:end);
            
            %             delyspd = 0.5*(del_y_arr(2) - del_y_arr(1));
            for Ay_idx = 1:num_Ay
                ratio_Ay = linspace(-1,1,sim_end_idx);
                for sim_idx = 1:sim_end_idx 
                    A_go.reset([0;0;0;u0;0;0;w0]);
                    Ay_sim = del_y_arr(Ay_idx) + ratio_Ay(sim_idx)*delta_y;% either plus or minus delta, it is repetitive i know
                    if mode == 1  
                        [T_go,U_go,Z_go] = gaussian_one_hump_parameterized_traj_with_brake(0,Ay_sim,Au,u0,t,syms_flag, scale_Ay_flag);
                    else
                        [T_go,U_go,Z_go] = gaussian_T_parameterized_traj_with_brake(0,Ay_sim,Au,u0,t,syms_flag, scale_Ay_flag);
                    end
                    if mode == 1
                        A_go.move(tpk_dir,T_go,U_go,Z_go) ;
                    else
                        A_go.move(tpk,T_go,U_go,Z_go) ;
                    end
                    
                    X = A_go.state';
                    T = A_go.time';
                    for t0_idx = 1:end_idx
                        %here don't need to input t0idx, just take the piece of time you need
                        t0_val = t0_dt*(t0_idx-1);
                        [~,t_idx_real] = min(abs(T-t0_val));
                        v_value_vec(sim_idx,t0_idx,Ay_idx, u0idx)  =  X(t_idx_real,5);%save v and r to find it's range
                        r_value_vec(sim_idx,t0_idx,Ay_idx, u0idx)  =  X(t_idx_real,6);
                    end
                end
            end
        elseif delta_y < y_ideal
            Ay_lower = Ay;
        elseif delta_y > y_ideal
            Ay_upper = Ay;
        end
        
        
        %get tire forces
        %[dzdt, Fxwf, Fywf, Fywr ,delta ,w_cmd, v, r, Fxwr] 
        [~, ~, ~, ~ ,delta ,w_cmd, ~, ~, ~] = cellfun(@(t,x)  A_go.dynamics(t,x.',T_go,U_go,Z_go), num2cell(T), num2cell(X,[2]),'uni',0);
        r0v0_limit(u0idx) = 0.05;%TODO:find more proper value for this%abs (0.3*max(cell2mat(Fyf))*u0/(1+A_go.lr)/A_go.Ca);%r0 v0 are static function of forces and orther things, so can find a closed
        %this is how much we can diviate from nominal v, r without too much force
        %% plot
        if plot_flag
            text_size = 10;
            figure(1);clf;
            subplot(4,1,1);hold on;axis equal
            A_go.plot();
            plot(Z_go(1,:), Z_go(2,:));xlabel('x');ylabel('y')
            set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18
            
            subplot(4,1,2);hold on;%axis equal
            plot(T_go,U_go(2,:));
            plot(T_go,U_go(3,:));
            plot(A_go.time,A_go.state(5,:));
            plot(A_go.time,A_go.state(6,:));
            legend('vd','rd','v','r');xlabel('t(s)');ylabel('m/s or rad/s')
            set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18
            
            subplot(4,1,3);hold on;%axis equal
            plot(T_go,U_go(1,:));
            plot(A_go.time,A_go.state(4,:));
            legend('ud','u');xlabel('t(s)');ylabel('m/s')
            set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18
            
%             subplot(4,1,4);hold on;%axis equal
%             plot( A_go.time,cell2mat(Fxf));
%             plot( A_go.time,cell2mat(Fyf));
%             legend('Fxf','Fyf'); xlabel('t(s)');ylabel('N')
%             set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18
%             drawnow
%             
            subplot(4,1,4);hold on;%axis equal
            yyaxis left;
            plot( A_go.time,cell2mat(delta));
            yyaxis right;
            plot( A_go.time,cell2mat(w_cmd));
            legend('\delta','w'); xlabel('t(s)');
            set(gca,'FontSize',text_size) % Creates an axes and sets its FontSize to 18
            drawnow
        end
    end
end
%% pose process to get r0v0 limit
%minmax t0 Ayidx u0idx
% r0 v0 should not require the tires to exert more than 30 percent of the peak force that was
% achieved during the bin. 
r0_limit_gen = zeros(size(r_value_vec,2),size(r_value_vec,3),size(r_value_vec,4));
v0_limit_gen = zeros(size(v_value_vec,2),size(v_value_vec,3),size(v_value_vec,4));
               % t0 idx                    %ay idx            u0_idx
r0_limit_c = permute(max(r_value_vec,[],1)+min(r_value_vec,[],1),[2 3 4 1])/2;
v0_limit_c = permute(max(v_value_vec,[],1)+min(v_value_vec,[],1),[2 3 4 1])/2;
for i = 1:size(r0_limit_gen,3) %populate 3d array with original 4d arrary data %num_t0    %num_Ay
r0_min =  r_value_vec(:,:,:,i)-repmat(r0v0_limit(i),[sim_end_idx, size(r0_limit_gen,1),size(r0_limit_gen,2)]);
r0_max =  r_value_vec(:,:,:,i)+repmat(r0v0_limit(i),[sim_end_idx, size(r0_limit_gen,1),size(r0_limit_gen,2)]);
r0_extreme = cat(1,r0_min,r0_max); 
r0_limit_gen(:,:,i) = squeeze(range(r0_extreme,1)/2);
v0_min =  v_value_vec(:,:,:,i)-repmat(r0v0_limit(i)/13,[sim_end_idx, size(v0_limit_gen,1),size(v0_limit_gen,2)]);
v0_max =  v_value_vec(:,:,:,i)+repmat(r0v0_limit(i)/13,[sim_end_idx, size(v0_limit_gen,1),size(v0_limit_gen,2)]);
v0_extreme = cat(1,v0_min,v0_max); 
v0_limit_gen(:,:,i) = squeeze(range(v0_extreme,1)/2);
end
if mode == 1
    file_name = "dir_change_Ay_info.mat";
else
    file_name = "lane_change_Ay_info.mat";
end
save(file_name,'Ay_vec','u0_vec','v_value_vec','r_value_vec','r0v0_limit','r0_limit_c','v0_limit_c','r0_limit_gen','v0_limit_gen','num_Ay','t0_dt','u0_gen');
%%

%% plotting viz r and v 

% load('lane_change_Ay_info.mat');
%in case you are still confused,let's plot it
for i = 1:size(r_value_vec,4)
    for j = 1: size(r_value_vec,3)
        figure(2);clf;
        subplot(2,1,1);cla;hold on;
        scatter(1:end_idx,r_value_vec(1,:,j,i),50,'r','Filled');%actual value that produce whatever r lower 
        scatter(1:end_idx,r_value_vec(end,:,j,i),50,'r','Filled');%upper
        scatter(1:end_idx,r_value_vec(1,:,j,i)-r0v0_limit(i),30,'b','Filled');% how much r0 can diviate to not make too much force.
        scatter(1:end_idx,r_value_vec(1,:,j,i)+r0v0_limit(i),30,'b','Filled');% 
        scatter(1:end_idx,r_value_vec(end,:,j,i)-r0v0_limit(i),30,'b','Filled');
        scatter(1:end_idx,r_value_vec(end,:,j,i)+r0v0_limit(i),30,'b','Filled');

        scatter(1:end_idx,r0_limit_c(:,j,i),20,'g','Filled');% look at if what we saved looks correct
        scatter(1:end_idx,r0_limit_c(:,j,i)' + r0_limit_gen(:,j,i)',10,'g','Filled');
        scatter(1:end_idx,r0_limit_c(:,j,i)' - r0_limit_gen(:,j,i)',10,'g','Filled');
        
        legend('r vec exp','','r value extreme','','','','c + g','','')
        subplot(2,1,2); cla;hold on;
        scatter(1:end_idx,v_value_vec(1,:,j,i),20,'r');
        scatter(1:end_idx,v_value_vec(end,:,j,i),20,'r');
        scatter(1:end_idx,v_value_vec(1,:,j,i)-r0v0_limit(i),20,'b');
        scatter(1:end_idx,v_value_vec(1,:,j,i)+r0v0_limit(i),20,'b');
        scatter(1:end_idx,v_value_vec(end,:,j,i)-r0v0_limit(i),20,'b');
        scatter(1:end_idx,v_value_vec(end,:,j,i)+r0v0_limit(i),20,'b');

        scatter(1:end_idx,v0_limit_c(:,j,i)'+ v0_limit_gen(:,j,i)',20,'g');
        scatter(1:end_idx,v0_limit_c(:,j,i)' - v0_limit_gen(:,j,i)',20,'g');
        drawnow
        pause(0.5);
    end
end
return
%% below code for finding a mapping between u and delta y, not important.
% evantually went with not the following mapping but different range for Ay
% for different bins
figure(2);
%Ay = 0.5
u= [3 5 8 10 12];
dely=[1.36 2.27 3.63 4.54 5.45]; %ITS LINEAR!!!!!
plot(u, dely)
figure(3);
%u = 10;
Ay = [0.1 0.3 0.5 1];
dely =[0.92 2.76 4.54 8.54];
plot(Ay,dely)
t=0;syms_flag = 0;
C=[];
d=[];
for u = 4:2:15
    u
    for Ay = 0:0.1:0.8
        C = [C;Ay u 1];
        [T_go,U_go,Z_go] = gaussian_parameterized_traj_with_brake(Ay,u,u,t,syms_flag,1);
        z_0 = [0;0;0;u;0;0] ;
        %          A_go.reset(z_0) ;
        %          A_go.move(T_go(end),T_go,U_go) ;
        del_y = Z_go(2,end);
        %        del_y = A_go.state(2,end);%
        d = [d;del_y];
    end
end
%%
valid_idx = abs(d) < 5;
% fun = @(x,xdata)x(1)*exp(x(2)*xdata);
% x = lsqlin(C(valid_idx,:),d(valid_idx),[],[])
%x =   [-6.9873; -0.5085 ;3.9522];
fitobject = fit([C(valid_idx,1),C(valid_idx,2)],d(valid_idx),'poly11')
%%
figure(3);clf;hold on;

scatter3(C(valid_idx,1),C(valid_idx,2),d(valid_idx));
Ay = -1:0.1:1;
u = 1:15;
[AY,U]= meshgrid(Ay, u);
Z=fitobject(AY, U);
surf(AY, U, Z);
%%


