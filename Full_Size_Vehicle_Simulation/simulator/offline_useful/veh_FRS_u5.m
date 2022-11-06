clear, clc

%% lane change
% clear
for t0 = 0:0.75:6-0.75
    dim = 13;

    options.tensorParallel = 0;%turn this to 1 if want to use parallel not working very well. 
    % set options for reachability analysis:
    options.taylorTerms=15; % number of taylor terms for reachable sets
    options.zonotopeOrder= 100; % zonotope order... increase this for more complicated systems.
    options.maxError = 1e100*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
    options.verbose = 0;
    options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
    options.U = zonotope([0, 0]);
    options.advancedLinErrorComp = 0;
    % options.intermediateOrder = 20; %if options.advancedLinErrorComp = 1
    options.tensorOrder = 2;
    options.errorOrder = 50;
    options.reductionInterval = inf;
    options.reductionTechnique = 'girard';

    options.tStart = 0;
    load_const
    %in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
    %            1  2  3    4  5  6  7  8  9  10  11    12    13
    % vehicle parameters
    options.x0 = zeros(dim,1);
    frs_gen_starttic = tic;

    load lane_change_Ay_info
    u0 = 5;
    [~,Ay_idx ]=min(abs(u0_vec -u0)); 
    del_y_arr = linspace(0,Ay_vec(Ay_idx),9);%    del_y_arr = -0.8:0.2:0;
    del_y_arr = del_y_arr(2:2:end);
    delyspd = 0.5*(del_y_arr(2) - del_y_arr(1));
    r0v0_gen = Ay_vec(Ay_idx);
    for del_y_idx = 1:length(del_y_arr)
        del_y = del_y_arr(del_y_idx);
        
        u0, del_y
        options.x0 = zeros(dim,1);
        options.tFinal = tpk;
        options.timeStep = 0.01;
        options.x0(4) = u0;
        options.x0(7) = u0;
        options.x0(11) = u0; %Au = u0
        options.x0(10) = t0; %set t0 = 0.5;
        options.x0(12) = del_y;
        delu0 = zeros(dim,1);delu0(4) = 0.5; delu0(7) = 0.5; delu0(11) = 0.5;
        delAy = zeros(dim,1); delAy(12) = delyspd;
        delv0 = zeros(dim,1);delv0(5) =r0v0_gen; delv0(8) = r0v0_gen;
        delr0 = zeros(dim,1); delr0(6) = r0v0_gen;delr0(9) = r0v0_gen;
        options.R0 = zonotope( [options.x0, delu0,delAy,delv0,delr0]);
        tic;
        sys = nonlinearSys(dim, 1, @dyn_y_change_u5, options);
        [vehRS,Rt] = reach(sys, options);
        figure(2); clf; hold on; axis equal;
        vehRS_save = cell(0,0);
        for i = 1:10:length(vehRS)
            p_FRS = plot(vehRS{i}{1}, [1, 2], 'g');
            vehRS_save{end+1} = vehRS{i};
        end
        brake_idx = length(vehRS_save);
        brake_u   = center(project(vehRS_save{end}{1},4));
        gen4 = generators(deleteAligned(project(vehRS_save{end}{1},4))); % is empty most of the time
        if ~isempty(gen4) %get the range of braking speed here
            brake_gen = abs(gen4);
        else 
            brake_gen = 0;
        end
        
        %% first part of brake
        t_prev = zeros(dim,1); t_prev(end)=tpk;%assume it complete the whole time
%         options.R0 = vehRS_save{end}{1} - t_prev; %initial state for reachability analysis
        
        options.R0 = deleteAligned(deleteZeros(Rt{end}{1}.set))- t_prev;
        options.x0 = center(options.R0);
        
        % rotate R0
        th0 = options.x0(3);
        xy0 = options.x0(1:2);
        options.R0 = ROTxy(options.R0, -th0);
        options.x0 = center(options.R0); 
        
%         options.tFinal = ceil((tbrk - 5/(u0+1)*tbrk)*10)/10+0.1; % JL: slow down to 5 m/s
        options.tFinal = 1.7; % JL: constant acceleration, doesn't work
%         options.tFinal =tbrk-0.5;%maybe try adjust this dynamically for when it reaches the low speed. plot model differences.
%         options.verbose = 1;
        %         options.timeStep = 0.001;
        sys = nonlinearSys(dim, 1, @dyn_low_speed_brk, options );
        [vehBrk,Rt_brk] = reach(sys, options);
        
        % rotate everything back
        for i = 1:length(vehBrk)
            vehBrk{i}{1} = ROTxy(vehBrk{i}{1},th0,xy0);
            Rt_brk{i}{1}.set = ROTxy(Rt_brk{i}{1}.set,th0,xy0);
        end
        
        tComp = toc;
        disp(['computation time of reachable set: ', num2str(tComp)]);
        toc(frs_gen_starttic)
        %% plot reachable set
        
        for i = 1:10:length(vehBrk)
            if size(vehBrk{i},2)~= 1
                break
            end
            p_FRS = plot(vehBrk{i}{1}, [1, 2], 'r');
            %             p_FRS = plot(vehBrk{i}{2}, [1, 2], 'y');
            %             p_FRS.FaceAlpha = 0.02;
            %             p_FRS.EdgeAlpha = 0.4;
            vehRS_save{end+1} = vehBrk{i};
        end
        
        %% gather all FRSs and enclose adjacent FRSs (vehRS_save is re-generated)
        temp = [vehRS; vehBrk];
        vehRS_save = cell(0,0);
        ENCLOSE = 0;
        figure(10), clf, hold on, axis equal
        for i = 1:length(temp)
            if ENCLOSE == 0
                ENCLOSE = 1;
            else
                ENCLOSE = 0;
                vehRS_save{end+1} = deleteAligned(deleteZeros(enclose(temp{i-1}{1},temp{i-1}{1})));
                plot(vehRS_save{end},[1,2],'b');
            end
        end
        
        % check if length(temp) is odd
        if mod(length(temp),2)==1
            vehRS_save{end+1} = temp{end}{1};
        end
        save("C:\Users\Jinsun Liu\Desktop\work\Simon_RTD\RTD_Highway_Simulation\data\lane_change\lane_change_t0="+num2str(t0)+"_u="+num2str(u0)+"_Ay="+num2str(del_y_idx)+","+num2str(del_y)+".mat",'vehRS_save','brake_idx','brake_u','brake_gen');
    end

    
    
end

%% dir change
% clear
% t0_arr = 0:0.75:3-0.75;
% for t0_idx = 1:length(t0_arr)
%     t0= t0_arr(t0_idx);
%     dim = 13;
% 
%     options.tensorParallel = 0;%turn this to 1 if want to use parallel not working very well. 
%     % set options for reachability analysis:
%     options.taylorTerms=15; % number of taylor terms for reachable sets
%     options.zonotopeOrder= 100; % zonotope order... increase this for more complicated systems.
%     options.maxError =1e100*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
%     options.verbose = 0;
%     options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
%     options.U = zonotope([0, 0]);
%     options.advancedLinErrorComp = 0;
%     options.tensorOrder = 1;
%     options.errorOrder = 50;
%     options.reductionInterval = inf;
%     options.reductionTechnique = 'girard';
%     options.verbose = 0;
%     options.tStart = 0;
%     load_const
%     %in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
%     %            1  2  3    4  5  6  7  8  9  10  11    12    13
%     % vehicle parameters
%     options.x0 = zeros(dim,1);
%     frs_gen_starttic = tic;
% 
%     load dir_change_Ay_info
%     for u0 = 5 % set range of Ay or del_y allowed
%         [~,u0_idx ]=min(abs(u0_vec -u0)); 
%         del_y_arr = linspace(0,Ay_vec(u0_idx),9);%    del_y_arr = -0.8:0.2:0;
%         del_y_arr = del_y_arr(2:2:end);
%         delyspd = 0.5*(del_y_arr(2) - del_y_arr(1));
%         r0v0_gen = Ay_vec(u0_idx);
%         for del_y_idx = 1:length(del_y_arr)
%             del_y = del_y_arr(del_y_idx);
% 
%             u0, del_y
%             options.x0 = zeros(dim,1);
%             options.tFinal = tpk_dir;
%             options.timeStep = 0.01;
%             options.x0(4) = u0;
%             options.x0(7) = u0;
%             options.x0(11) = u0; %Au = u0
%             options.x0(10) = t0; %set t0 = 0.5;
%             options.x0(12) = del_y;
%             delu0 = zeros(dim,1);delu0(4) = 1; delu0(7) = 1; delu0(11) = 1;
%             delAy = zeros(dim,1); delAy(12) = delyspd;
%             delv0 = zeros(dim,1);delv0(5) =r0v0_gen; delv0(8) = r0v0_gen;
%             delr0 = zeros(dim,1); delr0(6) = r0v0_gen;delr0(9) = r0v0_gen;
%             options.R0 = zonotope( [options.x0, delu0,delAy,delv0,delr0]);
%             tic;
%             sys = nonlinearSys(dim, 1, @dyn_dir_change_u5, options);
%             [vehRS,Rt] = reach(sys, options);
%             figure(1); clf; hold on; axis equal;
%             vehRS_save = cell(0,0);
%             for i = 1:10:length(vehRS)
%                 p_FRS = plot(vehRS{i}{1}, [1, 2], 'g');
%                 %             p_FRS.FaceAlpha = 0.02;
%                 %             p_FRS.EdgeAlpha = 0.4;
%                 vehRS_save{end+1} = vehRS{i};
%             end
%             %% save brake idx
%         brake_idx = length(vehRS_save);
%         brake_u   = center(project(vehRS_save{end}{1},4));
%         gen4 = generators(deleteAligned(project(vehRS_save{end}{1},4))); % is empty most of the time
%         if ~isempty(gen4) %get the range of braking speed here
%             brake_gen = abs(gen4);
%         else 
%             brake_gen = 0;
%         end
%         %% first part of brake
%         t_prev = zeros(dim,1); t_prev(end)=tpk_dir;%assume it complete the whole time
% %         options.R0 = vehRS_save{end}{1} - t_prev; %initial state for reachability analysis
%         
%         options.R0 = deleteAligned(deleteZeros(Rt{end}{1}.set))- t_prev; 
%         options.x0 = center(options.R0);
%         
%         % rotate R0
%         th0 = options.x0(3);
%         xy0 = options.x0(1:2);
%         options.R0 = ROTxy(options.R0, -th0);
%         options.x0 = center(options.R0); 
%         
%         temp = interval(options.R0);
%         temp = supremum(temp(4));
%         options.tFinal = 1.7; % ceil((tbrk - 5/(temp+1)*tbrk)*10)/10+0.1; % JL: slow down to 5 m/s
% %         options.tFinal =tbrk;%maybe try adjust this dynamically for when it reaches the low speed. plot model differences.
% %         
% 
% %         sys = nonlinearSys(dim, 1, @dyn_dir_brake, options );
%         sys = nonlinearSys(dim, 1, @dyn_low_speed_brk, options );
%         [vehBrk,Rt_brk] = reach(sys, options);
%         
%         % rotate everything back
%         for i = 1:length(vehBrk)
%             vehBrk{i}{1} = ROTxy(vehBrk{i}{1},th0,xy0);
%             Rt_brk{i}{1}.set = ROTxy(Rt_brk{i}{1}.set,th0,xy0);
%         end
%         
%         tComp = toc;
%         disp(['computation time of reachable set: ', num2str(tComp)]);
%         toc(frs_gen_starttic)
%         %% plot reachable set
%         
%         for i = 1:50:length(vehBrk)
%             if size(vehBrk{i},2)~= 1
%                 break
%             end
%             p_FRS = plot(vehBrk{i}{1}, [1, 2], 'r');
%             %             p_FRS = plot(vehBrk{i}{2}, [1, 2], 'y');
%             %             p_FRS.FaceAlpha = 0.02;
%             %             p_FRS.EdgeAlpha = 0.4;
%             vehRS_save{end+1} = vehBrk{i};
%         end
%         
%         %% gather all FRSs and enclose adjacent FRSs (vehRS_save is re-generated)
%         temp = [vehRS; vehBrk];
%         vehRS_save = cell(0,0);
%         ENCLOSE = 0;
%         figure(10), clf, hold on, axis equal
%         for i = 1:length(temp)
%             if ENCLOSE == 0
%                 ENCLOSE = 1;
%             else
%                 ENCLOSE = 0;
%                 vehRS_save{end+1} = deleteAligned(deleteZeros(enclose(temp{i-1}{1},temp{i-1}{1})));
%                 plot(vehRS_save{end},[1,2],'b');
%             end
%         end
%         
%         % check if length(temp) is odd
%         if mod(length(temp),2)==1
%             vehRS_save{end+1} = temp{end}{1};
%         end
%         save("C:\Users\Jinsun Liu\Desktop\work\Simon_RTD\RTD_Highway_Simulation\data\dir_change\dir_change_t0="+num2str(t0)+"_u="+num2str(u0)+"_Ay="+num2str(del_y_idx)+","+num2str(del_y)+".mat",'vehRS_save','brake_idx','brake_u','brake_gen');
%     
%             
%         end
%     end
% end



%% spd change
clear;
dim = 13;

options.tensorParallel = 0;%turn this to 1 if want to use parallel not working very well. 
% set options for reachability analysis:
options.taylorTerms=15; % number of taylor terms for reachable sets
options.zonotopeOrder= 100; % zonotope order... increase this for more complicated systems.
options.maxError = 10000*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
options.verbose = 0;
options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
options.U = zonotope([0, 0]);
options.advancedLinErrorComp = 0;
options.tensorOrder = 1;
options.errorOrder = 50;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

options.tStart = 0;
load_const
%in2 states: x, y, psi, u, v, r,u0,v0, r0,t0 ,Au,   Ay,   t
%            1  2  3    4  5  6  7  8  9  10  11    12    13
% vehicle parameters
options.x0 = zeros(dim,1);
frs_gen_starttic = tic;

load lane_change_Ay_info
for u0 = 5%7:2:27 % set range of Ay or del_y allowed
    
    [~,Ay_idx ]=min(abs(u0_vec -u0)); 
    r0v0_gen = Ay_vec(Ay_idx);
    for Au = 5:2:27
        if abs(u0-Au) > 5 % range of avaliable commanded velocity
            continue;
        end
        
        
        u0, Au
        options.x0 = zeros(dim,1);
        options.tFinal = tpk_dir;
        options.timeStep = 0.01;
        options.x0(4) = u0;
        options.x0(7) = u0;
        options.x0(11) = Au;
        options.x0(10) = 0; 
        delu0 = zeros(dim,1);delu0(4) = 1; delu0(7) = 1; 
        delAu = zeros(dim,1);delAu(11) = 1;
        delv0 = zeros(dim,1);% delv0(5) =r0v0_gen; delv0(8) = r0v0_gen;
        delr0 = zeros(dim,1);% delr0(6) = r0v0_gen;delr0(9) = r0v0_gen;
        options.R0 = zonotope( [options.x0, delu0,delAu,delv0,delr0]);
        tic;
        sys = nonlinearSys(dim, 1, @dyn_u_change_u5, options);
        [vehRS,Rt] = reach(sys, options);
        figure(1); clf; hold on; axis equal;
        vehRS_save = cell(0,0);
        for i = 1:10:length(vehRS)
            p_FRS = plot(vehRS{i}{1}, [1, 2], 'g');
            %             p_FRS.FaceAlpha = 0.02;
            %             p_FRS.EdgeAlpha = 0.4;
            vehRS_save{end+1} = vehRS{i};
        end
        %% save brake idx
        brake_idx = length(vehRS_save);
        brake_u   = center(project(vehRS_save{end}{1},4));
        gen4 = generators(deleteAligned(project(vehRS_save{end}{1},4))); % is empty most of the time
        if ~isempty(gen4) %get the range of braking speed here
            brake_gen = abs(gen4);
        else 
            brake_gen = 0;
        end
        %% brake
        t_prev = zeros(dim,1); t_prev(end)=tpk_dir;%assume it complete the whole time
        options.R0 = deleteAligned(deleteZeros(Rt{end}{1}.set))- t_prev;
        options.x0 = center(options.R0);
        options.tFinal = 1.7; % if Au = 5, break a little longer using Sean's model
        if Au ~=5 % perform the first part of brake
            % do not need to rotate R0 since speed change doesn't have
            % heading change
            
            temp = interval(options.R0);
            temp = supremum(temp(4));
            options.tFinal = ceil((tbrk - 5/(temp+1)*tbrk)*10)/10+0.1; % JL: slow down to 5 m/s
            sys = nonlinearSys(dim, 1, @dyn_u_brake, options );
            [vehBrk,Rt_brk] = reach(sys, options);
            tComp = toc;
            disp(['computation time of reachable set: ', num2str(tComp)]);
            toc(frs_gen_starttic)
            
            for i = 1:10:length(vehBrk)
                if size(vehBrk{i},2)~= 1
                    break
                end
                p_FRS = plot(vehBrk{i}{1}, [1, 2], 'r');
                %             p_FRS = plot(vehBrk{i}{2}, [1, 2], 'y');
                %             p_FRS.FaceAlpha = 0.02;
                %             p_FRS.EdgeAlpha = 0.4;
                vehRS_save{end+1} = vehBrk{i};
            end
            
            % assign ic for low speed model
            options.tFinal = 1.5; 
            for i = 1:length(Rt_brk)
                frs = interval(Rt_brk{i}{1}.set);
                if supremum(frs(4))<=5
                    frs_c = mid(frs);
                    t_prev = zeros(dim,1); t_prev(end)=frs_c(end);
                    options.R0 = deleteAligned(deleteZeros(Rt_brk{i}{1}.set)) - t_prev;
                    options.x0 = center(options.R0);
                    vehBrk(i:end) = [];
                    break
                end
            end
            if isempty(options.x0)
                warning('first part of brake should perform longer...');
                frs_c = mid(frs);
                t_prev = zeros(dim,1); t_prev(end)=frs_c(end);
                options.R0 = deleteAligned(deleteZeros(Rt_brk{end}{1}.set)) - t_prev;
                options.x0 = center(options.R0);
            end
        end
        
        % Sean's low speed model
        sys = nonlinearSys(dim, 1, @dyn_low_speed_brk, options );
        [vehBrk_low,Rt_brk_low] = reach(sys, options);
        tComp = toc;
        disp(['computation time of reachable set: ', num2str(tComp)]);
        toc(frs_gen_starttic)
        for i = 1:10:length(vehBrk_low)
            if size(vehBrk_low{i},2)~= 1
                break
            end
            p_FRS = plot(vehBrk_low{i}{1}, [1, 2], 'k');
            vehRS_save{end+1} = vehBrk_low{i};
        end
        
        %% gather all FRSs and enclose adjacent FRSs (vehRS_save is re-generated)
        if Au ~=5
            temp = [vehRS; vehBrk;vehBrk_low];
        else
            temp = [vehRS; vehBrk_low];
        end
        vehRS_save = cell(0,0);
        ENCLOSE = 0;
        figure(10), clf, hold on, axis equal
        for i = 1:length(temp)
            if ENCLOSE == 0
                ENCLOSE = 1;
            else
                ENCLOSE = 0;
                vehRS_save{end+1} = deleteAligned(deleteZeros(enclose(temp{i-1}{1},temp{i-1}{1})));
                plot(vehRS_save{end},[1,2],'b');
            end
        end
        
        % check if length(temp) is odd
        if mod(length(temp),2)==1
            vehRS_save{end+1} = temp{end}{1};
        end
        save("C:\Users\Jinsun Liu\Desktop\work\Simon_RTD\RTD_Highway_Simulation\data\spd_change\spd_change_u="+num2str(u0)+"_Au="+num2str(Au)+".mat",'vehRS_save','brake_idx','brake_u','brake_gen');
 
    end
end














