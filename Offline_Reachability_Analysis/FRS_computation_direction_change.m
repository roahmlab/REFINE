clear;close all
load dir_change_Ay_info
load my_const.mat



t0_arr = t0_dt*((1:tpk_dir/t0_dt)-1);
for t0_idx = 1:length(t0_arr)
    t0= t0_arr(t0_idx);
    dim = 20;
    
    options.tensorParallel = 0;%turn this to 1 if want to use parallel not working very well.
    % set options for reachability analysis:
    options.taylorTerms=15; % number of taylor terms for reachable sets
    options.zonotopeOrder= 100; % zonotope order... increase this for more complicated systems.
    options.maxError = 1e100*ones(dim, 1); % our zonotopes shouldn't be "splitting", so this term doesn't matter for now
    options.verbose = 1;
    options.uTrans = 0; % we won't be using any inputs, as traj. params specify trajectories
    options.U = zonotope([0, 0]);
    options.advancedLinErrorComp = 0;
    % options.intermediateOrder = 20; if
    options.tensorOrder = 2;
    options.errorOrder = 50;
    options.reductionInterval = inf;
    options.reductionTechnique = 'girard';
    
    options.tStart = 0;
    % vehicle parameters
    options.x0 = zeros(dim,1);
    frs_gen_starttic = tic;

    for u0 = u0_vec(2:end)
        
        [~,u0_idx ]=min(abs(u0_vec -u0));
        del_y_arr = linspace(0,Ay_vec(u0_idx),2*num_Ay+1);%    del_y_arr = -0.8:0.2:0;
        delyspd = del_y_arr(2) - del_y_arr(1);
        del_y_arr = del_y_arr(2:2:end);
        %     delyspd = 0.3;
        %     r0v0_gen = 0.1;
        %     r0v0_gen = Ay_vec(u0_idx);
        %     del_y_arr = [0.3];
        for del_y_idx = 1:length(del_y_arr)
            p_y = del_y_arr(del_y_idx);
            %% phase 1: dir change
            u0, p_y
            options.x0 = zeros(dim,1);
            options.tFinal = tpk_dir-t0;
            options.timeStep = 0.01;
            options.x0(4) = u0;
            options.x0(5) = v0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(8) = v0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(6) = r0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(9) = r0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(7) = u0;
            options.x0(11) = u0; %p_u = u0
            options.x0(10) = t0; %set t0 = 0.5;
            options.x0(12) = p_y;
            options.x0(14) = 0; %fyr uncertainty center
            
            delx0 = zeros(dim,1);delx0(1) = 0.2;
            dely0 = zeros(dim,1);dely0(2) = 0.1;
            delh0 = zeros(dim,1);delh0(3) = deg2rad(3.14);
            % delAu = zeros(dim,1); delAu(12) = 0;
            delAy = zeros(dim,1); delAy(12) = delyspd;
            delu0 = zeros(dim,1); delu0(4) = u0_gen; delu0(7) = u0_gen; delu0(11) = u0_gen;
            %         delAu = zeros(dim,1); delAu(11) = 1; %this is ignored during lane change, but is helpful during braking
            delv0 = zeros(dim,1); delv0(5) = v0_limit_gen(t0_idx,del_y_idx,u0_idx); delv0(8) = v0_limit_gen(t0_idx,del_y_idx,u0_idx);
            delr0 = zeros(dim,1); delr0(6) = r0_limit_gen(t0_idx,del_y_idx,u0_idx); delr0(9) = r0_limit_gen(t0_idx,del_y_idx,u0_idx);
            %         delt0 = zeros(dim,1); delt0(12) = 0;
            delFyr = zeros(dim,1); delFyr(14) = max_Fyr_uncertainty;
            delFx =  zeros(dim,1);  delFx(15) = max_Fx_uncertainty;

            options.R0 = zonotope( [options.x0, delx0,dely0,delh0,delu0, delAy,delv0,delr0,delFyr,delFx]);
            tic;
            
            sys = nonlinearSys(dim, 1, @dyn_dir_change, options);
            
            
            [vehRS,Rt] = reach(sys, options);
            fprintf('maneuver finished\n')

            
            %% phase 2: braking until u starts appearing below u really slow
%             if u0 > u_really_slow
                
                temp = interval(Rt{end}{1}.set);
                u_scale = [infimum(temp(4)),supremum(temp(4))];
                if u0 == u0_vec(1)
                    brake_time = (u0 +  10*u0_gen - u_really_slow )/amax;% + tbrk2;
                else
                    brake_time = (u0 - u_really_slow )/amax;% + tbrk2;
                end

                t_prev = zeros(dim,1); t_prev(end)=tpk_dir-t0;%assume it complete the whole time
                options.R0 = deleteZeros(Rt{end}{1}.set)- t_prev;
                Z = options.R0.Z;
                Z(13,1) = brake_time;
                options.R0 = zonotope(Z);
                options.x0 = center(options.R0);

                % rotate R0
                th0 = options.x0(3);
                xy0 = options.x0(1:2);
                options.R0 = ROTxy(options.R0, -th0);
                options.x0 = center(options.R0);
                %change hd by change th0
                options.x0(19) = th0;

                %reduce uncertainty in Fyr estimation
                GR0 = options.R0.generators;
                fyr_gen_idx = find(GR0(14, :) ~= 0);      
                g1 = GR0(:,fyr_gen_idx);
                GR0(:,fyr_gen_idx) = [];
                g1(14) = max_Fyr_uncertainty_braking;
                %introduce smaller uncertainty here 
                GR0 = [g1 GR0];
                options.R0 = zonotope([options.x0, GR0]);
    %             
    %             %reduce uncertainty in Fx estimation
                GR0 = options.R0.generators;
                fx_gen_idx = find(GR0(15, :) ~= 0);      
                g1 = GR0(:,fx_gen_idx);
                GR0(:,fx_gen_idx) = [];
                g1(15) = max_Fx_uncertainty_braking;
                %introduce smaller uncertainty here 
                GR0 = [g1 GR0];
                options.R0 = zonotope([options.x0, GR0]);

                if u0 == u0_vec(1)
                    options.tFinal = (u_scale(2) + 10*u0_gen - u_really_slow )/amax+0.02; % make sure the umax pass u_really_slow as well
                else
                    options.tFinal = (u_scale(2) - u_really_slow )/amax; % make sure the umax pass u_really_slow as well
                end
                sys = nonlinearSys(dim, 1, @dyn_dir_brake, options );
                [vehBrk,Rt_brk] = reach(sys, options);
                fprintf('mid braking finished\n')


                % find the time that vehBrk first intersect with u_real_slow
                % and the time that vehBrk is completely below u_real_slow
                % ASSUMING u is guranteed to be greater than u_real_slow before
                % entering phase 2 by A LOT!!!!!!!
                idx_justpass = inf;
                idx_allpass = inf;
                for i = 1:length(vehBrk)
                    % shift time btw
                    vehBrk{i}{1} = vehBrk{i}{1} + t_prev;
                    Rt_brk{i}{1}.set = Rt_brk{i}{1}.set + t_prev;

                    temp = interval(vehBrk{i}{1});
                    if ~(idx_justpass<inf) && infimum(temp(4))<u_really_slow
                        idx_justpass = i;
                    end
                    if ~(idx_allpass<inf) && supremum(temp(4))<u_really_slow
                        idx_allpass = i;
                        break
                    end
                end
                if idx_justpass == 1%for problem regarding the FRS touches the boundary at the beginning
                    idx_justpass = 2;
                end
    %             %% CHEATING HERE, using last idx as just pass and all pass, so that cross guard is kept, dt is small
    %             idx_justpass = length(vehBrk)-1;
    %             idx_allpass = length(vehBrk);
                vehRsBrk = vehBrk(1:idx_justpass-1);
%             else
%                 vehRsBrk = [];
%             end
            

            %%
            % now we have the index that the FRS_inverval_version for just
            % passing the guard, and completely passing the guard. That
            % means the system needs no more than T second where
            %   T = (idx_allpass - idx_justpass)*dt [sec]
            % to finish running in phase 2. Thus we can just simulate the
            % system in phase 2 before just passing for T second, and
            % proporgating the whole set in phase 3.
            dt = options.timeStep;
            options.R0 = Rt_brk{idx_justpass-1}{1}.set - t_prev;
            options.x0 = center(options.R0);
            options.tFinal = (idx_allpass - idx_justpass)*dt;
            options.timeStep = (idx_allpass - idx_justpass)*dt;
            sys = nonlinearSys(dim, 1, @dyn_dir_brake, options);
            [vehBrk_aux,Rt_brk_aux] = reach(sys, options);
            fprintf('One step crossing braking finished\n')
            vehBrk_aux{1}{1} = vehBrk_aux{1}{1} + t_prev;
            Rt_brk_aux{1}{1}.set = Rt_brk_aux{1}{1}.set + t_prev;
            vehRsBrk = [vehRsBrk; vehBrk_aux];

            
            for i = 1:10:length(vehRsBrk)
                if size(vehRsBrk{i},2)~= 1
                    break
                end
            end
%             %%% JL test shit %%%%%%%%%%%%%%%%%%%%
%             zono1 = zonotope_slice_interval(Rt_brk{idx_justpass-1}{1}.set,7,u0-u0_gen,u0-0.8*u0_gen);
%             zono2 = zonotope_slice_interval(Rt_brk{idx_justpass-1}{1}.set,7,u0-0.8*u0_gen,u0+u0_gen);
%             dt = options.timeStep;
%             options.tFinal = (idx_allpass - idx_justpass)*dt;
%             options.timeStep = (idx_allpass - idx_justpass)*dt;
%             
%             options.R0 = zono1 - t_prev;
%             options.x0 = center(options.R0);
%             sys = nonlinearSys(dim, 1, @dyn_dir_brake, options);
%             [vehBrk_aux1,Rt_brk_aux1] = reach(sys, options);
%             vehBrk_aux1{1}{1} = vehBrk_aux1{1}{1} + t_prev;
%             
%             options.R0 = zono2 - t_prev;
%             options.x0 = center(options.R0);
%             sys = nonlinearSys(dim, 1, @dyn_dir_brake, options);
%             [vehBrk_aux2,Rt_brk_aux2] = reach(sys, options);
%             vehBrk_aux2{1}{1} = vehBrk_aux2{1}{1} + t_prev;
%             
%             figure
%             plot(vehBrk_aux1{1}{1},[1,2],'b'); hold on;
%             plot(vehBrk_aux2{1}{1},[1,2],'r')
%             plot(enclose(vehBrk_aux1{1}{1},vehBrk_aux2{1}{1}),[1,2],'g')
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            
            %% phase 3
            % we know treat vehBrk_aux as the outer approximation of states
            % that just hit the guard, meaning u = 0.5 precisely, and these
            % states share the same time line. Then vehBrk_aux can be
            % treated as the initial set of low speed model, so we can just
            % propagate forward. yay
            options.timeStep = dt;
            options.R0 = vehBrk_aux{1}{1};
            options.x0 = center(options.R0);
            options.tFinal = 1;
            sys = nonlinearSys(dim, 1, @dyn_dir_slow, options);
            [vehBrk_low,Rt_brk_low] = reach(sys, options);
            fprintf('low braking finished\n')
            %             % plotting code, only for testing %%%%%%%%%%%%%
            %             for i = 1:length(vehBrk_low)
            %                 if plot_flag
            %                     p_FRS = plot(ROTxy(vehBrk_low{i}{1},th0,xy0), [1, 2], 'k');
            %                 end
            %             end %%%%%%%%%%%%%%%%%%%%%%%%
            first_brake_section_length = length(vehRsBrk);
            vehRsBrk = [vehRsBrk; vehBrk_low];
            
            %% rotate everything back
            for i = 1:length(vehRsBrk)
                vehRsBrk{i}{1} = ROTxy(vehRsBrk{i}{1},th0,xy0);
            end
            %% enclose
            temp = [vehRS;vehRsBrk];
            vehRS_save = cell(0,0);
            slice_dim = [7,8,9,12];
            cnt = 1;
            brake_idx1 = [];
            brake_idx2 = [];
            while cnt < length(temp)
                if isempty(brake_idx1) && cnt > length(vehRS)
                    brake_idx1 = length(vehRS_save) + 1;
                end
                if isempty(brake_idx2) && cnt > length(vehRS)+first_brake_section_length
                    brake_idx2 = length(vehRS_save) + 1;
                end

                temp{cnt}{1} = linear_regime_verification(temp{cnt}{1},'dir');
                z1 = temp{cnt}{1};
                if cnt == length(temp)
                    vehRS_save{end+1} = deleteAligned(deleteZeros(z1),slice_dim);
                    break
                end

                temp{cnt+1}{1} = linear_regime_verification(temp{cnt+1}{1},'dir');
                z2 = temp{cnt+1}{1};
                int1 = interval(z1); int1 = int1(1:2);
                int2 = interval(z2); int2 = int2(1:2);
                mini = min([infimum(int1),infimum(int2)],[],2);
                maxi = min([supremum(int1),supremum(int2)],[],2);
                range_ori = maxi - mini;
                
                G1 = z1.generators;
                G2 = z2.generators;
                for i = 1:length(slice_dim)
                    z1idx = find(G1(slice_dim(i), :) ~= 0);
                    z2idx = find(G2(slice_dim(i), :) ~= 0);
                    if length(z1idx) ~= 1 || length(z2idx) ~= 1
                        if length(z1idx) == 0 || length(z2idx) == 0
                            error('No generator for slice index');
                        else
                            error('More than one generator for slice index');
                        end
                    end
%                     if z1idx ~= z2idx
                    g1 = G1(:,z1idx);
                    g2 = G2(:,z2idx);

                    G1(:,z1idx) = [];
                    G2(:,z2idx) = [];
                    G1 = [g1 G1];
                    G2 = [g2 G2];
                    z1 = zonotope([z1.center, G1]);
                    z2 = zonotope([z2.center, G2]);
%                     end
                end
                
                z = enclose(z1,z2);
                G = z.generators;
                for i = 1:length(slice_dim)
                    idxes = find(G(slice_dim(i), :) ~= 0);
                    if length(idxes) ~= 1
                        if length(idxes) == 0 
                            error('No generator for slice index');
                        else
                            error('More than one generator for slice index');
                        end
                    end
                end
                intz = interval(z); intz = intz(1:2);
                range_z = supremum(intz) - infimum(intz);
                
                if all(range_z./range_ori <= 1.1)
                   vehRS_save{end+1} = deleteAligned(deleteZeros(z),slice_dim);
                   cnt = cnt + 2;
                else
                    vehRS_save{end+1} = deleteAligned(deleteZeros(z1),slice_dim);
                    cnt = cnt + 1;
                end
            end
            

            
            
            save("./FRSdata/dir_change_t0="+num2str(t0)+"_u="+num2str(u0)+"_p_y="+num2str(del_y_idx)+","+num2str(p_y)+".mat",'vehRS_save','brake_idx1','brake_idx2');
        end
    end
end