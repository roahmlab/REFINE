% this script generates zonotope reachable sets for all desired trajectory
% from the family of direction change.
clear;close all
load dir_change_Ay_info
load my_const.mat

tm = tpk_dir;
t0_arr = t0_dt*((1:tm/t0_dt)-1);
for t0_idx = 1:length(t0_arr)
    t0= t0_arr(t0_idx);
    dim = 20;
    
    % set options for reachability analysis:
    options.tensorParallel = 0;%turn this to 1 if want to use parallel not working very well.
    options.taylorTerms=15; % number of taylor terms for reachable sets
    options.zonotopeOrder= 100; % zonotope order... increase this for more complicated systems.
    options.maxError = 1e100*ones(dim, 1); 
    options.verbose = 1;
    options.uTrans = 0; % we won't be using any inputs
    options.U = zonotope([0, 0]);
    options.advancedLinErrorComp = 0;
    options.tensorOrder = 2;
    options.errorOrder = 50;
    options.reductionInterval = inf;
    options.reductionTechnique = 'girard';
    options.tStart = 0;
    options.x0 = zeros(dim,1);

    for u0 = u0_vec(1:end)
        if isSim && u0 == u0_vec(1) % u0 = u0_vec(1) causes splitting, so start from u0_vec(2)
            continue
        elseif ~isSim && u0 < u0_vec(3) % avoid splitting
            continue
        end

        [~,u0_idx ]=min(abs(u0_vec -u0));
        del_y_arr = linspace(0,Ay_vec(u0_idx),2*num_Ay+1);
        delpy = del_y_arr(2) - del_y_arr(1);
        del_y_arr = del_y_arr(2:2:end);
        for del_y_idx = 1:length(del_y_arr)
            p_y = del_y_arr(del_y_idx);
            %% phase 1: driving maneuver - dir change
            [u0, p_y]
            options.x0 = zeros(dim,1);
            options.tFinal = tm-t0;
            options.timeStep = 0.01;
            options.x0(4) = u0;
            options.x0(5) = v0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(8) = v0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(6) = r0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(9) = r0_limit_c(t0_idx,del_y_idx,u0_idx);
            options.x0(7) = u0;
            options.x0(11) = u0; % recall p_u = u0 by definition of P
            options.x0(10) = t0; 
            options.x0(12) = p_y;
            options.x0(14) = 0; 
            
            if isSim
                delx0 = zeros(dim,1); delx0(1) = 0.2;
                dely0 = zeros(dim,1); dely0(2) = 0.1;
                delh0 = zeros(dim,1); delh0(3) = deg2rad(3.14);
            else
                delx0 = zeros(dim,1); delx0(1) = 0.046;
                dely0 = zeros(dim,1); dely0(2) = 0.062;
                delh0 = zeros(dim,1); delh0(3) = deg2rad(1.93);
            end
            delp_y = zeros(dim,1); delp_y(12) = delpy;
            delu0 = zeros(dim,1); delu0(4) = u0_gen; delu0(7) = u0_gen; delu0(11) = u0_gen; % recall p_u = u0 by definition of P
            delv0 = zeros(dim,1); delv0(5) = v0_limit_gen(t0_idx,del_y_idx,u0_idx); delv0(8) = v0_limit_gen(t0_idx,del_y_idx,u0_idx);
            delr0 = zeros(dim,1); delr0(6) = r0_limit_gen(t0_idx,del_y_idx,u0_idx); delr0(9) = r0_limit_gen(t0_idx,del_y_idx,u0_idx);
            delFy = zeros(dim,1); delFy(14) = max_Fy_uncertainty;
            delFx =  zeros(dim,1);  delFx(15) = max_Fx_uncertainty;
            options.R0 = zonotope( [options.x0, delx0,dely0,delh0,delu0, delp_y,delv0,delr0,delFy,delFx]);

            sys = nonlinearSys(dim, 1, @dyn_dir_change, options);

            [vehRS,Rt] = reach(sys, options);
            fprintf('driving maneuver finished\n')

            
            %% phase 2: contigency braking when u>u_really_slow = u_cri. Note: dhd = rd = 0

            temp = interval(Rt{end}{1}.set);
            u_scale = [infimum(temp(4)),supremum(temp(4))];
            brake_time = (u0 - u_really_slow )/amax;
            t_prev = zeros(dim,1); t_prev(end)=tm-t0;
            options.R0 = deleteZeros(Rt{end}{1}.set)- t_prev;
            Z = options.R0.Z;
            Z(13,1) = brake_time;
            options.R0 = zonotope(Z);
            options.x0 = center(options.R0);

            % rotate R0 and reduce model uncertainty during braking.
            h0 = options.x0(3);
            xy0 = options.x0(1:2);
            options.R0 = ROTxy(options.R0, -h0);
            options.x0 = center(options.R0);
            options.x0(19) = h0;
            GR0 = options.R0.generators;
            fyr_gen_idx = find(GR0(14, :) ~= 0);      
            g1 = GR0(:,fyr_gen_idx);
            GR0(:,fyr_gen_idx) = [];
            g1(14) = max_Fy_uncertainty_braking;
            GR0 = [g1 GR0];
            options.R0 = zonotope([options.x0, GR0]);
            GR0 = options.R0.generators;
            fx_gen_idx = find(GR0(15, :) ~= 0);      
            g1 = GR0(:,fx_gen_idx);
            GR0(:,fx_gen_idx) = [];
            g1(15) = max_Fx_uncertainty_braking;
            GR0 = [g1 GR0];
            options.R0 = zonotope([options.x0, GR0]);

            options.tFinal = (u_scale(2) - u_really_slow )/amax + 0.01; % make sure to trigger and pass the guard
            sys = nonlinearSys(dim, 1, @dyn_dir_brake, options );
            [vehBrk,Rt_brk] = reach(sys, options);
            fprintf('hi-speed contingency braking finished\n')


            % find the time that vehBrk first triggers the guard
            % and the time that vehBrk is completely pass the guard
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
            if idx_justpass == 1 % in case idx_just_pass - 1 = 0 s.t. vehRsBrk is empty
                idx_justpass = 2;
            end

            vehRsBrk = vehBrk(1:idx_justpass-1);

            % Now we have the indices such that the FRS just passes
            % the guard and completely passes the guard. This
            % means the system needs no more than T second to completely 
            % pass the guard where
            %         T = (idx_allpass - idx_justpass)*dt [sec]
            % Thus we can just propogate the system in phase 2 for T second 
            % from idx_just_pass, and then proporgate the whole set in the
            % low speed mode
            dt = options.timeStep;
            options.R0 = Rt_brk{idx_justpass-1}{1}.set - t_prev;
            options.x0 = center(options.R0);
            options.tFinal = (idx_allpass - idx_justpass)*dt;
            options.timeStep = (idx_allpass - idx_justpass)*dt;
            sys = nonlinearSys(dim, 1, @dyn_dir_brake, options);
            [vehBrk_aux,Rt_brk_aux] = reach(sys, options);
            fprintf('One step guard crossing finished\n')
            vehBrk_aux{1}{1} = vehBrk_aux{1}{1} + t_prev;
            Rt_brk_aux{1}{1}.set = Rt_brk_aux{1}{1}.set + t_prev;
            vehRsBrk = [vehRsBrk; vehBrk_aux];
            
            
            %% phase 3: contigency braking, low speed
            % we know treat vehBrk_aux as the outer approximation of states
            % that just hit the guard, and these states share the same time 
            % line for phase 3. Thus vehBrk_aux can be treated as the 
            % initial set of low speed model, so we can propagate forward. 
            options.timeStep = dt;
            options.R0 = vehBrk_aux{1}{1};
            options.x0 = center(options.R0);
            options.tFinal = 1 + (1-isSim);
            sys = nonlinearSys(dim, 1, @dyn_dir_slow, options);
            [vehBrk_low,Rt_brk_low] = reach(sys, options);
            fprintf('low-speed contingency braking finished\n')
            first_brake_section_length = length(vehRsBrk);
            vehRsBrk = [vehRsBrk; vehBrk_low];
            
            %% rotate everything back
            for i = 1:length(vehRsBrk)
                vehRsBrk{i}{1} = ROTxy(vehRsBrk{i}{1},h0,xy0);
            end
            %% enclose
            % we want to decrease the number of zonotopes so that online
            % planning could have less constraints to enforce
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

                temp{cnt}{1} = linear_regime_verification(temp{cnt}{1},'dir',isSim);
                z1 = temp{cnt}{1};
                if cnt == length(temp)
                    vehRS_save{end+1} = deleteAligned_noslice(deleteZeros(z1),slice_dim);
                    break
                end

                temp{cnt+1}{1} = linear_regime_verification(temp{cnt+1}{1},'dir',isSim);
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
                            error('No generator for a sliceable dimension');
                        else
                            error('More than one generator for a slice dimension');
                        end
                    end
                    g1 = G1(:,z1idx);
                    g2 = G2(:,z2idx);

                    G1(:,z1idx) = [];
                    G2(:,z2idx) = [];
                    G1 = [g1 G1];
                    G2 = [g2 G2];
                    z1 = zonotope([z1.center, G1]);
                    z2 = zonotope([z2.center, G2]);
                end
                
                z = enclose(z1,z2);
                G = z.generators;
                for i = 1:length(slice_dim)
                    idxes = find(G(slice_dim(i), :) ~= 0);
                    if length(idxes) ~= 1
                        if length(idxes) == 0 
                            error('No generator for a sliceable dimension');
                        else
                            error('More than one generator for a slice dimension');
                        end
                    end
                end
                intz = interval(z); intz = intz(1:2);
                range_z = supremum(intz) - infimum(intz);
                
                if all(range_z./range_ori <= 1.1)
                   vehRS_save{end+1} = deleteAligned_noslice(deleteZeros(z),slice_dim);
                   cnt = cnt + 2;
                else
                    vehRS_save{end+1} = deleteAligned_noslice(deleteZeros(z1),slice_dim);
                    cnt = cnt + 1;
                end
            end
            

            
            
            save("./FRSdata/dir_change_t0="+num2str(t0)+"_u0="+num2str(u0)+"_p_y="+num2str(del_y_idx)+","+num2str(p_y)+".mat",'vehRS_save','brake_idx1','brake_idx2');
        end
    end
end