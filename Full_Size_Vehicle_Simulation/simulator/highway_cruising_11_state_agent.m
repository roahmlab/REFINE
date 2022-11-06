classdef highway_cruising_11_state_agent < RTD_agent_2D
    % Class: highway_cruising_7_state_agent < RTD_agent_2D < agent
    %
    % Highway car closed loop dynamics. may have tracking error
    %
    % based on real car model
    % states: x y h u v r
    
    properties
        % state limits
        max_speed = 22; % m/s
        min_spd = 0.5;
        %         % state indices
        speed_index = 4;
        
        maxFx = 1e50; % need proper limits eventually
        maxFy = 1e50; %
        
        wheel_plot_data = {};
        wheel_color  = [130 130 130]/255;
        
        % integrator type, to allow for fixed time step integration
        integrator_type = 'ode45'; % choose 'ode45' or 'ode4' or 'ode113'
        integrator_time_discretization = 0.01; % for ode4
        desired_initial_condition = [0; 0; 0; 1; 0; 0; 1; 0; 0; 0;  0];
        
        footprint_vertices_for_plotting = [-2.4,-1.5,-1.5 0 0.3     2    2   2.4 2.4 2  2   0.3 0 -1.5 -1.5 -2.4 -2.4;
            -0.5,-0.5 -1   -1 -0.5    -0.3 -1   -1  1  1 0.3  0.5 1 1   0.5 0.5 -0.5];
        
        m
        lf
        lr
        Cf1
        Cf2
        Cr1
        Cr2
        Cw
        Caf1
        Caf2
        Car1
        Car2
        Izz
        C_delta
        
        Speed_Ctrl_or_Current_Ctrl = "spd" %Options are 'spd','cur'
        
        %% Complicated LLC to cancel out dynamics; Front Wheel drive
        Ku
        Kh
        Kr
        
        C_friction_2
        C_friction_1
        C_current
        
        max_Fyr_uncertainty
        max_Fx_uncertainty
        
        use_rover_controller = 0; %set this to 1 if you do not wish to use FL
        %set to 0 if you use FL
        rover_LLC; % need rover low level controller if use rover ctrl is true
        
        accumulate_r_err = [];
        accumulate_h_err = [];
    end
    
    methods
        %% constructor
        function A = highway_cruising_11_state_agent(varargin)
            % set up default superclass values
            name = 'highway_cruiser' ;
            
            default_footprint = [5.2 2.8];
            n_states = 11 ;
            n_inputs = 6 ; % ud vd rd dud dvd three ref and derivative of 2
            %for rover it is two
            stopping_time = 50 ;%not used
            sensor_radius = 30 ;
            %             LLC = turtlebot_PD_LLC ; % doesnt need a controller
            
            % create agent
            A@RTD_agent_2D('name',name,...
                'footprint',default_footprint,...
                'n_states',n_states,'n_inputs',n_inputs,...
                'stopping_time',stopping_time,'sensor_radius',sensor_radius,varargin{:}) ;
            
            %draw F1 without wheels
            load_const
            A.m  = m;
            A.lf = lf;
            A.lr = lr;
%             A.Cf1 = 2*C1*A.lr/(A.lr+A.lf) ;
%             A.Cr1 = 2*C1*A.lf/(A.lr+A.lf) ;
%             A.Cf2 = C2; A.Cr2 = C2;
            A.Caf1 = Caf1;
            A.Caf2 = Caf2;
            A.Car1 = Car1;
            A.Car2 = Car2;
            A.Cw   = Cw;
            A.Izz  = Izz;
            A.C_delta = C_delta;
            
            A.C_friction_1 = C_friction_1;
            A.C_friction_2 = C_friction_2;
            A.C_current    = C_current;
            %% Complicated LLC to cancel out dynamics; Front Wheel drive
            A.Ku = Ku;
            A.Kh = Kh;
            A.Kr = Kr;
            A.max_Fyr_uncertainty = max_Fyr_uncertainty;
            A.max_Fx_uncertainty = max_Fx_uncertainty;
%             A.Kw = Kw;
            
            if A.use_rover_controller
                A.rover_LLC = highway_rover_LLC();
            end
            
        end
        
        %% get agent info
        function agent_info = get_agent_info(A)
            % call superclass method
            agent_info = get_agent_info@RTD_agent_2D(A) ;
            %             agent_info.lane = A.lane;
            %             agent_info.lane_des = A.lane_des;
            
        end
        function plot(A)
            plot@RTD_agent_2D(A);
            A.plot_wheel_at_time(A.time(end))
        end
        function plot_at_time(A,t)
            plot_at_time@RTD_agent_2D(A,t);
            A.plot_wheel_at_time(t);
            %             z_t = match_trajectories(t,A.time,A.state) ;
            %             xlim([z_t(1)-20,z_t(1)+50]); ylim([-0.7, 12.7]);
        end
        function plot_wheel_at_time(A,t)
            wheel_size = [0.7 0.4];
            wheel = make_box(wheel_size);
            %rare , front
            wheel_position = [-2   -2  1.5 1.5
                -0.75 0.75 -0.75 0.75];
            wheel_vertices = [];
            for i = 1:4
                wheel_vertices = [wheel_vertices wheel+repmat(wheel_position(:,i),[1,5]) [NaN;NaN]];
            end
            % compute footprint for plot
            z_t = match_trajectories(t,A.time,A.state) ;
            p_t = z_t(A.position_indices) ;
            h_t = z_t(A.heading_index) ;
            delta_t = z_t(6);%%% fake here
            R_r = rotation_matrix_2D(h_t); %+ rotation_matrix_2D(delta_t)  ;
            V_ft = R_r*A.footprint_vertices_for_plotting + repmat(p_t,1,size(A.footprint_vertices_for_plotting,2));
            R_f = rotation_matrix_2D(h_t+delta_t);
            V_all = R_r*wheel_vertices + repmat(p_t,1,size(wheel_vertices,2)) ;
            %             V_front =
            %             V_fp = [V_rare V_front];
            
            for i = 1:4
                if i == 3 || i == 4
                    wheel_vert = V_all(:,6*i-5:6*i-1);
                    wheel_center = repmat( 0.5*(max(wheel_vert,[],2)+min(wheel_vert,[],2)),[1,5]);
                    origion_vert = wheel_vert - wheel_center;
                    V_all(:,6*i-5:6*i-1) = R_f * origion_vert + wheel_center;
                end
                %             fill(V_all(1,6*i-5:6*i-1),V_all(2,6*i-5:6*i-1),[80 80 80]/255)
                
            end
            if check_if_plot_is_available(A,'wheel_plot_data')
                for i = 1:4
                    A.plot_data.wheel_plot_data{i}.XData =  V_all(1,6*i-5:6*i-1) ;
                    A.plot_data.wheel_plot_data{i}.YData =  V_all(2,6*i-5:6*i-1);
                    uistack(A.plot_data.wheel_plot_data{i}, 'top')
                end
                
            else
                for i =1:4
                    h = fill( V_all(1,6*i-5:6*i-1), V_all(2,6*i-5:6*i-1),A.wheel_color) ;
                    A.plot_data.wheel_plot_data{i} = h ;
                    h.FaceAlpha = A.plot_footprint_opacity;
                    h.EdgeAlpha = A.plot_footprint_edge_opacity;
                end
            end
            if check_if_plot_is_available(A,'pretty_footprint')
                A.plot_data.pretty_footprint.Vertices = V_ft' ;
                uistack(A.plot_data.pretty_footprint, 'top')
            else
                % plot footprint
                fp_data = patch(V_ft(1,:),V_ft(2,:),A.plot_footprint_color,...
                    'EdgeColor',A.plot_footprint_edge_color,...
                    'FaceAlpha',A.plot_footprint_opacity,...
                    'EdgeAlpha',A.plot_footprint_edge_opacity) ;
                
                
                % save plot data
                A.plot_data.pretty_footprint = fp_data ;
            end
            
            % make arrow for plot
            %             V_arrow = R_t*A.arrow_vertices + repmat(p_t,1,3) ;
            
            % plot
            %             if check_if_plot_is_available(A,'footprint')
            %                 A.plot_data.footprint.Vertices = V_fp' ;
            %                 A.plot_data.arrow.Vertices = V_arrow' ;
            %             else
            % plot footprint
            %                 fp_data = patch(V_fp(1,:),V_fp(2,:),A.plot_footprint_color,...
            %                     'EdgeColor',A.plot_footprint_edge_color,...
            %                     'FaceAlpha',A.plot_footprint_opacity,...
            %                     'EdgeAlpha',A.plot_footprint_edge_opacity) ;
            %
            %                 % plot arrow on footprint
            %                 arrow_data = patch(V_arrow(1,:),V_arrow(2,:),A.plot_arrow_color,...
            %                     'EdgeColor',A.plot_arrow_color,...
            %                     'FaceAlpha',A.plot_arrow_opacity,...
            %                     'EdgeAlpha',A.plot_arrow_opacity) ;
            %
            % save plot data
            %                 A.plot_data.footprint = fp_data ;
            %                 A.plot_data.arrow = arrow_data ;
            %             end
            
            %             if A.plot_trajectory_at_time_flag
            %                 % get the executed path up to the current time
            %                 X = A.state(A.position_indices,:) ;
            %                 T_log = A.time <= t ;
            %                 X = X(:,T_log) ;
            %
            %                 % plot it
            %                 if check_if_plot_is_available(A,'trajectory')
            %                     A.plot_data.trajectory.XData = X(1,:) ;
            %                     A.plot_data.trajectory.YData = X(2,:) ;
            %                 end
            %                     traj_data = plot_path(X,'g-','LineWidth',2) ;
            %                     A.plot_data.trajectory = traj_data ;
            %             end
        end
        function reset(A,state)
            if nargin < 2
                
                A.desired_time = zeros(1,0);
                A.desired_input = zeros(2,0);
                A.desired_trajectory =zeros(2,0);
                reset@RTD_agent_2D(A,[A.desired_initial_condition]) ;
            else
                reset@RTD_agent_2D(A,state) ;
            end
        end
        function [delta_cmd, w_cmd, r_err, h_err, u_err, desired_front_lat_force, desired_lon_force] = FL_LLC(A,t,z,T,U,Z)
            h = z(3);
            u = z(4);
            v = z(5);
            r = z(6);
            w = z(7);
            r_err_sum = z(8);
            h_err_sum = z(9);
            u_err_sum = z(10);
            cur_delta = z(11);
            
            
            
            ud =interp1(T,U(1,:),t,'linear'); % desired lon.velocity(u)
            uddot =interp1(T,U(4,:),t,'linear');
            
%             vd = interp1(T,U(2,:),t,'linear') ; % desired heading change,try gaussian
%             vddot =interp1(T,U(5,:),t,'linear')  ;
            
            rd = interp1(T,U(3,:),t,'linear') ; % desired heading change,try gaussian
            rddot =interp1(T,U(6,:),t,'linear')  ;

            hd = interp1(T,Z(3,:),t,'linear');
%             v_dot_des = A.Kv*(vd -v) + vddot + A.Kr *(rd -r);
            r_dot_des = A.Kr*(rd -r) + rddot + A.Kh*(hd -h);
            u_dot_des = A.Ku*(ud -u) + uddot;
            
            
            r_err = r - rd;
            h_err = h - hd;
            err_term = A.Kr* r_err + A.Kh*h_err;
            
            d = A.lr*(A.max_Fyr_uncertainty)/A.Izz;
            
            kappaP = 0.5; kappaI = 1;
            phiP = 4; phiI = 1;
            
            kappa = kappaP + kappaI* (r_err_sum + h_err_sum);
            phi = phiP + phiI*(r_err_sum + h_err_sum);
            
            V = -(kappa*d + phi) *err_term;
            
            vf = v + A.lf*r;
            vr = v - A.lr*r;
            
            uv_wr = [u; vr];
            %             betar = (w - uv_wr(1)) ./ max(sqrt(uv_wr(1).^2 +0.05), sqrt(w.^2 +0.05));
            alphar = atan( uv_wr(2) ./ (uv_wr(1) + sign(uv_wr(1)+0.0001)*0.2));
            
            %             Fxwr = A.Cr1*tanh(A.Cr2*betar);
            Fywr = -A.Car1*tanh(A.Car2*alphar);% here 8 is the uncertainty
            
            delta_arr = linspace(-0.45,0.45);
            front_lat_force =  zeros(size(delta_arr));
            
            for delta_idx = 1:length(delta_arr)
                delta = delta_arr(delta_idx);
                
                uv_wf = rotmat(-delta)*[u;vf];
                
%                 betaf = (w - uv_wf(1)) ./ max(sqrt(uv_wf(1).^2 +0.05), sqrt(w.^2 +0.05));
                
                alphaf = atan( uv_wf(2) ./ (uv_wf(1) + sign(uv_wf(1)+0.0001)*0.2));
                
                Fxwf = 0; %A.Cf1*tanh(A.Cf2*betaf);
                
                Fywf = -A.Caf1*tanh(A.Caf2*alphaf);
                
                %                 sin_delta_Fxwf = [sin_delta_Fxwf; sin(delta)*Fxwf];
                %                 cos_delta_Fywf = [cos_delta_Fywf; cos(delta)*Fywf];
                front_lat_force(delta_idx) = sin(delta)*Fxwf + cos(delta)*Fywf;
            end
            
              desired_front_lat_force = (A.Izz*(r_dot_des + V) + A.lr*Fywr)/A.lf;
%             

            if ud > 0.2
                if desired_front_lat_force < front_lat_force(1) ||  desired_front_lat_force > front_lat_force(end)
                     [~,delta_cmd_idx ] = min(abs(front_lat_force-desired_front_lat_force));
                    delta_des = delta_arr(delta_cmd_idx);
    %                 warning('delta out of bounds') 
                else
                    delta_des = interp1(front_lat_force,delta_arr,desired_front_lat_force); 
                end
            else
                delta_des = 0;
            end
            
            delta_tau = 0.02; %the time it takes to achieve delta desired
            delta_cmd = (delta_des - exp(-A.C_delta *delta_tau)*cur_delta)/ (delta_tau * exp(-A.C_delta *delta_tau)*A.C_delta);
            
            %recalculate from selected delta
            uv_wf = rotmat(-cur_delta)*[u;vf];                
            alphaf = atan( uv_wf(2) ./ (uv_wf(1) + sign(uv_wf(1)+0.0001)*0.2));
            Fywf = -A.Caf1*tanh(A.Caf2*alphaf);
%             w_cmd = A.Kw*(w_hat - w) + w ;
            ratio_term = A.m*(cos(cur_delta)*(A.lr/(A.lf+A.lr)) + A.lf/(A.lf+A.lr));
            
            kappaPU = 1.3; kappaIU = 0.7;
            phiPU = 1.3; phiIU = 0.7;

            kappaU = kappaPU + kappaIU*(u_err_sum);
            phiU = phiPU + phiIU*(u_err_sum);
            u_err = u - ud;
            err_term = A.Ku*u_err;
            
            d_u = A.max_Fx_uncertainty/A.m;
            U = -(kappaU*d_u + phiU) * err_term;
            
            desired_lon_force = ((u_dot_des + U - v*r)*A.m + sin(cur_delta)*Fywf)/(ratio_term);
            if A.Speed_Ctrl_or_Current_Ctrl == "spd";%%
                w_cmd = desired_lon_force/A.Cw + w;
            else A.Speed_Ctrl_or_Current_Ctrl == "cur"; %w_cmd here doubles as a electric current value in amp
                desired_lon_force = desired_lon_force + A.C_friction_1 * tanh(A.C_friction_2*w);
                w_cmd = desired_lon_force / A.C_current;
            end
%             if w_cmd < 0.5
%                 a = 1;
%             end
            %             if idx_lat == 1 || idx_lat == 100
            %                 warning ('optimal delta out of bounds')
            %             end
            % set up new variables now knowing delta
%             uv_wf = rotmat(-delta)*[u;vf];
%             alphaf = atan( uv_wf(2) ./ (uv_wf(1) + sign(uv_wf(1))*0.02));
%             Fywf = -A.Caf1*tanh(A.Caf2*alphaf);
            
            
            %% potentially have a w that you want, get to that w by commanding wd and use its natural dyanmics
%             w_proposals = linspace(w-0.1, w+0.1);
%             long_force = zeros(size(w_proposals));
%             for w_idx = 1:length(w_proposals)
%                 w_prop = w_proposals(w_idx);
%                 betar = (w_prop - uv_wr(1)) ./ max(sqrt(uv_wr(1).^2 +0.05), sqrt(w_prop.^2 +0.05));
%                 betaf = (w_prop - uv_wf(1)) ./ max(sqrt(uv_wf(1).^2 +0.05), sqrt(w_prop.^2 +0.05));
%                 
%                 Fxwf = A.Cf1*tanh(A.Cf2*betaf);
%                 Fxwr = A.Cr1*tanh(A.Cr2*betar);
%                 
%                 long_force(w_idx) = cos(delta)*Fxwf + Fxwr;
%                 %     Fywr = -Car1*tanh(Car2*alphar);
%             end
%             desired_long_force = A.m*(u_dot_des- v*r) + sin(delta)*Fywf;
% 
%             if length(unique(long_force))<100 || desired_long_force < long_force(1) ||  desired_long_force > long_force(end)
%                 [~,w_cmd_idx ] = min(abs(long_force-desired_long_force));
%                 w_hat = w_proposals(w_cmd_idx);
% %                  warning("w out of bounds"+num2str(w_cmd)+"des long force = "+num2str(desired_long_force))
%             else
%                 w_hat = interp1(long_force,w_proposals,desired_long_force);
%             end
%                  w_cmd = A.Kw*(w_hat - w) + w ;
%             else% there is a issue with sometimes force is really big, unable to find a w_cmd, maybe due to ode45
%             end
            
%             w_cmd = w_cmd + (w_cmd - w);
            %             if idx_long == 1 || idx_long == 100
            %                 warning ('optimal w cmd out of bounds')
        end
        %% dynamics
        function [dzdt, Fxwf, Fywf, Fywr ,delta ,w_cmd, v, r, Fxwr]= dynamics(A,t,z,T,U,Z) % extra output used
            % to plot forces in later code, ode45 only uses first output
%             Fxwr = 0;
            %% Smart controller
            
            % Cr2 = Cf2;
            % Cwt  = model_consts(13);
            
            h = z(3);
            u = z(4);
            v = z(5);
            r = z(6);
            w = z(7);
            delta = z(11);
%             u = w;
            
            
            [delta_cmd, w_cmd, cur_r_err, cur_h_err, cur_u_err] = A.FL_LLC(t,z,T,U,Z);
%             w_cmd
%             [w_cmd, u]
            
            vf = v + A.lf*r;
            vr = v - A.lr*r;
            
            uv_wf = rotmat(-delta)*[u;vf];
            uv_wr = [u; vr];
            
            alphar = atan( uv_wr(2) ./ (uv_wr(1) + sign(uv_wr(1)+0.0001)*0.2));
            Fywr = -A.Car1*tanh(A.Car2*alphar);
%             betaf = (w - uv_wf(1)) ./ max(sqrt(uv_wf(1).^2 +0.05), sqrt(w.^2 +0.05));
%             betar = (w - uv_wr(1)) ./ max(sqrt(uv_wr(1).^2 +0.05), sqrt(w.^2 +0.05));
            
            alphaf = atan( uv_wf(2) ./ (uv_wf(1) + sign(uv_wf(1)+0.0001)*0.2));
            
%             Fxwf = A.Cf1*tanh(A.Cf2*betaf);
%             Fxwr = A.Cr1*tanh(A.Cr2*betar);
            
            Fywf = -A.Caf1*tanh(A.Caf2*alphaf);
            if A.Speed_Ctrl_or_Current_Ctrl == "spd";
                Fxwf =  A.m*A.lr/(A.lf + A.lr) * A.Cw*(w_cmd - w);
                Fxwr =  A.m*A.lf/(A.lf + A.lr) * A.Cw*(w_cmd - w);
                dw = A.Cw*(w_cmd - w);
            else A.Speed_Ctrl_or_Current_Ctrl == "cur";
                Fxwf =  A.m*A.lr/(A.lf + A.lr) * (A.C_current*(w_cmd) - A.C_friction_1 * tanh(A.C_friction_2*w));
                Fxwr =  A.m*A.lf/(A.lf + A.lr) * (A.C_current*(w_cmd) - A.C_friction_1 * tanh(A.C_friction_2*w));
                dw = 0;
            end
            au =  v*r + (cos(delta)*Fxwf-sin(delta)*Fywf + Fxwr)/A.m;
            av = - u*r + (sin(delta)*Fxwf+cos(delta)*Fywf + Fywr)/A.m ;
            torque =  A.lf*(sin(delta)*Fxwf+cos(delta)*Fywf) - A.lr * Fywr;
            ar = torque/A.Izz;
            
            
            dzdt = [u*cos(h)-v*sin(h);
                u*sin(h)+v*cos(h);
                r;
                au;
                av;
                ar;
                dw;
                cur_r_err^2;
                cur_h_err^2;
                cur_u_err^2;
                -A.C_delta*(delta - delta_cmd)];
            
            if any(isnan(dzdt))
                error("nan appeared in dzdt = "+num2str(dzdt))
            end
            
        end
        
        %% integrator options
        function [tout,zout] = integrator(A,fun,tspan,z0)
            switch A.integrator_type
                case 'ode45'
                    [tout,zout] = ode45(@(t,z) fun(t,z),tspan,z0(:)) ;
                case 'ode113'
                    [tout,zout] = ode113(@(t,z) fun(t,z),tspan,z0(:)) ;
                case {'ode4','RK4'}
                    dt = A.integrator_time_discretization ;
                    tout = tspan(1):dt:tspan(end) ;
                    if tout(end) ~= tspan(end)
                        tout = [tout, tspan(end)] ;
                    end
                    zout = ode4(@(t,z) fun(t,z),tout,z0(:)) ;
                otherwise
                    error('Please set A.integrator_type to either ode45 or ode4')
            end
            tout = tout(:)' ;
            zout = zout' ;
        end
    end
end