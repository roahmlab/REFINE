classdef highway_cruising_6_state_agent < RTD_agent_2D
    % Class: highway_cruising_6_state_agent < RTD_agent_2D < agent
    %
    % Highway car closed loop dynamics. may have tracking error
    %
    % based on real car model
    % states: x y h u v r
    
    properties
        % state limits
        max_speed = 40; % m/s
        min_spd = 1;
        %         % state indices
        speed_index = 4;
        
        maxFx = 1e50; % need proper limits eventually
        maxFy = 1e50; %
        
        wheel_plot_data = {};
        wheel_color  = [130 130 130]/255;
        
        % integrator type, to allow for fixed time step integration
        integrator_type = 'ode45'; % choose 'ode45' or 'ode4' or 'ode113'
        integrator_time_discretization = 0.01; % for ode4
        desired_initial_condition = [0; 0; 0; 5; 0; 0];
        
        footprint_vertices_for_plotting = [-2.4,-1.5,-1.5 0 0.3     2    2   2.4 2.4 2  2   0.3 0 -1.5 -1.5 -2.4 -2.4;
            -0.5,-0.5 -1   -1 -0.5    -0.3 -1   -1  1  1 0.3  0.5 1 1   0.5 0.5 -0.5];
        
        m
        lf
        lr
        C1
        C2
        Caf1
        Caf2
        Car1
        Car2
        Izz
        
        %% Complicated LLC to cancel out dynamics; Front Wheel drive
        Ku
        Kv
        Kr
        Fyr_real =[]
        Fyr_old =[]
        Fyr_new =[]
        coeff=[]
        use_rover_controller = 1; %set this to 1 if you do not wish to use FL
        %set to 0 if you use FL
        rover_LLC; % need rover low level controller if use rover ctrl is true
        
    end
    
    methods
        %% constructor
        function A = highway_cruising_6_state_agent(varargin)
            % set up default superclass values
            name = 'highway_cruiser' ;
            
            default_footprint = [4.8 2];
            n_states = 6 ;
            n_inputs = 5 ; % ud vd rd dud dvd three ref and derivative of 2
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
            A.C1 = C1;
            A.C2 = C2;
            A.Caf1 = Caf1;
            A.Caf2 = Caf2;
            A.Car1 = Car1;
            A.Car2 = Car2;
            A.Izz  = Izz; 
            
            %% Complicated LLC to cancel out dynamics; Front Wheel drive
            A.Ku = Ku;
            A.Kv = Kv;
            A.Kr = Kr;
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
        function [delta, w] = FL_LLC(A,t,z,T,U,Z,theta_vf,Fywr)
            u = z(4);
            v = z(5);
            r = z(6);
            
            ud =interp1(T,U(1,:),t,'linear') ;% desired lon.velocity(u)
            uddot =interp1(T,U(4,:),t,'linear');
            
            vd = interp1(T,U(2,:),t,'linear') ; % desired heading change,try gaussian
            vddot =interp1(T,U(5,:),t,'linear')  ;
            
            rd = interp1(T,U(3,:),t,'linear') ; % desired heading change,try gaussian
            hd = interp1(T,Z(3,:),t,'linear') ; % desired heading change,try gaussian
            
            v_dot_des = A.Kv*(vd -v) + vddot + A.Kr *(rd -r);
            Fywf = (v_dot_des + u*r) * A.m - Fywr; %omit cos
            delta = atanh(Fywf/A.Caf1)/A.Caf2 + theta_vf;
            delta = bound_values(delta, 0.2);
            
            u_dot_des = A.Ku*(ud -u) + uddot;
            Fywf = -A.Caf1*tanh(A.Caf2*(theta_vf-delta));
            Fx = A.m* (u_dot_des -v*r) + sin(delta)*Fywf;
            beta = atanh(Fx/2 /A.C1)/A.C2;
            w = beta *(sqrt(u^2+0.05)) + u;
            
            
            
        end
        %% dynamics
        function [dzdt, Fxwf, Fywf, Fywr ,Fywr_n ,delta, v, r, Fxwr]= dynamics(A,t,z,T,U,Z) % extra output used
            % to plot forces in later code, ode45 only uses first output
            
            %% Smart controller
            Cf1 = 2*A.C1*A.lf/(A.lr+A.lf) ;
            Cr1 = 2*A.C1*A.lr/(A.lr+A.lf) ;
            Cf2 = A.C2; Cr2 = A.C2;
            % Cr2 = Cf2;
            % Cwt  = model_consts(13);
            
            h = z(3);
            u = z(4);
            v = z(5);
            r = z(6);
            
            vf = v + A.lf*r;
            vr = v - A.lr*r;
            
            uv_wr = [u; vr];
            alphar = atan( uv_wr(2) ./ (uv_wr(1) + sign(uv_wr(1))*0.02));
            Fywr = -A.Car1*tanh(A.Car2*alphar);
            Fywr_n = Fywr;
            
            theta_vf = atan(vf/u);
            
            [delta, w] = A.FL_LLC(t,z,T,U,Z,theta_vf,Fywr);
            
            uv_wf = rotmat(-delta)*[u;vf];
           

            betaf = (w - uv_wf(1)) ./ max(sqrt(uv_wf(1).^2 +0.05), sqrt(w.^2 +0.05));
            betar = (w - uv_wr(1)) ./ max(sqrt(uv_wr(1).^2 +0.05), sqrt(w.^2 +0.05));
            
            alphaf = atan( uv_wf(2) ./ (uv_wf(1) + sign(uv_wf(1))*0.02));
            
            Fxwf = Cf1*tanh(Cf2*betaf);
            Fxwr = Cr1*tanh(Cr2*betar);
            
            Fywf = -A.Caf1*tanh(A.Caf2*alphaf);
            
            au =   v*r + (cos(delta)*Fxwf-sin(delta)*Fywf + Fxwr)/A.m;
            av = - u*r + (sin(delta)*Fxwf+cos(delta)*Fywf + Fywr)/A.m ;
            torque =  A.lf*(sin(delta)*Fxwf+cos(delta)*Fywf) - A.lr * Fywr;
            ar = torque/A.Izz;
            
            dzdt = [u*cos(h)-v*sin(h);
                u*sin(h)+v*cos(h);
                r;
                au;
                av;
                ar];
            
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