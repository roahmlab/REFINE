classdef highway_cruising_10_state_agent < RTD_agent_2D
    % Agent of full-size vehicle model with closed loop dynamics using the
    % controller proposed by REFINE. NOTE the vehicle agent is assumed to
    % be FWD in this script. 
    
    properties
        
        wheel_plot_data = {};
        wheel_color  = [255 255 255]/255;
        
        % integrator type, to allow for fixed time step integration
        integrator_type = 'ode45'; % choose 'ode45' or 'ode4' or 'ode113'
        integrator_time_discretization = 0.01; % for ode4
        desired_initial_condition = [0; 0; 0; 1; 0; 0; 1; 0; 0; 0];
        
        footprint_vertices_for_plotting = [-2.4,-1.5,-1.5 0 0.3     2    2   2.4 2.4 2  2   0.3 0 -1.5 -1.5 -2.4 -2.4;
                                           -0.5,-0.5 -1   -1 -0.5    -0.3 -1   -1  1  1 0.3  0.5 1 1   0.5 0.5 -0.5];
        
        m
        lf
        lr
        mu_bar
        Caf1
        Caf2
        Car1
        Car2
        Izz
        rw
        
        grav_const
        l
        %% Complicated LLC to cancel out dynamics; Front Wheel drive
        Ku
        Kh
        Kr
        kappaP_vx 
        kappaI_vx
        phiP_vx
        phiI_vx
        kappaP 
        kappaI
        phiP
        phiI

        Mu
        Mr

        
        Cus
        u_cri
        
        max_Fy_uncertainty
        max_Fx_uncertainty
        max_Fx_uncertainty_braking
    end
    
    methods
        %% constructor
        function A = highway_cruising_10_state_agent(varargin)
            % set up default superclass values
            name = 'highway_cruiser' ;
            
            default_footprint = [4.8 2.2]; 
            n_states = 10 ;
            n_inputs = 6 ; % ud vd rd dud dvd drd
            stopping_time = 50 ; %not used
            sensor_radius = 400 ;

            
            % create agent
            A@RTD_agent_2D('name',name,...
                'footprint',default_footprint,...
                'n_states',n_states,'n_inputs',n_inputs,...
                'stopping_time',stopping_time,'sensor_radius',sensor_radius,varargin{:}) ;
            
            load('my_const.mat')
            A.m  = m;
            A.lf = lf;
            A.lr = lr;
            A.l = lf+lr;
            A.grav_const = grav_const;
            A.Caf1 = Caf1;
            A.Caf2 = Caf2;
            A.Car1 = Car1;
            A.Car2 = Car2;
            A.mu_bar = mu_bar;
            A.Izz  = Izz;
            A.rw = rw;
            A.Mu = Mu;
            A.Mr = Mr;
            
            
            %% proposed controller to cancel out dynamics; Front Wheel drive
            A.Ku = Ku;
            A.Kh = Kh;
            A.Kr = Kr;
            A.kappaP_vx = kappaP_vx; % kappa_1,vx
            A.kappaI_vx = kappaI_vx; % kappa_2,vx
            A.phiP_vx = phiP_vx;   % phi_1,vx
            A.phiI_vx = phiI_vx;   % phi_2,vx
            A.kappaP = kappaP;  % kappa_1,r 
            A.kappaI = kappaI;    % kappa_2,r
            A.phiP = phiP;      % phi_1,r
            A.phiI = phiI;      % phi_2,r

            A.max_Fy_uncertainty = max_Fy_uncertainty;
            A.max_Fx_uncertainty = max_Fx_uncertainty;
            A.max_Fx_uncertainty_braking = max_Fx_uncertainty_braking;
            A.Cus = m * grav_const * (lr / (A.l * Caf1) - lf / (A.l * Car1));
            A.u_cri = vx_really_slow;
            
            
        end
        
        %% get agent info
        function agent_info = get_agent_info(A)
            % call superclass method
            agent_info = get_agent_info@RTD_agent_2D(A) ;
        end
        function plot(A)
            plot@RTD_agent_2D(A);
            A.plot_wheel_at_time(A.time(end))
        end
        function plot_at_time(A,t)
            plot_at_time@RTD_agent_2D(A,t);
            A.plot_wheel_at_time(t);
        end
        function plot_wheel_at_time(A,t)
            wheel_size = [0.7 0.4];
            wheel = make_box(wheel_size);
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
            delta_t = z_t(6);
            R_r = rotation_matrix_2D(h_t); 
            V_ft = R_r*A.footprint_vertices_for_plotting + repmat(p_t,1,size(A.footprint_vertices_for_plotting,2));
            R_f = rotation_matrix_2D(h_t+delta_t);
            V_all = R_r*wheel_vertices + repmat(p_t,1,size(wheel_vertices,2)) ;
            
            for i = 1:4
                if i == 3 || i == 4
                    wheel_vert = V_all(:,6*i-5:6*i-1);
                    wheel_center = repmat( 0.5*(max(wheel_vert,[],2)+min(wheel_vert,[],2)),[1,5]);
                    origion_vert = wheel_vert - wheel_center;
                    V_all(:,6*i-5:6*i-1) = R_f * origion_vert + wheel_center;
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
        function [delta, w_cmd, r_err, h_err, vx_err, desired_front_lat_force, desired_lon_force] = FL_LLC(A,t,z,T,U,Z)
            h = z(3);
            vx = z(4);
            vy = z(5);
            r = z(6);
%             w = z(7); % tire speed is no longer used
            r_err_sum = z(8);
            h_err_sum = z(9);
            vx_err_sum = z(10);
            
            
            % desired trajectory
            vxd =interp1(T,U(1,:),t,'linear'); 
            vxddot =interp1(T,U(4,:),t,'linear');
            rd = interp1(T,U(3,:),t,'linear');
            rddot =interp1(T,U(6,:),t,'linear');
            hd = interp1(T,Z(3,:),t,'linear');
            r_dot_des = A.Kr*(rd -r) + rddot + A.Kh*(hd -h);
            vx_dot_des = A.Ku*(vxd -vx) + vxddot;
            
            % lateral force computation for steering angle delta
            r_err = r - rd;
            h_err = h - hd;
            err_term = A.Kr* r_err + A.Kh*h_err;
            kappa = A.kappaP + A.kappaI* (r_err_sum + h_err_sum);
            phi = A.phiP + A.phiI*(r_err_sum + h_err_sum);
            tau_r = -(kappa*A.Mr + phi) *err_term;
            vf = vy + A.lf*r;
            vr = vy - A.lr*r;
            alphar = - vr / max(vx, A.u_cri*1.1); % modify denominator for numerical stability
            Fywr = A.Car1*alphar;
            desired_front_lat_force = (A.Izz*(r_dot_des + tau_r) + A.lr*Fywr)/A.lf;
            delta = desired_front_lat_force/A.Caf1 + vf/vx;

            

            % longitudina force computation for wheel speed w_cmd      
            kappa_vx = A.kappaP_vx + A.kappaI_vx*(vx_err_sum);
            phi_vx = A.phiP_vx + A.phiI_vx*(vx_err_sum);
            vx_err = vx - vxd;
            err_term = A.Ku*vx_err;
            tau_vx = -(kappa_vx*A.Mu + phi_vx) * err_term;
            desired_lon_force = (vx_dot_des + tau_vx - vy*r)*A.m;
            Cbf = A.m * A.grav_const * A.lr / A.l * A.mu_bar;
            if vxddot <= 0 
                w_cmd = (desired_lon_force/Cbf * vx + vx) / A.rw;
            else
                w_cmd = vx/A.rw/(1-desired_lon_force/Cbf);
            end
        end
        function [delta, w_cmd, vylo, rlo, vx_err] = Low_Spd_LLC(A,t,z,T,U,Z)
            vx = z(4);
            vx_err_sum = z(10);
            mr = A.lf/A.l *A.m;
            

            vxd =interp1(T,U(1,:),t,'linear'); 
            vxddot =interp1(T,U(4,:),t,'linear');
            rd = interp1(T,U(3,:),t,'linear');
            delta = rd *(A.l+A.Cus*vx^2/A.grav_const) / max(vx,0.01); % modify denominator for numerical stability
            rlo = delta*vx/(A.l+A.Cus*vx^2/A.grav_const);
            vylo = rlo*(A.lr - vx^2*mr/A.Car1);
            
            u_dot_des = A.Ku*(vxd -vx) + vxddot;   
            kappa_vx = A.kappaP_vx + A.kappaI_vx*(vx_err_sum);
            phi_vx = A.phiP_vx + A.phiI_vx*(vx_err_sum);
            vx_err = vx - vxd;
            err_term = A.Ku*vx_err;
            Mu_lo = A.max_Fx_uncertainty_braking / A.m;
            tau_vx = -(kappa_vx*Mu_lo + phi_vx) * err_term;
            desired_lon_force = (u_dot_des + tau_vx - vylo*rlo)*A.m;
            Cbf = A.m * A.grav_const * A.lr / A.l * A.mu_bar;
            if vxddot <= 0 
                w_cmd = (desired_lon_force/Cbf *  vx+ vx) / A.rw;
            else
                w_cmd = vx/A.rw/(1-desired_lon_force/Cbf);
            end
            
        end
        %% dynamics
        function [dzdt, Fxwf, Fywf, Fywr ,delta ,w_cmd, vy, r, Fxwr]= dynamics(A,t,z,T,U,Z) % extra output used
            
            h = z(3);
            vx = z(4);
            vy = z(5);
            r = z(6);

            if  vx > A.u_cri
                    [delta, w_cmd, cur_r_err, cur_h_err, cur_vx_err] = A.FL_LLC(t,z,T,U,Z);
            else 
                    [delta, w_cmd, vy, r, cur_vx_err] = A.Low_Spd_LLC(t,z,T,U,Z);
                     cur_r_err= 0;
                     cur_h_err = 0;
            end

            uddot = interp1(T,U(4,:),t,'linear');
            vf = vy + A.lf*r;
            vr = vy - A.lr*r;
            v_wf = rotmat(-delta)*[vx;vf];
            v_wr = [vx; vr];
            alphar = atan( v_wr(2) ./ max(vx, A.u_cri*1.1)); % modify denominator for numerical stability
            Fywr = -A.Car1*tanh(A.Car2*alphar);
            if uddot <= 0
                lambda_f = (A.rw * w_cmd - v_wf(1)) / max(v_wf(1), 0.01);
            else
                lambda_f = (A.rw * w_cmd - v_wf(1)) / max(A.rw * w_cmd, 0.01);
            end
            alphaf = atan( v_wf(2) ./ (v_wf(1) + sign(v_wf(1)+0.001)*0.01));
            Cbf = A.m * A.grav_const * A.lr / A.l * A.mu_bar;
            Fxwf = Cbf*lambda_f;
            Fxwr = 0;
            Fywf = -A.Caf1*tanh(A.Caf2*alphaf);
            
            dvx =  vy*r + (cos(delta)*Fxwf-sin(delta)*Fywf + Fxwr)/A.m;
            dvy = - vx*r + (sin(delta)*Fxwf+cos(delta)*Fywf + Fywr)/A.m ;
            torque =  A.lf*(sin(delta)*Fxwf+cos(delta)*Fywf) - A.lr * Fywr;
            dr = torque/A.Izz;
            
            
             if vx > A.u_cri
                dzdt = [vx*cos(h)-vy*sin(h);
                vx*sin(h)+vy*cos(h);
                r;
                dvx;
                dvy;
                dr;
                0;
                cur_r_err^2;
                cur_h_err^2;
                cur_vx_err^2];
             else 
                dzdt = [vx*cos(h)-vy*sin(h);
                vx*sin(h)+vy*cos(h);
                r;
                dvx;
                0;
                0;
                0;
                0;
                0;
                cur_vx_err^2];
             end
        
        
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
                case 'ode1'
                    dt = A.integrator_time_discretization ;
                    tout = tspan(1):dt:tspan(end) ;
                    zout = ode1(@(t,z) fun(t,z),tout,z0(:)) ;
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