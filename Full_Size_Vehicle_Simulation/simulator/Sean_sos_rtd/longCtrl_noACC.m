classdef longCtrl_noACC < handle
    %longCtrl Computes longitudinal acceleration command based on IDM model
    
    properties  
        set_speed = 20      % Set speed in m/s
        time_gap = 0.8      % Time gap in s
        dist_gap = 5        % Minimum gap in m
        max_acc = 0.3*9.8   % Maximum acceleration in m/s^2
        max_dcc = 0.3*9.8   % Maximum deceleration in m/s^2
        ax = 0;             % Longitudinal acceleration in m/s^2
        
        Vehicle             % Inhereted vehicle properties
    end
    
    methods
        function this = longCtrl_noACC(v) 
            %longCtrl Creates an instance of the class
            %   Input argument is vehicle object
            
            this.Vehicle = v;   
        end
        function update(this)
%             %update Computes vehicle longitudinal acceleration command
%             
%             r = this.Vehicle.Sensing.r_lead;
%             rr = -this.Vehicle.Sensing.rr_lead;
%             
%             v = this.Vehicle.States.Vx;
%             s = this.dist_gap + v*this.time_gap + v/2*rr/sqrt(this.max_acc*this.max_dcc);
%             acc = this.max_acc*(1-(v/this.set_speed)^4 - (s/r)^2);
%             
%             acc = max(acc,-this.max_dcc);
%             acc = min(acc,this.max_acc);
            
            this.ax = 0;
        end

    end
end

