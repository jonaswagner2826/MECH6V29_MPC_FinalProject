classdef bikeModel
    %bikeModel - Model for a vehicle using a bike model
    %   TODO: include more details

    properties
        % states = states_bike(); % []
        % inputs = inputs_bike(); % [V,\delta]
        % x (6,1) % [\dot{\theta}, V_zeta, V_nu, X, Y, \theta]^T (zeta = lat, nu = long)
        x (3,1) % [X, Y, \psi]
        u (2,1) % [V, \delta]
        params = params_bike();%<=== parameters struct
    end

    properties (Dependent)
        % params ----
        a
        b
        m
        I
        % states -----
        X
        Y
        psi
        % theta_dot
        % V_zeta
        % V_nu
        % X
        % Y
        % theta
        % inputs -------
        V
        delta
    end

    properties (Dependent)
        beta
    end

    % Contstructor
    methods
        function obj = bikeModel(x,u,params)
            %bikeModel constructor
            arguments
                x = zeros(6,1);%states_bike();
                u = zeros(2,1);%inputs_bike();
                params = params_bike();
            end
            obj.x = x;
            obj.u = u;
            obj.params = params;
        end
    end

    % Class Methods
    methods
        function dx = dx(obj)
            dx = zeros(3,1);
            dx(1) = obj.V*cos(obj.theta + obj.beta);
            dx(2) = obj.V*sin(obj.theta + obj.beta);
            dx(3) = obj.V*cos(obj.beta)*tan(obj.delta)/(obj.a+obj.b);
        end
    end

    % Getter/setter functions
    methods
        % Getter
        % params
        function out = get.a(obj); out = obj.params.a; end
        function out = get.b(obj); out = obj.params.b; end
        function out = get.m(obj); out = obj.params.m; end
        function out = get.I(obj); out = obj.params.I; end

        % states
        function out = get.X(obj); out = obj.x(1); end
        function out = get.Y(obj); out = obj.x(2); end
        function out = get.psi(obj); out = obj.x(3); end
        % function out = get.theta_dot(obj); out = obj.x(1); end
        % function out = get.V_zeta(obj); out = obj.x(2); end
        % function out = get.V_nu(obj); out = obj.x(3); end
        % function out = get.X(obj); out = obj.x(4); end
        % function out = get.Y(obj); out = obj.x(5); end
        % function out = get.theta(obj); out = obj.x(6); end

        % dependent states
        function beta = get.beta(obj)
            % beta = atan2(obj.V_zeta,obj.V_nu); 
            beta = atan2(obj.b*tan(obj.delta),obj.a+obj.b);
        end

        % inputs
        function out = get.V(obj); out = obj.u(1); end
        function out = get.delta(obj); out = obj.u(2); end


        % Setter
        function obj = set.a(obj,a); obj.params.a = a; end
        function obj = set.b(obj,b); obj.params.b = b; end
        function obj = set.m(obj,m); obj.params.m = m; end
        function obj = set.I(obj,I); obj.params.I = I; end





    end
end