function out = params_bike(in)
    arguments
        % Vehicle Geometry (L = 3.39 [m])
        in.L = 3.39; %[m]
        % in.m = 769; %[kg] (dry weight)
        % in.mu = 0.6; %<--- friction... likely better
        % in.c = 5000; %<--- tyre model... idk what value
    end 
    out = in;
    out.a = in.L/2; %<--- should improve
    out.b = in.L/2; %<--- should improve
    % out.I = 0.5*out.m*a^2 + 0.5*m*b^2; %<--- should fix
end