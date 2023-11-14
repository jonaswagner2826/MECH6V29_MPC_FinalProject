function out = states_bike(in)
    arguments
        in.theta_dot = 0;
        in.v_zeta = 0;
        in.v_nu = 0;
        in.x = 0;
        in.y = 0;
        in.theta = 0;
    end
    out = in;
end