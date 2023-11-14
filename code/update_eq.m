function x_new = update_eq(x,u,dt,sys_fun)
    arguments
        x
        u
        dt
        sys_fun function_handle = @bikeModel_CT;
    end

    % RK4 - method
    % https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
    k1 = sys_fun(x,u);
    k2 = sys_fun(x+dt*k1/2,u);
    k3 = sys_fun(x+dt*k2/2,u);
    k4 = sys_fun(x+dt*k3,u);
    
    x_new = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end