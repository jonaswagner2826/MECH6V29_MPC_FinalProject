function dx = bikeModel_CT(x,u,params)
    arguments
        x (3,1)
        u (2,1)
        params.a = 1.7 %[m]
        params.b = 1.7 %[m]
    end
    
    % % States
    % X = x(1);
    % Y = x(2);
    % psi = x(3);
    % 
    % % Inputs
    % V = u(1);
    % delta = u(2);
    % % 
    % % Params
    % a = params.a;
    % b = params.b;

    % calcs
    beta = atan(params.b*tan(u(2))/(params.a+params.b));
    % beta = atan2(params.b*tan(u(2)),params.a+params.b);
    % series expansion:
    % beta = (b*delta)/(a+b) - (b^3*delta^3)/(3*(a+b)^3) + (b^5*delta^5)/(5*(a+b)^5); 
    


    dx(1) = u(1)*cos(x(3)+beta);
    dx(2) = u(1)*sin(x(3)+beta);
    dx(3) = (u(1)*cos(beta)*tan(u(2)))/(params.a+params.b); 
    
    dx = reshape(dx,[],1);
    
    


end
