%% MECH 4310 - Final Project
% Jonas Wagner
clear; close all;
addpath(genpath(pwd));
[filepath,~,~] = fileparts(mfilename('fullpath'));
fig_subfolder = strcat(filepath,filesep,'figs');
data_subfolder = strcat(filepath,filesep,'data');

%% CT-Model (currently using kinnematic model)
% bikeModel_CT(x,u,params)
nx = 3;
nu = 2;
sys_fun = @bikeModel_CT;
x0 = zeros(nx,1);

% constraints
% (max/min of U_set)
bounds.u_ub = [10; pi/3];
bounds.u_lb = [0; -pi/3];
bounds.u_dot_ub = [2; pi/10];
bounds.u_dot_lb = -[2;1].*bounds.u_dot_ub;

U_set = Polyhedron('lb',bounds.u_lb,'ub',bounds.u_ub); U_set.minHRep;
U_dot_set = Polyhedron('lb',bounds.u_dot_lb, 'ub',bounds.u_dot_ub); U_dot_set.minHRep;

% % Plot Constraint Sets
% figure
% hold on;
% plot(U_set);
% plot(U_dot_set,'color','b')
% xlabel('Velocity (m/s, m/s^2)')
% ylabel('Angle (rad, rad/s)')
% legend({'U','U_dot'})
% title('Input Constraints')
% saveas(gcf,strcat(fig_subfolder,filesep,'input_constraints.png'))

% %% Initial Simulation
% u0 = [10; 0];%deg2rad(5)];
% u_fun = @(t) u0 + [0; deg2rad(15*sin(t))];
% 
% [t,x] = ode45(@(t,x) sys_fun(x,u_fun(t)),tspan,x0);
% 
% plot_vehicleModel(t,x',u_fun,bounds)

%% Cost Map
% Cost Map is a grayscale image with the cost at each space pre-determined
% The perception center of the vehicle, between the back two wheels, 
% is located (from the bottom left corner) 75px to the right and 50px up 
% out of the total 150px by 150px costmap image.  
% The pixels are 0.4m and the whole cost map is 60m by 60m.


% load from cost-map png
costMap_filename = 'costmap1.png';
costMap = 1 - rescale(im2gray(imread(strcat(data_subfolder,filesep,costMap_filename))));


mapX = -30 + (0.2:0.4:60); % 60m/2 = 30m
mapY = -20 + (0.2:0.4:60); % 50px = 20m
[gridX,gridY] = ndgrid(mapX,mapY);
costMap_fun = griddedInterpolant(gridX,gridY,rot90(costMap,3));
surf(gridX,gridY,costMap_fun(gridX,gridY))
xlabel('x');
ylabel('y');
title('Cost Map Function Visualization')
saveas(gcf,strcat(fig_subfolder,filesep,'surf_',costMap_filename));

% loc2px = @(x,y) round([(x+30)/0.4; (y+20)/0.4]);
% px2cost = @(px) costMap(end-px(2),px(1));
costMap_fun = @(x,y) interp2(gridX,gridY,rot90(costMap,3),x,y,'linear');

%% MPC Controller Setup
dt_MPC = 1;
N = 15;

yalmip('clear'); clear('controller');
x_ = sdpvar(repmat(nx,1,N+1),ones(1,N+1));
u_ = sdpvar(repmat(nu,1,N),ones(1,N));
u_0 = sdpvar(nu,1);

constraints = []; objective = 0;
constraints = [constraints, U_dot_set.A*((u_{1}-u_0)/dt_MPC) <= U_dot_set.b];

for k = 1:N
    % objective = objective - x_{k}(2) + (x_{k}(1)-0).^2 + (x_{k}(3) - pi)^2;
    % objective = objective + costMap_fun(x_{k}(1),x_{k}(2));
    % objective = objective + x_{k}(1) + (x_{k}(3)-pi).^2;
    % objective = objective + 1e-2*(x_{k}(2) - 0).^2;

    constraints = [constraints, x_{k+1} == update_eq(x_{k},u_{k},dt_MPC,sys_fun)];
    constraints = [constraints, U_set.A*u_{k} <= U_set.b];
    if k > 1
        constraints = [constraints, ...
            U_dot_set.A*((u_{k}-u_{k-1})/dt_MPC) <= U_dot_set.b];
    end
end

% objective = objective - x_{N+1}(2);% + (x_{N+1}(3)-pi)^2; %<--- minimize these things...
% objective = objective + (x_{N+1}(1) - 0).^2;
objective = objective + costMap_fun(x_{N+1}(1),x_{N+1}(2));
% objective = objective + interp2(gridX,gridY,rot90(costMap,3),x_{N+1}(1),x_{N+1}(2),'linear');%costMap_fun(x_{k}(1),x_{k}(2));

% opts = sdpsettings('solver','ipopt');
opts = sdpsettings('solver','ipopt','debug',0,'verbose',1);
% opts = sdpsettings;
controller = optimizer(constraints, objective, opts, {x_{1},u_0}, [u_{1}]);


%% MPC controller testing
runTesting = false;

if runTesting

% sim settings
dt = 0.1; tf = 10;
tspan = 0:dt:tf;
x0 = zeros(nx,1);
u0 = zeros(nu,1);

timerVal = tic;
% Initial Conditions
X_{1} = x0; 
U_{1} = controller{x0,u0};
% Time Evolution
for k = 1:length(tspan)-1
    % Simulate Update Step
    [~,x_k] = ode45(@(t,x) sys_fun(x,U_{k}),tspan(k:k+1),X_{k});
    X_{k+1} = x_k(end,:)';
    % Next-timestep controller
    U_{k+1} = controller{X_{k+1},U_{k}};
    elapsedTime_{k} = toc(timerVal);
    fprintf('Finished k = %d... elapsed: t = %f\n',k,elapsedTime_{k});
end

x = [X_{:}]; u = [U_{:}];

testName = 'Max Y - Final (fmincon)';


plot_vehicleModel(tspan,x,u,bounds, ...
    filename=strcat(fig_subfolder,filesep,strrep(testName,' ','_')), ...
    title=strcat(testName));

end
