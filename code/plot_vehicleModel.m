function plot_vehicleModel(t,x,u,bounds)

arguments
    t
    x
    u = [];
    bounds = [];
end

% Quiver Plot
X = x(1,:);
Y = x(2,:);
U = cos(x(3,:));
V = sin(x(3,:));

figure
quiver(X,Y,U,V,'off')
axis('equal')
xlabel('X [m]')
ylabel('Y [m]')


% Rest of the plots
if ~isempty(u)
    if ~isnumeric(u)
        try
            u = arrayfun(u,t,'UniformOutput',false);
        catch
            error('issue');
        end
        u = [u{:}];
    end

    stateNames = {"X","Y","\psi"};
    x(3,:) = rad2deg(x(3,:));

    figure
    tiledlayout('flow','TileSpacing','Compact')
    % subplot(2,2,[1 3])
    % quiver(X,Y,U,V,'off')
    for j = 1:size(x,1)
        nexttile(2*j-1)
        plot(t,x(j,:),'DisplayName',stateNames{j});%sprintf('x_%d',j));
        xlabel('Time [s]')
        legend
    end

    
    nexttile
    % subplot(2,2,2);
    hold on
    plot([t(1),t(end)],[bounds.u_lb(1),bounds.u_lb(1)],'k--','DisplayName','V_min')
    plot([t(1),t(end)],[bounds.u_ub(1),bounds.u_ub(1)],'k--','DisplayName','V_max')
    plot(t,u(1,:),'DisplayName','V')
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    legend


    nexttile
    % subplot(2,2,4)
    hold on
    plot([t(1),t(end)],rad2deg([bounds.u_lb(2),bounds.u_lb(2)]),'k--','DisplayName','delta_min')
    plot([t(1),t(end)],rad2deg([bounds.u_ub(2),bounds.u_ub(2)]),'k--','DisplayName','delta_max')
    plot(t,rad2deg(u(2,:)),'DisplayName','\delta')
    xlabel('Time [s]')
    ylabel('Degrees')
    legend
end





end