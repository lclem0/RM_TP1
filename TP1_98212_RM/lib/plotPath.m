function [total_x, total_y, all_points] = plotPath(beacon_coords, V, Dt)

% function created by Joao Clemente (98212) to calculate the both the
% linear and the pchip path for to robot to traverse

% beacons calculation
    figure()
    subplot(1,2,1)
    for i = 1:size(beacon_coords,1)
        
        rad= 5;
        th = 0:pi/50:2*pi;
        
        xunit = rad* cos(th) + beacon_coords(i,1);
        yunit = rad* sin(th) + beacon_coords(i,2);
        plot(xunit, yunit, 'b--', 'LineWidth', 1); % Larger radius with dashed line

        hold on
    end

    current_x = 0;
    current_y = 0;
    total_x = [];
    total_y = [];
    beacon_coords = [0 0; beacon_coords];
    for i = 1:size(beacon_coords,1)-1
        distance = sqrt((beacon_coords(i+1,1) - beacon_coords(i,1))^2 + (beacon_coords(i+1,2) - beacon_coords(i,2))^2);
        steps = ceil(distance / (V*Dt));
        x_steps = (beacon_coords(i+1,1) - beacon_coords(i,1)) / steps;
        y_steps = (beacon_coords(i+1,2) - beacon_coords(i,2)) / steps;
        for j = 1:steps
            current_x = current_x + x_steps;
            current_y = current_y + y_steps;
            total_x = [total_x current_x];
            total_y = [total_y current_y];
        end
    end
    title ("Beacons with linearized path")
    plot(total_x,total_y,'r.','MarkerSize',10);
    xlabel('x(m)');
    ylabel('y(m)');
    
    %pchip part - right figure - true data
    subplot(1,2,2)
    for i = 1:size(beacon_coords,1)
        rad= 5;
        th = 0:pi/50:2*pi;
        xunit = rad* cos(th) + beacon_coords(i,1);
        yunit = rad* sin(th) + beacon_coords(i,2);
        plot(xunit, yunit, 'b--', 'LineWidth', 1); 
        axis([min(total_x)-10 max(total_x)+10 min(total_y)-10 max(total_y)+10]);
    
        hold on
    end

    x_pchip_values = [];
    y_pchip_values = [];
    
    for i=1:size(beacon_coords,1)-1
        distance = sqrt((beacon_coords(i+1,1) - beacon_coords(i,1))^2 + (beacon_coords(i+1,2) - beacon_coords(i,2))^2);
        steps = ceil(distance / (V * Dt));


        % Generate x-coordinates for this segment
        % avoid duplication
        if i == 1
            x_segment = linspace(beacon_coords(i, 1), beacon_coords(i+1, 1), steps);
        else
        x_segment = linspace(beacon_coords(i, 1), beacon_coords(i+1, 1), steps + 1);
        x_segment = x_segment(2:end);  % Skip the first point of the segment
        end
    
        % Apply pchip interpolation to the updated beacon coordinates
        y_pchip = pchip(beacon_coords(:, 1), beacon_coords(:, 2), x_segment);
        
        x_pchip_values = [x_pchip_values x_segment];
        y_pchip_values = [y_pchip_values y_pchip];

    end

    % Plotting the pchip path
    title ("Path by Hermite polynomial interpolation (pchip)")
    plot(x_pchip_values, y_pchip_values, 'g-', 'MarkerSize', 10)
    xlabel('x(m)');
    ylabel('y(m)');

    hold on
    for i = 1:length(x_pchip_values)
        line([x_pchip_values(i), x_pchip_values(i)], [0, y_pchip_values(i)], 'Color', 'blue', 'LineStyle', '--');
    end

    angles= [0]; % Initialize with a starting orientation 0.
    all_x_y = [x_pchip_values' y_pchip_values'];
    for i=1:size(all_x_y,1)-1
        dx = all_x_y(i+1,1) - all_x_y(i,1);
        dy = all_x_y(i+1,2) - all_x_y(i,2);
        ang_i = atan2(dy, dx);
        angles = [angles; ang_i];    
    end
    %true x, y, theta
    all_points = [all_x_y angles];

    % Plot lines with angle orientation to confirm they correspond to the 
    % right direction
    hold on;

    lineLength = 2; 
    
    for i=1:size(all_points,1)-1
        dx = lineLength * cos(angles(i));
        dy = lineLength * sin(angles(i));
        
        startX = all_points(i,1);
        startY = all_points(i,2);
        
        endX = startX + dx;
        endY = startY + dy;
        
        plot([startX endX], [startY endY], 'r-', 'LineWidth', 2);
    end
    
    hold off;
   
end