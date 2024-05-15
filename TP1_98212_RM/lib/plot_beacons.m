function plot_beacons(beacon_coords)
%not used anymore

for i = 1:size(beacon_coords,1)
    
    rad= 5;
    th = 0:pi/50:2*pi;
    
    % Calculate circle coordinates
    xunit = rad* cos(th) + beacon_coords(i,1);
    yunit = rad* sin(th) + beacon_coords(i,2);

    plot(xunit, yunit, 'b--', 'LineWidth', 1); % Larger radius with dashed line
    axis equal
    axis([0 200 0 200])
    
    hold on
end
