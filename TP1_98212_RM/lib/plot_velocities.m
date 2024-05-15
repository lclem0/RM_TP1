function plot_velocities(velocities)

% function created by Joao Clemente (98212)
figure()

subplot(2,1,1)
    grid on
    plot(velocities(:,1), 'b-', 'MarkerSize', 10)
    hold on
    plot(velocities(:,3), 'r-', 'MarkerSize', 10)
    title("Linear Velocities");
    xlabel('Point trajectory');
    ylabel('Velocity (m/s)');
    legend('Without Noise', 'With Noise');
    grid on;
    hold off;

subplot(2,1,2)
    grid on
    plot(velocities(:,2), 'b-', 'MarkerSize', 10)
    hold on
    plot(velocities(:,4), 'r-', 'MarkerSize', 10)
    title("Angular Velocities");
    xlabel('Point trajectory');
    ylabel('Angular Velocity (rad/s)');
    legend('Without Noise', 'With Noise');
    grid on;
    hold off;
    