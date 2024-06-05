clear all
close all
clc

%% Data
% Initial point
x_i = 0;
y_i = 0;
theta_i = 0;

% Generation of end point with norma = 1
while true
    % Final point: random coordinates

    final_conf = rand(1,3);
    x_f = final_conf(1);
    y_f = final_conf(2);
    
    % Theta angle: random between 0 and 2*pi
    theta_f = final_conf(3); 
 
    qi = [x_i;y_i;theta_i];
    qf = [x_f;y_f;theta_f];

    qf = qf/norm(qf);

    norm_vec = norm(qf-qi);

    if abs(norm_vec -1 ) == 0
        break;  % Exit loop if condition is met
    end
end

% Visualization final point
disp(['Coordinate of final point: (', num2str(x_f), ', ', num2str(y_f), ', ', num2str(theta_f), ')']);

%% Parameters of the Cubic Polynomial
k=6;
alfa_x = k*cos(theta_f)-3*x_f;
alfa_y = k*sin(theta_f)-3*y_f;
beta_x = k*cos(theta_i)+3*x_i;
beta_y = k*sin(theta_i)+3*y_i;

%% Definition of time Law
s = 0 : 0.001 : 1;
    
x_s=s.^3.*x_f-(s-1).^3.*x_i+alfa_x.*s.^2.*(s-1)+beta_x.*s.*(s-1).^2;
y_s=s.^3.*y_f-(s-1).^3.*y_i+alfa_y.*s.^2.*(s-1)+beta_y.*s.*(s-1).^2;
x_s_dot = 3.*s.^2.*x_f-3.*(s-1).^2.*x_i+2.*alfa_x.*s.*(s-1)+alfa_x.*s.^2+beta_x.*(s-1).^2+2.*beta_x.*s.*(s-1);
y_s_dot = 3.*s.^2.*y_f-3.*(s-1).^2.*y_i+2.*alfa_y.*s.*(s-1)+alfa_y.*s.^2+beta_y.*(s-1).^2+2.*beta_y.*s.*(s-1);
theta = atan2(y_s_dot, x_s_dot);

%% Plots
figure(1)
subplot(1,2,1)
plot(x_s, y_s, 'LineWidth', 3)
hold on;
plot(x_i, y_i, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r') 
plot(x_f, y_f, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b') 
hold off;
title('Trajectory','FontSize',14)
xlabel('x [meters]','FontSize',14)
ylabel('y [meters]','FontSize',14)
axis square
grid on
legend('Trajectory', 'Initial Point', 'Final Point');

subplot(1,2,2)
plot(s, theta, 'LineWidth', 3)
title('Theta Evolution','FontSize',14)
xlabel('s','FontSize',14)
ylabel('\theta [rad]','FontSize',14)
axis square
grid on