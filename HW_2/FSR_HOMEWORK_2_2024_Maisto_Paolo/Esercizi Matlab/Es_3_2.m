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
while true

valore = input('Insert tf value: ');
ti = 0;
tf = valore;
t = 0:0.0001:tf;

% Computing of coefficients a2 and a3
a0 = 0;
a1 = 0;
a2 = 3  / (tf^2);
a3 = -2 / (tf^3);
s = a0 + a1 * t + a2 * t.^2 + a3 * t.^3;
ds = 2 * a2 * t + 3 * a3 * t.^2;

for i=1 : length(t)
    si=s(i);

    x(i)=si^3*x_f-(si-1)^3*x_i+alfa_x*si^2*(si-1)+beta_x*si*(si-1)^2;
    y(i)=si^3*y_f-(si-1)^3*y_i+alfa_y*si^2*(si-1)+beta_y*si*(si-1)^2;
    xp(i)=3*si^2*x_f-3*(si-1)^2*x_i+2*alfa_x*si*(si-1)+alfa_x*si^2+beta_x*(si-1)^2+2*beta_x*si*(si-1);
    yp(i)=3*si^2*y_f-3*(si-1)^2*y_i+2*alfa_y*si*(si-1)+alfa_y*si^2+beta_y*(si-1)^2+2*beta_y*si*(si-1);
    xpp(i)=6*si*x_f-6*(si-1)*x_i+2*alfa_x*(si-1)+2*alfa_x*si+2*alfa_x*si+2*beta_x*(si-1)+2*beta_x*(si-1)+2*beta_x*si;
    ypp(i)=6*si*y_f-6*(si-1)*y_i+2*alfa_y*(si-1)+2*alfa_y*si+2*alfa_y*si+2*beta_y*(si-1)+2*beta_y*(si-1)+2*beta_y*si;
    theta(i)=atan2(yp(i),xp(i));

    v_tilde(i)=sqrt(xp(i)^2+yp(i)^2);
    w_tilde(i)=(ypp(i)*xp(i)-xpp(i)*yp(i))/(xp(i)^2+yp(i)^2);
    
    v(i)=(v_tilde(i)*ds(i)); 
    w(i)=(w_tilde(i)*ds(i));
end

v_max = 2;          % Heading speed limit
omega_max = 1;      % Angular speed limit

% Constraints
if (max(abs(v)) <= v_max && max(abs(w)) <= omega_max)
   break;
end

disp('It is recommended to increase the T period');
figure
subplot(1,2,1)
plot(t, v, 'LineWidth', 3)
title('Heading Velocity','FontSize',14)
xlabel('t [seconds]','FontSize',14)
ylabel('v [m/s]','FontSize',14)
axis square
grid on

subplot(1,2,2)
plot(t, w, 'LineWidth', 3)
title('Angular Velocity','FontSize',14)
xlabel('t [seconds]','FontSize',14)
ylabel('$$\omega$$ [rad/s]','Interpreter','latex','FontSize',14)
axis square
grid on

end

%% Plots
figure(1)
subplot(1,2,1)
plot(x, y, 'LineWidth', 3)
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
ylabel('$\theta$ [rad]','FontSize',14)
axis square
grid on

figure(2)
subplot(1,2,1)
plot(s, v_tilde, 'LineWidth', 3)
title('Geometric Heading Velocity','FontSize',14)
xlabel('s','FontSize',14)
ylabel('$$\tilde{v}$$ [m/s]','Interpreter','latex','FontSize',14)
axis square
grid on

subplot(1,2,2)
plot(s, w_tilde, 'LineWidth', 3)
title('Geometric Angular Velocity','FontSize',14)
xlabel('s','FontSize',14)
ylabel('$$\tilde{\omega}$$ [rad/s]','Interpreter','latex','FontSize',14)
axis square
grid on

figure(3)
subplot(1,2,1)
plot(t, v, 'LineWidth', 3)
title('Heading Velocity','FontSize',14)
xlabel('t [seconds]','FontSize',14)
ylabel('v [m/s]','FontSize',14)
axis square
grid on

subplot(1,2,2)
plot(t, w, 'LineWidth', 3)
title('Angular Velocity','FontSize',14)
xlabel('t [seconds]','FontSize',14)
ylabel('$$\omega$$ [rad/s]','Interpreter','latex','FontSize',14)
axis square
grid on

disp('All constraints have been satisfied');

xd=timeseries(x,t);
yd=timeseries(y,t);
thetad=timeseries(theta,t);