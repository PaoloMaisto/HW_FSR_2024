%% EXERCISE 4
%% FSR HOMEWORK 3 2024
%% PAOLO MAISTO P38000191

close all
clc
clear all

%% Parameters
load("ws_homework_3_2024.mat");
time = attitude.time;
Ts = 1*10^-3;
m = 1.5;
g = 9.81;
Ib = diag([1.2416 1.2416 2*1.2416]);
e3 = [0 0 1]';

%% Order Estimator
r = 1; 

%% Estimator
q = zeros(6,length(time));
gamma = zeros(6,length(time),r); 
ex_w = zeros(6,length(time));
wn = 1;

%% Butter filter
[b,cj] = butter(r,wn,'low','s');
G = tf(b,cj)

%% Gains
cj = flip(cj);
K = zeros(1,r);
p = 1;
for i = r:-1:1
    if i == r
        K(i) = cj(i);
        p = K(i);
    elseif i == 1
        K(i) = cj(1)/p;
    else 
        K(i) = cj(i)/p;
        p = p*K(i);
    end
end

for k=1:length(time)-1

    % using the file .mat provided define the following parameters
    % Attitude
    phi = attitude.signals.values(k,1);
    theta = attitude.signals.values(k,2);
    psi = attitude.signals.values(k,3);
    % Attitude Velocities
    phi_dot = attitude_vel.signals.values(k,1);
    theta_dot = attitude_vel.signals.values(k,2);
    psi_dot = attitude_vel.signals.values(k,3);
        
    Rb = [cos(theta)*cos(psi)   sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)  cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
          cos(theta)*sin(psi)   sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
          -sin(theta)           sin(phi)*cos(theta)                             cos(phi)*cos(theta)];

    Q = [1  0           -sin(theta);
         0  cos(phi)    cos(theta)*sin(phi)
         0  -sin(phi)   cos(phi)*cos(theta)];
    
    Q_dot = [0  0                   -cos(theta)*theta_dot;
             0  -sin(phi)*phi_dot   -sin(theta)*sin(phi)*theta_dot+cos(theta)*cos(phi)*phi_dot;
             0  -cos(phi)*phi_dot   -sin(theta)*cos(phi)*theta_dot-cos(theta)*sin(phi)*phi_dot];

    M = Q'*Ib*Q;

    q(:,k+1) = [m*eye(3) zeros(3,3); zeros(3,3) M]*[linear_vel.signals.values(k+1,:)' ; attitude_vel.signals.values(k+1,:)']  ;
   
    S = skew(Q*attitude_vel.signals.values(k,:)');

    C = Q'*S*Ib*Q+Q'*Ib*Q_dot;

    gamma(:,k+1,1) = gamma(:,k,1)+K(1)*((q(:,k+1)-q(:,k))-Ts*[m*g*e3-thrust.signals.values(k)*Rb*e3;C'*attitude_vel.signals.values(k,:)'+Q'*tau.signals.values(k,:)']-Ts*ex_w(:,k));
    if r>=2
        for i = 2:r
           gamma(:,k+1,i) = gamma(:,k,i)+Ts*K(i)*(-ex_w(:,k)+gamma(:,k,i-1) );
        end
    end
   ex_w(:,k+1) = gamma(:,k+1,r);
end

%% Results
fprintf('RESULTS:\n');
fprintf('f_x = %f N\n',ex_w(1,end));
fprintf('f_y = %f N\n',ex_w(2,end));
fprintf('f_z = %f N\n',ex_w(3,end));
fprintf('tau_x = %f Nm\n',ex_w(4,end));
fprintf('tau_y = %f Nm\n',ex_w(5,end));
fprintf('tau_z = %f Nm\n',ex_w(6,end));

%% Plots

% Disturbances required from the trace
d_x = 0.5;
d_y = 0.5;
d_yaw = 0.2;

personal_plot2(time, ex_w(1,:), d_x.*ones(length(time),1), 'time[s]', 'force along x axis [N]', '$\hat{f_x}$', 'disturbance along x-axis', 'f_x.pdf')
personal_plot2(time, ex_w(2,:), d_y.*ones(length(time),1), 'time[s]', 'force along y axis [N]', '$\hat{f_y}$', 'disturbance along y-axis', 'f_y.pdf')
personal_plot1(time, ex_w(3,:), 'time[s]', '$\hat{f_z}$ [N] ', 'f_z.pdf')
personal_plot3(time, ex_w(1,:), ex_w(2,:), ex_w(3,:), 'time[s]', 'force along axis [N]', '$\hat{f_x}$', '$\hat{f_y}$', '$\hat{f_z}$', 'f.pdf')

personal_plot1(time, ex_w(4,:), 'time[s]', '$\hat{\tau_x}$ [Nm] ', 'tau_x.pdf')
personal_plot1(time, ex_w(5,:), 'time[s]', '$\hat{\tau_y}$ [Nm] ', 'tau_y.pdf')
personal_plot2(time, ex_w(6,:), d_yaw.*ones(length(time),1), 'time[s]', 'torque [Nm] ', '$\hat{\tau_z}$', 'disturbance around yaw-axis', 'tau_z.pdf')
personal_plot3(time, ex_w(4,:), ex_w(5,:), ex_w(6,:), 'time[s]', 'torques [Nm] ', '$\hat{\tau_x}$', '$\hat{\tau_y}$', '$\hat{\tau_z}$', 'tau.pdf')

%% Real mass using the z-disturbance 
mass_r = -(-ex_w(3,end)/g)+m; 
