% Index names
ind_time = 1;   
ind_ox = 2;
ind_oy = 3;
ind_oz = 4;
ind_ax = 5;
ind_ay = 6;
ind_az = 7;
ind_dist = 8;
ind_alpha = 9;

ind_drive_com = 10;
ind_steer_com = 11;

ind_gyro_rawX = 12;
ind_gyro_rawY = 13;
ind_gyro_rawZ = 14;

ind_load_right = 15;
ind_load_left = 16;
ind_load_rear = 17;

%% Check if data has been read and don't read again (slow)
% Need to seperate the plotting function from reading
if ~exist('all_data','var')
    all_data = read_folder('./Test Runs/','data',[ 14 ]);
end


%% Choose data and get time information
index = 14; % Choose which test run (index = {1,2} are directories)

data = all_data{index};

data = data(:,15:end); % Ignore some of the start (set this depending on test to ignore unimportant information)

time = data(ind_time,:)/1000; % Time in seconds
dt = diff(time); % Effective sampling time

%% Angular acceleration 
omega_dX = (data(ind_gyro_rawX,2:end) - data(ind_gyro_rawX,1:end-1)).*dt;
omega_dY = (data(ind_gyro_rawY,2:end) - data(ind_gyro_rawY,1:end-1)).*dt;
omega_dZ = (data(ind_gyro_rawZ,2:end) - data(ind_gyro_rawZ,1:end-1)).*dt;

%data(ind_az,:) = data(ind_az,:) - 9.81;

% Plot Angular acceleration
figure()
hold on
plot(time(2:end),omega_dX,'b');
plot(time(2:end),omega_dY,'r');
plot(time(2:end),omega_dZ,'k');
hold off
title('\dot{\omega}');

%% Velocities
% vx = data(ind_ax,2:end).*dt;
% vy = data(ind_ay,2:end).*dt;
% vz = (data(ind_az,2:end)- 9.81).*dt;

vx = cumtrapz(data(ind_ax,2:end)).*dt;
vy = cumtrapz(data(ind_ay,2:end)).*dt;
vz = cumtrapz(data(ind_az,2:end)- 9.81).*dt;
mag_vs = sqrt(vx.^2 + vy.^2 + vz.^2); % Assume only planar motion

figure()
hold on
plot(time(2:end),vx,'b');
plot(time(2:end),vy,'r');
plot(time(2:end),vz,'k');
plot(time(2:end),mag_vs,'m');
hold off
title('Velocities');

figure()
mag_as = sqrt(data(ind_ax,:).^2 + data(ind_ay,:).^2 + (data(ind_az,:)-9.81).^2);
plot(time, mag_as)
title('Magnitude of acceleration');

figure()
plot(time, sum(data([ind_load_left, ind_load_right, ind_load_rear],:)));
title('Total mass');

%% Loads
m_chassis = 13;
m_load = 5;

m = m_chassis + m_load;

%% Rough position of CoG and IMU
r_cog_chassis = [-0.4;0;1.5];
r_cog_mass = [-0.25;-0.18;0.56];

r_cog = r_cog_chassis*m_chassis/m + r_cog_mass*m_load/m;

r_imu = [-0.325 ; 0.1; 0.25];

%%
Ixx = 0.85;
Iyy = 0.97;
Izz = 0.58;

Ixz = 0;
Ixy = 0.17;

Iyx = Ixy;
Iyz = 0;

Izx = Ixz;
Izy = Iyz;

I = [Ixx Ixy Ixz; Iyx Iyy Iyz; Izx Izy Izz];

%%
wheels = [0 0 -0.665 0 ; -0.20 0.20 0 -0.2];
%data(data([ind_load_right ind_load_left ind_load_rear],:) < 0)  = 0;

data([ind_load_right ind_load_left ind_load_rear],:) = data([ind_load_right ind_load_left ind_load_rear],:)/1000;

m_mean = mean(sum(data([ind_load_right ind_load_left ind_load_rear],:)));
m_std = std(sum(data([ind_load_right ind_load_left ind_load_rear],:)));

zmp_w = [((data(ind_load_right,:))*wheels(1,1) +  (data(ind_load_left,:))*wheels(1,2) + (data(ind_load_rear,:))*wheels(1,3))./m_mean;...
        ((data(ind_load_right,:))*wheels(2,1) +  (data(ind_load_left,:))*wheels(2,2) + (data(ind_load_rear,:))*wheels(2,3))/m_mean];
zmp_w = zmp_w(:,2:end);
%%
zmps = zeros(3,length(data)-1);
criteria_w = zeros(3,length(data)-1);
stable_w = zeros(3,length(data)-1);

criteria_s = zeros(3,length(data)-1);
stable_s = zeros(3,length(data)-1);

for ind = 2:length(data)
    w = deg2rad([data(ind_gyro_rawX,ind); data(ind_gyro_rawY,ind); data(ind_gyro_rawZ,ind)])/16.4;
    w_dot = deg2rad([omega_dX(ind-1); omega_dY(ind-1); omega_dZ(ind-1)])/16.4;
    A = [data(ind_ax,ind); data(ind_ay,ind); data(ind_az,ind)] - cross(w_dot, (r_cog - r_imu));
    %A = [0;0;-9.8];
    R = [1 ,0 , 0; 0, 1, 0; 0, 0, 1];
    [zmp_x, zmp_y, zmp_z] = calculate_zmp(A,I,R,w,w_dot,m,r_cog);
    zmps(:,ind-1) = [zmp_x; zmp_y; zmp_z];
    [criteria_s(:,ind-1), stable_s(:,ind-1)] = check_stability(wheels(:,1:end-1), [zmp_x; zmp_y; zmp_z], 0);
    [criteria_w(:,ind-1), stable_w(:,ind-1)] = check_stability(wheels(:,1:end-1), [zmp_w(:,ind-1);0], 0);
end

% figure();
% hold on
% 
% plot(wheels(1,:), wheels(2,:),'k');
% cmap = jet(length(zmps));
% scatter(zmps(1,:),zmps(2,:),10,cmap,'filled');
% plot(zmps(1,:),zmps(2,:));
% 
% cmap = jet(length(zmp_w));
% scatter(zmp_w(1,:),zmp_w(2,:),10,cmap,'filled');
% plot(zmp_w(1,:),zmp_w(2,:));
% 
% plot(r_cog(1), r_cog(2), 'k*');
% hold off

%% Timed
figure();
hold on

plot(wheels(1,:), wheels(2,:),'k');
w_cmap = winter(length(zmps));
s_cmap = autumn(length(zmps));
h_zmps_s = scatter(zmps(1,1),zmps(2,1),10,s_cmap(1,:),'filled');
h_zmps_p = plot(zmps(1,1),zmps(2,1),'b');

h_zmpw_s = scatter(zmp_w(1,1),zmp_w(2,1),10,w_cmap(1,:),'filled');
h_zmpw_p = plot(zmp_w(1,1),zmp_w(2,1),'r');

plot(r_cog(1), r_cog(2), 'k*');
plot(r_cog_chassis(1), r_cog_chassis(2), 'c*');
plot(r_cog_mass(1), r_cog_mass(2), 'm*');
hold off

for i=1:length(zmps)
    set(h_zmps_s,'XData',zmps(1,1:i));
    set(h_zmps_s,'YData',zmps(2,1:i));
    set(h_zmps_p,'XData',zmps(1,1:i));
    set(h_zmps_p,'YData',zmps(2,1:i));
    set(h_zmps_s,'CData',s_cmap(1:i,:));
    
    set(h_zmpw_s,'XData',zmp_w(1,1:i));
    set(h_zmpw_s,'YData',zmp_w(2,1:i));
    set(h_zmpw_p,'XData',zmp_w(1,1:i));
    set(h_zmpw_p,'YData',zmp_w(2,1:i));
    set(h_zmpw_s,'CData',w_cmap(1:i,:));
    drawnow;
    pause(0.03);
end

%% 
criteria_w(:,abs(sum(data([ind_load_right ind_load_left ind_load_rear],2:end)) - m_mean) >= 2*m_std) = 0;
figure();
hold on
plot(data(ind_time,2:end), min(criteria_s),'b');
plot(data(ind_time,2:end),min(criteria_w),'r');
hold off