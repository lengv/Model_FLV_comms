%% Reads files saved by connect_to_flv_tcpip.m

filename = '[2015-03-19][14-11-00]Stop_test1'; % Choose appropriate file
ext = 'data';

fid = fopen(strcat(filename,'.',ext,'r')); % Open in read only mode

% Get data
data = fscanf(fid,'[%d][o:%f,%f,%f][a:%f,%f,%f][e:%f,%f][c:%d,%d][w:%d,%d,%d]\n',[14, inf]);

fclose(fid); % Close file

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

%% Put data into appropriate variables
time = data(ind_time,:);
ax = data(ind_ax,:);
ay = data(ind_ay,:);
az = data(ind_az,:);

ox = data(ind_ox,:);
oy = data(ind_oy,:);
oz = data(ind_oz,:);

dist = data(ind_dist,:);
alpha = data(ind_alpha,:);
alpha = wrapToPi(alpha);

c_drive = data(ind_drive_com,:);
c_steer = data(ind_steer_com,:);

wx = data(ind_gyro_rawX,:);
wy = data(ind_gyro_rawY,:);
wz = data(ind_gyro_rawZ,:);

time = time/1000;

%% Plot
figure();
subplot(3,1,1);
hold on;
plot(time,ax);
plot(time,ay,'r');
plot(time,az,'k');
%plot(time,rad2deg(alpha),'g');
hold off;
legend('ax','ay','az','Location','NorthWest');
xlabel('Time (s)');
ylabel('Acceleration m/s/s');

subplot(3,1,2);
hold on;
plot(time,ox);
plot(time,oy,'r');
plot(time,oz,'k');
plot(time,rad2deg(alpha),'g');
hold off;

legend('ox','oy','oz','Alpha','Location','NorthWest');
xlabel('Time (s)');
ylabel('Orientation (deg)');

subplot(3,1,3);
hold on
plot(time,c_drive);
plot(time,c_steer,'r');

plot(time,wx,'y');
plot(time,wy,'g');
plot(time,wz,'m');

legend('Drive','Steer','wx','wy','wz','Location','NorthWest');
xlabel('Time (s)');
ylabel('Command Signal');

line([time(1),time(end)],[64,64],'Color',[0,0,0]);
line([time(1),time(end)],[192,192],'Color',[0,0,0]);
hold off
