function [ output, fnames ] = read_folder( dir_name, ext, index_to_view)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    %% Reads files saved by connect_to_flv_tcpip.m
%     if nargin < 3
%         index_to_view = [];
        if nargin < 2 
            ext = 'data';
            if nargin<1
                dir_name = './Test Runs/';
            end
        end
%     end
    
    if dir_name(end) ~= '/'
        dir_name = [dir_name, '/'];
    end
    
    all_files = dir(dir_name);
    
    hs = zeros(length(all_files),1);
    
    output{length(all_files)}=[];
    
    if isempty(index_to_view)
        index_to_view = 1:length(all_files);
    end

    for ind = 1:length(all_files)
        %%
        %fid = fopen(strcat(filename,'.',ext),'r'); % Open in read only mode

        % Get data
        % data = fscanf(fid,'[%d][o:%f,%f,%f][a:%f,%f,%f][e:%f,%f][c:%d,%d][w:%d,%d,%d][m:%f,%f,%f]\n',[17, inf]);
        if ~all_files(ind).isdir && length(all_files(ind).name) > length(['.' ext]) && strcmp(all_files(ind).name(end+1-length(ext):end),ext);
            data = csvread([dir_name, all_files(ind).name]);
            data = data';
            output{ind} = data;
            %fclose(fid); % Close file
            % 
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

            time_diff  =  data(ind_time,2:end) - data(ind_time,1:end-1);
            m_right = data(ind_load_right,:);
            m_left = data(ind_load_left,:);
            m_rear = data(ind_load_rear,:);

            load_nan = zeros(size(m_right));
            load_nan = nan*load_nan;
            load_nan(m_right <= 0) = m_right(m_right <= 0);
            load_nan(m_left <= 0) = m_left(m_left <= 0);
            load_nan(m_rear <= 0) = m_rear(m_rear <= 0);
            %%
            red_line = zeros(size(m_right));
            red_line = nan*load_nan;
            red_line(m_right <= 0) = 1;
            red_line(m_left <= 0) = 1;
            red_line(m_rear <= 0) = 1;

            %% Plot
                if any(ind == index_to_view)
                hs(ind) = figure('units','normalized','outerposition',[0 0 1 1]);

                subplot(4,2,1);
                hold on;
                plot(time,ax,'b');
                plot(time,ay,'r');
                plot(time,az,'k');

                plot(time, sqrt((ax.^2 + ay.^2 + az.^2)/3),'m');

                plot(time,ax.*red_line,'b*','MarkerSize',3);
                plot(time,ay.*red_line,'r*','MarkerSize',3);
                plot(time,az.*red_line,'k*','MarkerSize',3);

                hold off;
                legend('ax','ay','az','Location','NorthWestOutside');
                xlabel('Time (s)');
                ylabel('Acceleration m/s/s');
                axis([time(1) time(end) -20 20]);

                subplot(4,2,3);
                hold on;
                plot(time,ox);
                plot(time,oy,'r');
                plot(time,oz,'k');
                plot(time,rad2deg(alpha),'g');

                plot(time,ox.*red_line,'b*','MarkerSize',3);
                plot(time,oy.*red_line,'r*','MarkerSize',3);
                plot(time,oz.*red_line,'k*','MarkerSize',3);
                plot(time,rad2deg(alpha).*red_line,'g*','MarkerSize',3);
                hold off;

                legend('ox','oy','oz','Alpha','Location','NorthWestOutside');
                xlabel('Time (s)');
                ylabel('Orientation (deg)');
                axis([time(1) time(end) -180 180]);

                subplot(4,2,5);
                hold on

                plot(time,wx,'b');
                plot(time,wy,'r');
                plot(time,wz,'k');

                plot(time,wx.*red_line,'b*','MarkerSize',3);
                plot(time,wy.*red_line,'r*','MarkerSize',3);
                plot(time,wz.*red_line,'k*','MarkerSize',3);

                legend('wx','wy','wz','Location','NorthWestOutside');
                xlabel('Time (s)');
                ylabel('Omega');
                hold off
                axis([time(1) time(end) -300 300]);

                subplot(4,2,7);
                hold on
                plot(time,c_drive,'b');
                plot(time,c_steer,'r');

                plot(time,c_drive.*red_line,'b*','MarkerSize',3);
                plot(time,c_steer.*red_line,'r*','MarkerSize',3);

                legend('Drive','Steer','Location','NorthWestOutside');
                xlabel('Time (s)');
                ylabel('Command Signal');
                axis([time(1) time(end) -5 260]);

                line([time(1),time(end)],[64,64],'Color',[0,0,0]);
                line([time(1),time(end)],[192,192],'Color',[0,0,0]);
                hold off

                %%
                %figure()
                subplot(4,2,[2,4])

                title_str = sprintf('Time between samples. mean: %f, std: %f', mean(time_diff), std(time_diff));
                hold on
                plot(time(2:end),time_diff,'b');
                plot(time(2:end),time_diff.*red_line(2:end),'b*','MarkerSize',3);
                hold off
                %axis([0 length(time_diff) 0 300]);
                title(title_str);
                xlabel('Time (s)');
                ylabel('Time (ms)');
                axis([time(1) time(end) 0 300]);

                subplot(4,2,[6,8])

                hold on
                plot(time,m_right);
                plot(time,m_left,'k');
                plot(time,m_rear,'m');
                plot(time, load_nan, 'r', 'LineWidth',4);
                plot(time, load_nan, 'r*', 'LineWidth',2);
                hold off
                legend('Right','Left','Rear','Failure','Location','NorthEastOutside');
                xlabel('Time (s)');
                ylabel('Mass');
                axis([time(1) time(end) -1000 15000]);

                suptitle([dir_name, all_files(ind).name]);
            end %check to plot
        end
    %%
    end
    
end

