function connect_to_flv(protocol,varargin)
    % Check which protocol is being used and additional arguments
    if strcmpi('serial',protocol)
        
        if nargin == 1
            % defaults
            port = 'COM6';
            baudrate = '19200';
            
        elseif nargin == 2
            port = varargin{1};
        elseif nargin == 3
            port = varargin{1};
            baudrate = varargin{2};
        else
            error('Incorrect number of arguments. Only 0, 1 (port), 2 (port, baudrate) arguments. Default values: port = COM6; baudrate = 9600.');
        end
            connection = serial('com6');
            
            set(connection, 'DataBits', 8);
            set(connection, 'StopBits', 1);
            set(connection, 'BaudRate', 19200);
            set(connection, 'Parity', 'none');
            
    elseif strcmpi('tcpip',protocol)
        
        if nargin == 1
            % defaults
            address = '192.168.2.2';
            port = 80;
        elseif nargin == 2
            address = varargin{1};
        elseif nargin == 3
            address = varargin{1};
            port = varargin{2};
        else
            error('Incorrect number of arguments. Only 0, 1 (address), 2 (address,port) arguments. Default values: address = 192.168.2.2; port = 80.');
        end

        temp_str = strcat('Connecting to IP: ',address, '; on Port: ',num2str(port),'.');
        disp(temp_str);
        connection = tcpip(address, port);
    else
        error('Use ''Serial'' or ''TCPIP'' protocol.');
    end

    disp(strcat('Opening ''', protocol, ''' connection.'));
    fopen(connection);
    
    %% Open file for writing
    test_name = 'Test1'; % Change name as appropriate
    file_ext = 'data';

    filename = strcat(datestr(now,'[yyyy-mm-dd][HH-MM-SS]'),test_name,'.',file_ext);
    temp_str = strcat('Preparing file: ', filename);
    disp(temp_str);
    
    data_file = fopen(filename,'a'); %for appending
    %% Cleanup on close
    % This should run when ctrl+C is used.

    disp('Preparing cleanup operations.');
    cleanup1_ = onCleanup(@() fclose(connection)); % Close connection
    cleanup2_ = onCleanup(@() fclose(data_file)); % Close file
    cleanup3_ = onCleanup(@() disp('Safely exited')); % Display progress

    %% Prep figures
    h_fig = figure();
    pData_len = 200;
    pData = nan*zeros(17+3,pData_len);
    pData_temp = nan*zeros(17, 512);
    pData_ind = 1;
    pData_inc = 1; % Allow increment of index
    %set(get(h_fig,'JavaFrame'),'Maximized',1); % maximise
    
    d_count = 0;
    d_count_max = 1;
    
    % Acceleration
    subplot(4,2,1);
    hold on
    h_acl_x = plot(nan,'b');
    h_acl_y = plot(nan,'r');
    h_acl_z = plot(nan,'k');
    hold off
    
    legend('ax','ay','az','Location','NorthWest');
    xlabel('Time (s)');
    ylabel('Acceleration m/s/s');
    
    % Orientation
    subplot(4,2,3);
    hold on
    h_ori_x = plot(nan,'b');
    h_ori_y = plot(nan,'r');
    h_ori_z = plot(nan,'k');
    h_ori_a = plot(nan,'g'); % alpha
    hold off;

    legend('ox','oy','oz','Alpha','Location','NorthWest');
    xlabel('Time (s)');
    ylabel('Orientation (deg)');
    
    % Omega
    subplot(4,2,5);
    hold on
    h_ome_x = plot(nan,'y');
    h_ome_y = plot(nan,'g');
    h_ome_z = plot(nan,'m');
    hold off
    
    legend('wx','wy','wz','Location','NorthWest');
    xlabel('Time (s)');
    ylabel('Omega');

    % Drive commands
    subplot(4,2,7);
    hold on
    h_com_d = plot(nan);
    h_com_s = plot(nan,'r');

    legend('Drive','Steer','Location','NorthWest');
    xlabel('Time (s)');
    ylabel('Command Signal');
    
    %line([time(1),time(end)],[64,64],'Color',[0,0,0]);
    %line([time(1),time(end)],[192,192],'Color',[0,0,0]);
    hold off
    
    subplot(4,2,[2,4]);
    h_tim = plot(nan);
    xlabel('Time (s)');
    ylabel('Time (ms)');
    
    subplot(4,2,[6,8]);
    hold on
    h_mass_right = plot(nan,'b');
    h_mass_left = plot(nan,'k');
    h_mass_rear = plot(nan,'m');
    h_mass_fail = plot(nan,'r','LineWidth',4);
    h_mass_fail_mark = plot(nan,'r*','LineWidth',2);
    
    
    hold off
    legend('Right','Left','Rear','Fail','Location','NorthWest');
    
    hs = [h_acl_x;h_acl_y;h_acl_z;...
        h_ori_x;h_ori_y;h_ori_z;h_ori_a;...
        h_ome_x;h_ome_y;h_ome_z;...
        h_com_d;h_com_s;...
        h_mass_right;h_mass_left;h_mass_rear;...
        h_mass_fail;h_mass_fail_mark;...
        h_tim];
    
    drawnow;
    %% Main reading loop
    
    disp('Beginning reading loop');
    if(strcmpi('tcpip',protocol))
        while(1) % NOTE: Need to find a way to send an exit command (catch interrupt would be best) at the moment we use ctrl+C to exit
            while(get(connection,'BytesAvailable') > 0)
                if(strcmp(connection.status,'open')) % check if connection is still open
                    data = fgetl(connection);
                    fprintf('%s',data); % Write to display - this can be turned off
                    fprintf(data_file,'%s', data); % Write to file
                else
                    disp('Connection lost, exiting now.'); % Can replace with reasquisition
                    break;
                end
            end
        end
    elseif(strcmpi('serial',protocol))
        count = 1;
        d_buffer  = zeros(1,512);
        d_zero = d_buffer;
        while(strcmp(connection.status,'open'))
            data = fgetl(connection);
%             while( connection.BytesAvailable <= 510 )
%             end
%             data = fread(connection,510,'uchar');

            fprintf(data_file,'%s\n',char(data));
            [pData_temp,scan_count] = sscanf(char(data),'[%d][o:%f,%f,%f][a:%f,%f,%f][e:%f,%f][c:%d,%d][w:%d,%d,%d][m:%f,%f,%f]\n',[17, inf]);
            %size(pData_temp)
            %scan_count
            scanned_lines = floor(scan_count/17);
            c_shift = pData_ind + scanned_lines-1 - pData_len;
            if( c_shift > 0)
                pData = circshift(pData,[0 -c_shift]);
                pData_ind = pData_ind - c_shift;
                %pData_inc = 0;
            end
            
            pData(1:17,pData_ind:min(scanned_lines-1+pData_ind, pData_len)) = pData_temp(:,1:scanned_lines);
            
            %char(data)
            %pData(2:17,pData_ind)
            % Update counters
            pData_ind = pData_ind + (scanned_lines)*pData_inc;
            
            d_count = d_count+1;
            if d_count > d_count_max
                d_count = 0;
                updatePlot(hs, pData)
            end
        end;
        disp('Connection lost, exiting now.'); % Can replace with reasquisition
        
    end
end

function updatePlot(hs, pData)
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
    
    time = pData(ind_time,:);
    ax = pData(ind_ax,:);
    ay = pData(ind_ay,:);
    az = pData(ind_az,:);

    ox = pData(ind_ox,:);
    oy = pData(ind_oy,:);
    oz = pData(ind_oz,:);

    dist = pData(ind_dist,:);
    alpha = pData(ind_alpha,:);
    alpha = wrapToPi(alpha);
    
    wx = pData(ind_gyro_rawX,:);
    wy = pData(ind_gyro_rawY,:);
    wz = pData(ind_gyro_rawZ,:);

    c_drive = pData(ind_drive_com,:);
    c_steer = pData(ind_steer_com,:);
    
    m_right = pData(ind_load_right,:);
    m_left = pData(ind_load_left,:);
    m_rear = pData(ind_load_rear,:);
    
    load_nan = zeros(size(m_right));
    load_nan = nan*load_nan;
    load_nan(m_right <= 0) = m_right(m_right <= 0);
    load_nan(m_left <= 0) = m_left(m_left <= 0);
    load_nan(m_rear <= 0) = m_rear(m_rear <= 0);

    time = time/1000;
    
    set(hs(1),'XData', time,'YData', ax);
    set(hs(2),'XData', time,'YData', ay);
    set(hs(3),'XData', time,'YData', az);
    
    set(hs(4),'XData', time,'YData', ox);
    set(hs(5),'XData', time,'YData', oy);
    set(hs(6),'XData', time,'YData', oz);
    set(hs(7),'XData', time,'YData', alpha);
    
    set(hs(8),'XData', time,'YData', wx);
    set(hs(9),'XData', time,'YData', wy);
    set(hs(10),'XData', time,'YData', wz);
    
    set(hs(11),'XData', time,'YData', c_drive);
    set(hs(12),'XData', time,'YData', c_steer);
    
    set(hs(13),'XData', time,'YData', m_right);
    set(hs(14),'XData', time,'YData', m_left);
    set(hs(15),'XData', time,'YData', m_rear);
    
    set(hs(16),'XData', time,'YData', load_nan);
    set(hs(17),'XData', time,'YData', load_nan);
    
    dt = time(2:end) - time(1:end-1);
    set(hs(18),'XData', time(2:end),'YData', dt);
    
    drawnow;
end
