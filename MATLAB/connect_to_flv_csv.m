function connect_to_flv_csv(protocol,varargin)
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
            set(connection, 'InputBufferSize', 2058);
            
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
        
        while(strcmp(connection.status,'open'))
            data = fread(connection,connection.inputBufferSize,'uchar');

            fprintf(data_file,'%s',(data));
        end;
        disp('Connection lost, exiting now.'); % Can replace with reasquisition
        
    end
end
% ind_time = 1;   
% ind_ox = 2;
% ind_oy = 3;
% ind_oz = 4;
% ind_ax = 5;
% ind_ay = 6;
% ind_az = 7;
% ind_dist = 8;
% ind_alpha = 9;
% 
% ind_drive_com = 10;
% ind_steer_com = 11;
% 
% ind_gyro_rawX = 12;
% ind_gyro_rawY = 13;
% ind_gyro_rawZ = 14;
% 
% ind_load_right = 15;
% ind_load_left = 16;
% ind_load_rear = 17;