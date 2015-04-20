function connect_to_flv_tcpip(varargin)
    address = '192.168.2.2';
    port = 80;
    if nargin == 1
        address = varargin{1};
    elseif nargin == 2
        port = varargin{2};
    else
        error('Incorrect number of arguments. Only 0, 1 (address), 2 (address,port) arguments. Default values: address = 192.168.2.2; port = 80.');
    end
    
    
    temp_str = strcat('Connecting to IP: ',address, '; on Port: ',num2str(port),'.');
    disp(temp_str);
    connection = tcpip(address, port);

    disp('Opening connection.');
    fopen(connection);
    %%

    test_name = 'Stop_test1';
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
    while(1) % NOTE: Need to find a way to send an exit command (catch interrupt would be best) at the moment we use ctrl+C to exit
        while(get(connection,'BytesAvailable') > 0)
            if(strcmp(connection.status,'open')) % check if connection is still open
                data = fread(connection,connection.BytesAvailable);
                fprintf('%s',data); % Write to display - this can be turned off
                fprintf(data_file,'%s', data); % Write to file
            else
                disp('Connection lost, exiting now.'); % Can replace with reasquisition
                break;
            end
        end
    end
end