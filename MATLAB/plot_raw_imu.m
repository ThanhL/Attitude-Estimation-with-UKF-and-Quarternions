function plot_raw_imu(serial_port, ~)
    % Read the ASCII data from the serialport object.
    data = readline(serial_port);

    % Convert the string data to numeric type and save it in the UserData
    % property of the serialport object.
    serial_port.UserData.Data(end+1) = str2double(data);
    
    % Update the Count value of the serialport object.
    serial_port.UserData.Count = serial_port.UserData.Count + 1;
    
    % If 1001 data points have been collected from the Arduino, switch off 
    % the callbacks and plot the data.
    if serial_port.UserData.Count > 1001
        configureCallback(serial_port, "off");
        plot(serial_port.UserData.Data(2:end));
    end
    
end

