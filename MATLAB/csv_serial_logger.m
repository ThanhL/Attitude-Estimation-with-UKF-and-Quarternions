%% CSV data logger
teensy_device = serialport("COM6", 115200)


while 1
    % data = read(teensy_device, 9, "double");
    data = readline(teensy_device);
    imu_raw_data = sscanf(data, '%f');
    imu_raw_data = imu_raw_data';
    imu_raw_data
    
    writematrix(imu_raw_data,'imu_raw_data.csv', 'WriteMode','append') 
    
end
