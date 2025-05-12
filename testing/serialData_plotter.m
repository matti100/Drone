clear
clc
close all

serialPort = serialport("/dev/cu.usbserial-0001", 9600);
% serialPort = serialport("/dev/tty.usbserial-0001", 250000);

gyro = 0;
i = 1;

figure;

while true

    data = readline(serialPort);

    if (i > 50)

        gyro = [gyro; str2double(data)];

        if (i <= 150)
            plot(gyro);
        else
            plot(gyro(end-100:end));
        end

        ylim([-20; 10]);
    end

    i = i+1;
end
