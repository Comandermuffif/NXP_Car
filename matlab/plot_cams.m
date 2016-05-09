% Real time data collection example
%
% This script is implemented as a function so that it can
%   include sub-functions
%
% This script can be modified to be used on any platform by changing the
% serialPort variable.
% Example:-
% On Linux:     serialPort = '/dev/ttyS0';
% On MacOS:     serialPort = '/dev/tty.KeySerial1';
% On Windows:   serialPort = 'COM1';
%
%To run: 
%plot_cams()
%To reset ports: (if MATLAB still thinks they're busy)
%delete(instrfindall)
%
function plot_cams 

    %Send over bluetooth or serial
    serialPort = 'COM4';
    serialObject = serial(serialPort);
    %configure serial connection
    serialObject.BaudRate = 9600; %(Default)
    %serialObject.BaudRate = 115200;
    %serialObject.FlowControl = 'software';

    %Initiate serial connection
    fopen(serialObject);

    % This gets called on cleanup to ensure the stream gets closed
    finishup = onCleanup(@() myCleanupFun(serialObject));

    % Instantiate variables
    count = 1;
    trace = zeros(1, 128); %Stored Values for Raw Input
    
    while (1)
        % Check for data in the stream
        if serialObject.BytesAvailable
            val = fscanf(serialObject,'%i');
            if (val == -1) %Start code
                count = 1;
            elseif (val == -2) % End code
                if (count == 128)
                    plot(trace);
                end
                count = 1;
            else
                trace(count) = val;
                count = count + 1;
            end
        end
    end
    % Clean up the serial object
    fclose(serialObject);
    delete(serialObject);
    clear serialObject;

end %plot_cams

function myCleanupFun(serialObject)
% Clean up the serial object
fclose(serialObject);
delete(serialObject);
clear serialObject;
delete(instrfindall);
end