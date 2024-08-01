clear; clc;
% Check available serial ports and identify Arduino port
serialInfo = serialportlist("available");
disp('Available ports:');
disp(serialInfo); 
port = "COM14";  % Replace with your Arduino's port number

% Create serial port object
arduinoSerial = serialport(port, 250000);
% Set a longer timeout if data is infrequent
configureTerminator(arduinoSerial, "LF", "LF"); % Ensuring correct terminator
set(arduinoSerial, 'Timeout', 10);

% Configure a clean-up function to ensure the serial port gets closed when the script terminates
cleanupObj = onCleanup(@() configureCleanup(arduinoSerial));

% Initialize storage for parsed data
sensorData = [];
flush(arduinoSerial);

% Read data continuously and parse it
try
    while true
        data = readline(arduinoSerial);  % Read the data from Arduino
        if isempty(data) || strcmp(data, "")
            disp('No data received. Check Arduino output.');
            continue; % Skip the rest of the loop if no data received
        end
        parsedData = parseSensorData(data);  % Parse the data
        if isempty(parsedData)
            disp('Invalid data format received.');
            continue; % Skip appending and displaying if data is invalid
        end
        sensorData = [sensorData; parsedData];  % Append new data to the array
        disp('Updated Sensor Data:');
        disp(sensorData);  % Display the updated data array in the MATLAB Command Window
    end
catch ME
    disp('Error reading from serial port.');
    disp(getReport(ME));
end

function data = parseSensorData(inputString)
    % Parse input string from Arduino and convert into numerical data array
    % Safeguard against non-text input
    if isstring(inputString) || ischar(inputString)
        values = str2double(split(inputString, ','));
        if any(isnan(values)) % Check for non-numeric entries that failed to convert
            data = [];
        else
            data = values'; % Return a column vector of the data
        end
    else
        data = [];
    end
end

function configureCleanup(s)
    % Function to close serial port
    disp('Cleaning up serial port...');
    configureTerminator(s, "LF");  % Restore default line terminator for safety
    delete(s);
end
