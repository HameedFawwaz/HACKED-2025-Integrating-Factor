
function IMU_Orientation_Function()
    
    %% Port Setup
    portName = "COM4";    % 4 or 7 depending on esp
    baudRate = 115200;    % make sure to update in platformIO init file
    sampleRate = 100;     % sampling rate
    
    %% Serial data stream
    s = serialport(portName, baudRate);
    configureTerminator(s, "LF");
    flush(s);                      % Clear old data
    
    %% IMU Filter in built toolbox thing
    fuse = imufilter('SampleRate', sampleRate);

    %% 3d box setup
    figure('Name','Real-Time IMU Orientation','NumberTitle','off');
    clf;
    axis equal; grid on; view(3); hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('IMU Orientation');
    
    % Cube definition, change to rectangle later
    vertices = [
        0 0 0;
        1 0 0;
        1 1 0;
        0 1 0;
        0 0 1;
        1 0 1;
        1 1 1;
        0 1 1
    ];
    centeredVertices = vertices - 0.5; 
    faces = [
        1 2 3 4;
        5 6 7 8;
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8
    ];
    
    % Create an hgtransform
    t = hgtransform;
    hBox = patch('Vertices', centeredVertices, 'Faces', faces, ...
                 'FaceColor', 'cyan', 'FaceAlpha', 0.3, ...
                 'Parent', t);
    
    % Expand axes limits so we can see the box rotating
    xlim([-3 3]); ylim([-3 3]); zlim([-3 3]);
    
    %% Initial Orientation
    initialQ = [];  
    
    %% read, fuse, update loop. will probably shit itself but we see
    disp('Reading IMU data...  (Press Ctrl+C to stop)');

    while true
        % Read all available lines (if multiple lines arrive quickly)
        while s.NumBytesAvailable > 0
            lineStr = readline(s);
            dataValues = str2double(split(lineStr, ","));
            
            % Check [ax, ay, az, gx, gy, gz]
            if numel(dataValues) == 6
                % sensor data
                ax = dataValues(1);  % m/s^2
                ay = dataValues(2);
                az = dataValues(3);
                gx = dataValues(4);  % deg/s
                gy = dataValues(5);
                gz = dataValues(6);
                
                % Convert gyro from deg/s to rad/s
                gyroSI  = deg2rad([gx; gy; gz]);
                accelSI = [ax; ay; az];  % m/s^2
                
                % Fuse to get orientation
                q = fuse(accelSI', gyroSI');
                
                if isempty(initialQ)
                    initialQ = q;
                end
                
                % Make orientation relative to the initial orientation
                relQ = quatmultiply(quatconj(initialQ), q);
                
                % Convert to rot matrix
                R = quat2rotm(relQ);
                
                % hg transform matrix
                T = eye(4);
                T(1:3,1:3) = R;
                
                % translation. Comment if shit itself
                %T(1:3,4) = [0; 0; 0];
                
                % transform
                set(t, 'Matrix', T);
                
                % Update fig
                drawnow limitrate;
            else
                fprintf("Skipping invalid line: %s\n", lineStr);
            end
        end
        pause(0.01); % delay so we don't die
    end
    
end
