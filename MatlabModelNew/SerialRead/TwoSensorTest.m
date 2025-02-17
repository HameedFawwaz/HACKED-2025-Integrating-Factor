function TwoSensorTest()
    
    %% Port Setup
    portName = "COM4";    % Adjust as needed (e.g., "COM4" or "COM7")
    baudRate = 115200;    % Must match your ESP configuration
    sampleRate = 100;     % Sampling rate (Hz)
    
    %% Serial data stream
    s = serialport(portName, baudRate);
    configureTerminator(s, "LF");
    flush(s);  % Clear any old data
    
    %% IMU Filter (requires MATLAB’s sensor fusion toolbox)
    fuse = imufilter('SampleRate', sampleRate);
    
    %% 3D Box (Cube) Setup
    figure('Name','Real-Time IMU Orientation and Position','NumberTitle','off');
    clf;
    axis equal; grid on; view(3); hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('IMU Orientation and Position');
    
    % Define cube vertices and faces (cube centered at origin)
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
    
    % Create an hgtransform for the cube so that we can update its transform
    t = hgtransform;
    hBox = patch('Vertices', centeredVertices, 'Faces', faces, ...
                 'FaceColor', 'cyan', 'FaceAlpha', 0.3, ...
                 'Parent', t);
    
    % Set axis limits to allow room for both rotation and translation
    xlim([-20 10]); ylim([-20 10]); zlim([-20 10]);
    
    %% Initial Orientation, Position, and Velocity
    initialQ = [];          % For relative orientation
    velocity = [0; 0; 0];   % Initial velocity (m/s)
    position = [0; 0; 0];   % Initial position (m)
    g = 9.5;               % Gravitational acceleration (m/s^2)
    
    % Define thresholds to limit drift from integration errors
    accelThreshold = 0.7;   % m/s^2; ignore very small accelerations
    velThreshold = 0.01;    % m/s; ignore very small velocities
    
    %% Timing Setup
    lastTime = tic;
    
    %% Read, Fuse, and Update Loop
    disp('Reading IMU data... (Press Ctrl+C to stop)');
    
    while true
        % Process all available serial lines
        while s.NumBytesAvailable > 0
            lineStr = readline(s);
            dataValues = str2double(split(lineStr, ","));
            
            % Expect exactly six values: [ax, ay, az, gx, gy, gz]
            if numel(dataValues) == 6 && all(~isnan(dataValues))
                % Extract sensor data
                ax = dataValues(1);  % Acceleration (m/s^2)
                ay = dataValues(2);
                az = dataValues(3);
                gx = dataValues(4);  % Gyro (deg/s)
                gy = dataValues(5);
                gz = dataValues(6);
                
                % Convert gyro readings from deg/s to rad/s
                gyroSI  = deg2rad([gx; gy; gz]);
                accelSI = [ax; ay; az];  % Accelerometer data in m/s^2
                
                % Use the IMU filter to obtain an orientation quaternion
                q = fuse(accelSI', gyroSI');
                
                if isempty(initialQ)
                    initialQ = q;
                end
                
                % Compute the relative orientation (as a quaternion)
                relQ = quatmultiply(quatconj(initialQ), q);
                
                % Convert the relative quaternion to a rotation matrix
                R = quat2rotm(relQ);
                
                % Compute elapsed time since last update
                dt = toc(lastTime);
                lastTime = tic;
                
                % --- Position Update via Double Integration ---
                % Transform the raw acceleration to the world frame.
                % Note: accelSI includes gravity so we subtract the gravity vector.
                a_world = R * accelSI - [0; 0; g];
                
                % Apply deadzone: ignore very small accelerations
                if norm(a_world) < accelThreshold
                    a_world = [0; 0; 0];
                end
                
                % Update velocity and position (simple Euler integration)
                velocity = velocity + a_world * dt;
                
                % Apply deadzone for velocity to reduce drift over time
                if norm(velocity) < velThreshold
                    velocity = [0; 0; 0];
                end
                
                position = position + velocity * dt;
                
                % Option: If you expect the sensor to remain stationary,
                % you can simply force the position to remain at the origin:
                % position = [0; 0; 0];
                
                % Build the full 4x4 transformation matrix:
                % Upper-left 3x3 is the rotation; rightmost column is the translation.
                T = eye(4);
                T(1:3,1:3) = R;
                T(1:3,4) = position;
                
                % Update the hgtransform with the new transformation matrix
                set(t, 'Matrix', T);
                
                % Refresh the display
                drawnow limitrate;
            else
                fprintf("Skipping invalid line: %s\n", lineStr);
            end
        end
        pause(0.01); % Short pause to prevent excessive CPU usage
    end
    
end
