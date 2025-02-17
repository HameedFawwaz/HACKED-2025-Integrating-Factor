%% this code does not work. It will tell you that your box has left on an
%% expedition to hell

function liveIMU3DBox_Kalman()

    %% port info and setup
    portName   = "COM4";    % 4 or 7
    baudRate   = 115200;    % 115200 or 921600
    sampleRate = 100;       
    g = 9.25;      
    dt         = 1 / sampleRate;   % fixed time step

    % dr-fit control perchance
    zuptThreshold = 0.05;  % m/s

    % Init
    P_init = diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]);
    Q = diag([1e-6, 1e-6, 1e-6, 5e-3, 5e-3, 5e-3]);
    R_zupt = diag([1e-3, 1e-3, 1e-3]);

    %% serial stuffs fr
    s = serialport(portName, baudRate);
    configureTerminator(s, "LF");
    flush(s);

    %% orientation filter
    fuse = imufilter('SampleRate', sampleRate);

    %% "THE" CUBE setup
    figure('Name','3D IMU + KF Demo','NumberTitle','off');
    clf;
    axis equal; grid on; view(3); hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Kalman Filter: Orientation + Position');
    xlim([-10 10]); ylim([-10 10]); zlim([-5 25]);

    % "THE CUBE"
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
    faces = [
        1 2 3 4;
        5 6 7 8;
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8
    ];
    centeredVertices = vertices - 0.5; 
    t3d = hgtransform;
    patch('Vertices', centeredVertices, 'Faces', faces, ...
          'FaceColor', 'cyan', 'FaceAlpha', 0.3, ...
          'Parent', t3d);

    %% Kalman filter state
    initialQ = [];
    X = zeros(6,1);  % 0 velocity
    P = P_init;      % initial covariance

    disp('Reading IMU data...  (Ctrl+C to stop)');
% I honestly don't know how some of these functions work
    while true
        while s.NumBytesAvailable > 0
            lineStr = readline(s);
            dataValues = str2double(split(lineStr, ","));
            if numel(dataValues) ~= 6
                fprintf("Skipping invalid line: %s\n", lineStr);
                continue;
            end

            % sensro data from serial 
            ax = dataValues(1);  % m/s^2
            ay = dataValues(2);
            az = dataValues(3);
            gx = dataValues(4);  % deg/s
            gy = dataValues(5);
            gz = dataValues(6);

            % Convert gyro to rad/s
            gyroRad = deg2rad([gx; gy; gz]);
            accel   = [ax; ay; az];

            %% orientation
            q = fuse(accel', gyroRad');
            if isempty(initialQ)
                initialQ = q;  % store initial reference
            end
            relQ = quatmultiply(quatconj(initialQ), q); 
            Rbody_to_global = quat2rotm(relQ);

            %% global acceleration without g
            aGlobal = Rbody_to_global * accel;
            % Z+ is up, no orientation alignment aside from init
            aGlobal(3) = aGlobal(3) - g;

            %% Kalman filter step 
            % predict
            [Xpred, Ppred] = kf_predict(X, P, aGlobal, Q, dt);

            % Zero-Velocity Update
            velNorm = norm(Xpred(4:6));
            if velNorm < zuptThreshold
                % velocity = 0
                z_meas = [0;0;0];
                H_zupt = [0 0 0 1 0 0; 
                          0 0 0 0 1 0; 
                          0 0 0 0 0 1];
                [X, P] = kf_update(Xpred, Ppred, z_meas, H_zupt, R_zupt);
            else
                X = Xpred;
                P = Ppred;
            end

            %% 3d update
            Tmat = eye(4);
            Tmat(1:3,1:3) = Rbody_to_global;   % orientation
            Tmat(1:3,4)   = X(1:3);           % global position
            set(t3d, 'Matrix', Tmat);

            %% picasso
            drawnow limitrate;
        end
        pause(0.01);  % put this or go boom
    end

end

function [Xpred, Ppred] = kf_predict(X, P, aGlobal, Q, dt)
    % State dimension
    nx = 6;
    % discrete time matrix
    A = [1 0 0 dt 0  0 ;
         0 1 0 0  dt 0 ;
         0 0 1 0  0  dt;
         0 0 0 1  0  0 ;
         0 0 0 0  1  0 ;
         0 0 0 0  0  1 ];

    % B matrix for the acceleration input
    B = [0 0 0; 
         0 0 0; 
         0 0 0; 
         dt 0 0; 
         0 dt 0; 
         0 0 dt];

    % Predict state
    Xpred = A * X + B * aGlobal;

    % pos accuracy
    % Xpred(1:3) = Xpred(1:3) + 0.5 * aGlobal * (dt^2);

    % Predict covariance
    Ppred = A * P * A' + Q;
end

%% update kalman, comment if no work
function [Xupd, Pupd] = kf_update(Xpred, Ppred, z_meas, H, R)
    %innovation
    z_pred = H * Xpred;
    y = z_meas - z_pred;

    % Innovation covariance
    S = H * Ppred * H' + R;

    % Kalman gain
    K = Ppred * H' / S;

    % Update
    Xupd = Xpred + K * y;

    % Joseph form for covariance update
    I = eye(length(Xpred));
    Pupd = (I - K*H) * Ppred * (I - K*H)' + K * R * K';
end
