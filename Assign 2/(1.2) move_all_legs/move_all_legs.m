clc;
clear;
close all;

%% -------------------------------------------------
% Add ZMQ Remote API path and connect to CoppeliaSim
%% -------------------------------------------------
addpath('C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab');

client = RemoteAPIClient();
sim    = client.require('sim');

sim.startSimulation();

%% -------------------------------------------------
% Get object handles
%% -------------------------------------------------
robot = sim.getObject('/hexapod');

% Joint 1 handles (hip yaw)
for i = 1:6
    foot1(i) = sim.getObject(sprintf('/joint1_[%d]', i-1));
end

% Joint 2 handles (hip pitch)
for i = 1:6
    foot2(i) = sim.getObject( ...
        sprintf('/joint1_[%d]/link1Respondable_/joint2_', i-1));
end

% Joint 3 handles (knee)
for i = 1:6
    foot3(i) = sim.getObject( ...
        sprintf('/joint1_[%d]/link1Respondable_/joint2_/link2Respondable_/joint3_', i-1));
end

% Foot tip handles
for i = 1:6
    foottip(i) = sim.getObject(sprintf('/footTip%d', i-1));
end

%% -------------------------------------------------
% Set initial joint configuration for all legs
%% -------------------------------------------------
for i = 1:6
    sim.setJointTargetPosition(foot1(i),  0    * pi/180);
    sim.setJointTargetPosition(foot2(i), -45   * pi/180);
    sim.setJointTargetPosition(foot3(i), 120   * pi/180);
end

%% -------------------------------------------------
% Robot geometry parameters
%% -------------------------------------------------
% foot1 -> foot2 -> foot3 -> foottip
c = 0.0515;     % length of link 1
b = 0.072385;   % length of link 2
a = 0.1161;     % length of link 3
H = 0.066;      % height from ground

%% -------------------------------------------------
% Motion parameters
%% -------------------------------------------------
boundary   = 0.2;

forwardx  = -0.22;
backwardx =  0.22;
lefty     =  0.2;
righty    = -0.2;

points = [0 0];   % desired robot (x,y)

%% -------------------------------------------------
% Initial robot position and error
%% -------------------------------------------------
pos   = sim.getObjectPosition(robot, -1);
error = [points(1)-pos{1}, points(2)-pos{2}];

deltax = 0;
deltay = 0;

aman  = 1;
akash = 9;

%% -------------------------------------------------
% Main control loop
%% -------------------------------------------------
while aman ~= akash

    % Leg execution order
    leg = [1 4 2 6 5 3];

    % Update robot position and error
    pos   = sim.getObjectPosition(robot, -1);
    error = [points(1)-pos{1}, points(2)-pos{2}];

    %% -------- X-direction correction --------
    if error(1) > boundary
        deltax = forwardx; 
        deltay = 0;
        fprintf("done aman\n");
        aman = 7;

    elseif error(1) < -boundary
        deltax = backwardx; 
        deltay = 0;
        fprintf("done ---aman\n");
        aman = 7;

    else
        aman = 0;
        fprintf("aman\n");
    end

    %% -------- Move legs for X correction --------
    for l = 1:length(leg)

        i = leg(l);

        % Rotation angle of each leg w.r.t body
        if     i == 1, rotangle =  pi/6;
        elseif i == 2, rotangle =  pi/2;
        elseif i == 3, rotangle = 5*pi/6;
        elseif i == 4, rotangle = -5*pi/6;
        elseif i == 5, rotangle = -pi/2;
        elseif i == 6, rotangle = -pi/6;
        end

        % Transform global dx,dy into leg frame
        dxx = deltay*sin(rotangle) + deltax*cos(rotangle);
        dyy = deltay*cos(rotangle) - deltax*sin(rotangle);

        % Inverse kinematics
        theta1 = atan2(dyy, dxx);
        dx     = hypot(dxx, dyy);

        if dx <= c
            fprintf("dx <= c error\n");
        else
            delta = hypot(dx-c, H);

            % Theta 3 calculation
            num  = a^2 + b^2 - delta^2;
            deno = 2*a*b;

            if (num/deno) > 1
                fprintf("error in theta3\n");
            else
                thetacal = acos(num/deno);
                theta3   = pi - thetacal;
            end

            % Theta 2 calculation
            num  = b^2 + delta^2 - a^2;
            deno = 2*b*delta;

            thetacal2 = acos(num/deno);
            thetacal3 = abs(atan2(dx-c, H));

            theta2 = thetacal2 + thetacal3 - pi/2;
            theta2 = -theta2;
        end

        % Apply joint angles
        sim.setJointTargetPosition(foot1(i), theta1);
        sim.setJointTargetPosition(foot2(i), theta2);
        sim.setJointTargetPosition(foot3(i), theta3);
        pause(0.1);

        % Reset to default pose
        sim.setJointTargetPosition(foot1(i),  0*pi/180);
        sim.setJointTargetPosition(foot2(i), -45*pi/180);
        sim.setJointTargetPosition(foot3(i), 120*pi/180);
        pause(0.1);
    end

    %% -------- Y-direction correction --------
    pos   = sim.getObjectPosition(robot, -1);
    error = [points(1)-pos{1}, points(2)-pos{2}];

    if error(2) > boundary
        deltay = righty;
        deltax = 0;
        fprintf("done akash\n");
        akash = 8;

    elseif error(2) < -boundary
        deltay = lefty;
        deltax = 0;
        fprintf("done --akash\n");
        akash = 8;

    else
        akash = 0;
        fprintf("akash\n");
    end

    %% -------- Move legs for Y correction --------
    for l = 1:length(leg)

        i = leg(l);

        % Rotation angle of leg
        if     i == 1, rotangle =  pi/6;
        elseif i == 2, rotangle =  pi/2;
        elseif i == 3, rotangle = 5*pi/6;
        elseif i == 4, rotangle = -5*pi/6;
        elseif i == 5, rotangle = -pi/2;
        elseif i == 6, rotangle = -pi/6;
        end

        dxx = deltay*sin(rotangle) + deltax*cos(rotangle);
        dyy = deltay*cos(rotangle) - deltax*sin(rotangle);

        theta1 = atan2(dyy, dxx);
        dx     = hypot(dxx, dyy);

        if dx <= c
            fprintf("dx <= c error\n");
        else
            delta = hypot(dx-c, H);

            num  = a^2 + b^2 - delta^2;
            deno = 2*a*b;

            if (num/deno) > 1
                fprintf("error in theta3\n");
            else
                thetacal = acos(num/deno);
                theta3   = pi - thetacal;
            end

            num  = b^2 + delta^2 - a^2;
            deno = 2*b*delta;

            thetacal2 = acos(num/deno);
            thetacal3 = abs(atan2(dx-c, H));

            theta2 = thetacal2 + thetacal3 - pi/2;
            theta2 = -theta2;
        end

        sim.setJointTargetPosition(foot1(i), theta1);
        sim.setJointTargetPosition(foot2(i), theta2);
        sim.setJointTargetPosition(foot3(i), theta3);
        pause(0.1);

        sim.setJointTargetPosition(foot1(i),  0*pi/180);
        sim.setJointTargetPosition(foot2(i), -45*pi/180);
        sim.setJointTargetPosition(foot3(i), 120*pi/180);
        pause(0.1);
    end
end
