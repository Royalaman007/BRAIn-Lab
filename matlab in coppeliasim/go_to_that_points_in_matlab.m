clc;
clear;
close all;

% Add ZMQ API path
addpath('C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab');

% Connect to CoppeliaSim
client = RemoteAPIClient();
sim = client.require('sim');

% disp("Connected to ZeroMQ API");

% Enable stepping mode
%sim.setStepping(true);
sim.startSimulation();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fl = sim.getObject('/front_left_wheel');
fr = sim.getObject('/front_right_wheel');

% disp("All joint handles received");

robot = sim.getObject('/RobotnikSummitXL');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

point =[
        -1.5 1;
        -1.7 1.6;
        1.4 1;
        0 0;
        1 1
        ];

speed = 10;
fprintf("\nMission Start\n\n");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1 : size(point, 1)

    p = point(i,1);  % the target point 
    q = point(i,2);
    fprintf("target point (x,y) = (%f,%f)\n", p,q);

    if p<=2 && q<=2
    
      
        pos = sim.getObjectPosition(robot, -1);
        x=[0,0];
        x(1) =p - pos{1};
        x(2) =q - pos{2};
        theta_in_radian = atan2(x(2),x(1));
        theta_in_degree = atan2(x(2),x(1)) * 57.2958;
        
        
        ori = sim.getObjectOrientation(robot, -1);
        g = ori{3} * 57.2958;
        
        
    
        
        % disp (theta_in_degree);
        % disp (g(1));
        
        
        
        nowori = sim.getObjectOrientation(robot, -1);
        nowori = ori{3} * 57.2958;
        
        diff = abs(nowori - theta_in_degree);
        
        if theta_in_degree >0  %anti
            while abs(diff)>10
                sim.setJointTargetVelocity(fl, -speed);
                sim.setJointTargetVelocity(fr, - speed);
                pause(0.05);
        
        
        
                nowori = sim.getObjectOrientation(robot, -1);
                nowori{3} = nowori{3} * 57.2958;
        
                diff = nowori{3} - theta_in_degree;
        
        
            end
        
        elseif theta_in_degree <0 %clock
            while abs(diff)>10
                sim.setJointTargetVelocity(fl, speed);
                sim.setJointTargetVelocity(fr, speed);
                pause(0.05);
        
        
                nowori = sim.getObjectOrientation(robot, -1);
                nowori{3} = nowori{3} * 57.2958;
        
                diff = nowori{3} - theta_in_degree;
        
        
        
            end
        
        end
        
        sim.setJointTargetVelocity(fl, 0);
        sim.setJointTargetVelocity(fr, 0);
    
        %go to point
    
        if x(1)>0 
        
            while x(1) >0 
                sim.setJointTargetVelocity(fl, speed);
                sim.setJointTargetVelocity(fr, -speed);
                pos = sim.getObjectPosition(robot, -1);
                x(1) =p - pos{1};
                x(2) =q - pos{2};    
            end
    
        elseif x(1)<0
    
            while x(1) <0
                sim.setJointTargetVelocity(fl, speed);
                sim.setJointTargetVelocity(fr, -speed);
                pos = sim.getObjectPosition(robot, -1);
                x(1) =p - pos{1};
                x(2) =q - pos{2};
            end
    
    
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
        % if x(2)>0 
        % 
        %     while x(2) >0 
        %         sim.setJointTargetVelocity(fl, speed);
        %         sim.setJointTargetVelocity(fr, -speed);
        %         pos = sim.getObjectPosition(robot, -1);
        %         x(1) =p - pos{1};
        %         x(2) =q - pos{2};    
        %     end
        % 
        % elseif x(2)<0
        % 
        %     while x(2) <0
        %         sim.setJointTargetVelocity(fl, speed);
        %         sim.setJointTargetVelocity(fr, -speed);
        %         pos = sim.getObjectPosition(robot, -1);
        %         x(1) =p - pos{1};
        %         x(2) =q - pos{2};
        %     end
        % 
        % 
        % end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
        
        sim.setJointTargetVelocity(fl, 0);
        sim.setJointTargetVelocity(fr, 0);
        pos = sim.getObjectPosition(robot, -1);
        
        fprintf("reached to point (x,y) = (%f,%f)\n\n",pos{1},pos{2});
        
        pause(2)


    else

        fprintf("cant go to x = %f and y = %f as it will fall\n\n",p,q);

    end



end

fprintf("mission complete RESPECT+++\n");