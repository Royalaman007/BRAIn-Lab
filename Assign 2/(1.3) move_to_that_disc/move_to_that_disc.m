clc;
clear;
close all;
addpath('C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab');
client = RemoteAPIClient();
sim = client.require('sim');
sim.startSimulation();
%% handles
spo = sim.getObject('/Disc');
flag = sim.getObjectPosition(spo,-1);
robot =sim.getObject('/hexapod');
for i =1:6
    foot1(i) = sim.getObject( ...
        sprintf('/joint1_[%d]',  i-1));
end
for i =1:6
    foot2(i) = sim.getObject( ...
        sprintf('/joint1_[%d]/link1Respondable_/joint2_',  i-1));
end
for i =1:6
    foot3(i) = sim.getObject( ...
        sprintf('/joint1_[%d]/link1Respondable_/joint2_/link2Respondable_/joint3_',  i-1));
end
for i =1:6
    foottip(i) = sim.getObject( ...
        sprintf('/footTip%d',  i-1));
end

for i =1:6
    sim.setJointTargetPosition(foot1(i), 0*pi / 180);
    sim.setJointTargetPosition(foot2(i), -45*pi / 180);
    sim.setJointTargetPosition(foot3(i), 120*pi / 180);
end
%% foot1 -> foot2 -> foot3 -> foottip ( 1->6 ) 
%% angles data___ angle1 -> angle2-> angle3
c = 0.0515; % length of foot1
b = 0.072385; % length of foot2
a = 0.1161; % length of foot3
H = +0.066; %height fro grd


boundary =0.1;
forwardx = -.22;backwardx =.22;
lefty=0.2;righty=-0.2;
io=flag{1};
ij=flag{2};
points = [io ij];

pos = sim.getObjectPosition(robot,-1);
error = [points(1)-pos{1} points(2)-pos{2} ];
% deltax= 0;deltay=0.2;
deltax=0;deltay=0;
aman =1;akash=9;u=0;
leg = [1 4 2 6 5 3];ego=1;
while aman~=akash
    %%
    if mod(u, 2)  == 0
        if error(2) > boundary
            deltay =  righty;deltax= 0;fprintf("done akash\n");akash=8;
        elseif error(2)<-boundary
            deltay =  lefty;deltax=0;fprintf("done --akash\n");akash=8;
        else
            akash=0;fprintf("akash\n");
        end
        ego =2;
    else
        if error(1)>boundary
            deltax = forwardx;deltay=0;fprintf("done aman\n");aman = 7;
        elseif error(1)<-boundary
            deltax = backwardx;deltay=0;fprintf("done ---aman\n");aman = 7;
        else
            aman = 0;fprintf("aman\n");
        end
        ego =1;
    end
    %%
    %%
    for l = 1:length(leg)
        
        pos = sim.getObjectPosition(robot,-1);
        error = [points(1)-pos{1} points(2)-pos{2} ];
        if abs(hypot(error(1),error(2)) )<boundary

        else
        
            i = leg(l);
            pos = sim.getObjectPosition(robot,-1);
            error = [points(1)-pos{1} points(2)-pos{2} ];

            
            %rotation
                if i ==1
                    rotangle = pi/6;
                elseif i ==2
                    rotangle =pi/2;
                elseif i ==3
                    rotangle = pi*5/6;
                elseif i ==4
                    rotangle = -pi*5/6;
                elseif i == 5
                    rotangle = -pi/2;
                elseif i== 6
                    rotangle = - pi/6;
                end
            
                dxx=deltay*sin(rotangle)+ deltax* cos(rotangle);
                dyy =deltay * cos(rotangle) -deltax*sin(rotangle);
                %
                
                theta1 = atan2(dyy,dxx);
                
                dx = hypot(dxx,dyy);
                
                delta =hypot(dx-c, H);
            
                num=a^2 +b^2-delta^2;
                deno=2*a*b;
                if(num/deno)>1
                    fprintf("error in theta3\n");
                else
                    thetacal = acos(num/deno);
                    theta3 = pi - thetacal;%%
                end 
            
                num = b^2 + delta^2 -a^2;
                deno = 2*b*delta;
                thetacal2 = acos(num/deno);
                
                thetacal3= abs(atan2((dx-c),H));
            
                theta2=thetacal2+thetacal3-pi/2;
                theta2 =theta2 * -1;%%
            
                
                % disp(theta1*180/pi);
                % disp (theta2*180/pi);
                % disp (theta3*180/pi);
                
                sim.setJointTargetPosition(foot1(i), theta1);
                sim.setJointTargetPosition(foot2(i), theta2);
                sim.setJointTargetPosition(foot3(i), theta3);
                pause(0.1);
          %%
                sim.setJointTargetPosition(foot1(i), 0*pi / 180);
                sim.setJointTargetPosition(foot2(i), -45*pi / 180);
                sim.setJointTargetPosition(foot3(i), 120*pi / 180);
                pause(0.1);
           
            
        end
    end
    u=u+1;
   %555
end