clc
clear;
close all;
addpath('C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab');
client = RemoteAPIClient();
sim = client.require('sim');
sim.startSimulation();
%% handles

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
    sim.setJointTargetPosition(foot2(i), -30*pi / 180);
    sim.setJointTargetPosition(foot3(i), 120*pi / 180);
end
%
%%     foot1 -> foot2 -> foot3 -> foottip ( 1->6 ) 
%      angles data___ theta1 -> theta2-> theta3

c = 0.0515; % length of foot1
b = 0.072385; % length of foot2
a = 0.1161; % length of foot3
H = +0.066; %height fro grd

boundary =0.1;%% error we want

dxx=0.124;dyy=0; %% move wrt first joint

i = 1; %% for leg 1 only

%% ik calculation

theta1 = atan2(dyy,dxx); %% got theta1

dx = hypot(dxx,dyy);

if(dx<=c)
    fprintf("dx <= c error/n");
else
    delta =hypot(dx-c, H);

    num=a^2 +b^2-delta^2;
    deno=2*a*b;
    if(num/deno)>1
        fprintf("error in theta3\n");
    else
        thetacal = acos(num/deno);
        theta3 = pi - thetacal; %% got theta3
    end 

    num = b^2 + delta^2 -a^2;
    deno = 2*b*delta;
    thetacal2 = acos(num/deno);
    
    thetacal3= abs(atan2((dx-c),H));

    theta2=thetacal2+thetacal3-pi/2;
    theta2 =theta2 * -1; %% got theta2
end
%
%% set angles
sim.setJointTargetPosition(foot1(i), theta1);
sim.setJointTargetPosition(foot2(i), theta2);
sim.setJointTargetPosition(foot3(i), theta3);

% disp(theta1*180/pi);
% disp (theta2*180/pi);
% disp (theta3*180/pi);
