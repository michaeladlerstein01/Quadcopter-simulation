%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 22;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];

%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');
end

%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d
time_interval = 0.01;
num_drones = 1 ;


%instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    drones = [drones Drone(ax1, spaceDim, num_drones , time_interval)];
end
dist = []
angles = []

while(drones(1).time < 100 && drones.count < 25)
    %clear axis
    cla(ax1);
   

    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end
    %optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
    dist = [dist , drones.pos];
    angles = [angles , drones.angles];

    
    %apply fancy lighting (optional)
    %camlight

    %update figure
    drawnow
    %pause(0.04)
end
plot3(dist(1,:),dist(2,:),dist(3,:))

subplot(1,3,1)
plot(angles(1,:))
title('roll')

subplot(1,3,2)
plot(angles(2,:))
title('pitch')

subplot(1,3,3)
plot(angles(3,:))
title('yaw')


