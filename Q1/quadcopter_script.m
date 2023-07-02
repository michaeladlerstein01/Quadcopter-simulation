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

num_drones = 1 ;

dist = [];
angles =  [];
%instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    drones = [drones Drone(ax1, spaceDim, num_drones)];
end


while(drones(1).time < 8)
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
    
    dist = [dist, drones.pos];
    angles = [angles , drones.angles];
    
    %apply fancy lighting (optional)
    %camlight

    %update figure
    drawnow
    pause(0.02)
end

subplot(2,1,1)
plot(0.02:0.02:8 , dist)
title('position over time')
xlabel('time')
ylabel('position')
legend('x' ,'y' ,'z' )

subplot(2,1,2)
plot(0.02:0.02:8 , angles)
title('angles over time')
xlabel('time')
ylabel('angles')
legend('roll' ,'pitch' ,'yaw' )

T_dist = table(dist);
T_angles = table(angles);
writetable(T_dist);
writetable(T_angles)

