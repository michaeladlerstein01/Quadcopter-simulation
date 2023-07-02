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

%load the data from q1
q1table_dist = readtable('T_dist.txt');
q1table_angles = readtable("T_angles.txt");
q1_dist = table2array(q1table_dist);
q1_angles = table2array(q1table_angles);


figure(2)
title('position over time')
xlabel('x position')
ylabel('y position')
zlabel('z position')
plot3(dist(1,:) , dist(2,:) , dist(3,:));
axis("equal")
axis([-1 0 -5 0 0 9])
hold on
plot3(q1_dist(1,:),q1_dist(2,:),q1_dist(3,:))
title('linearised dynamics and non linearised dynamics')
legend('linearised' , 'non-linearised')

figure(3)
subplot(1,3,1)
plot(0:0.02:8 , angles(1,:)); hold on 
plot((0.02:0.02:8) ,q1_angles(1,:) , '--')
legend('linear' , 'non-linear')
title('roll changes')
subplot(1,3,2)
plot(0:0.02:8 , angles(2,:)); hold on 
plot((0.02:0.02:8) ,q1_angles(2,:) , '--')
title('pitch changes')
subplot(1,3,3)
plot(0:0.02:8 , angles(3,:)); hold on 
plot((0.02:0.02:8) ,q1_angles(3,:) , '--')
title('yaw changes')

