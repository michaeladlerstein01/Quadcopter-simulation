%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [2 2 0.0];
        
        %time interval for simulation (seconds)
        time_interval = 0.02;
        
        % size of floating window that follows drone
        axis_size = 2.;
        
        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];
        
        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = true;

        


        %%%%%ATTRIBUTES THAT WE DEFINE%%%%%%%%%%%%%%%%%%%%%%%

        I = [1 0 0; 0 1 0; 0 0 0.5];
        m = 0.2;
        g = 9.8;
        k_d = 0.1;
        k = 1; 
        L = 0.2; 

        %drag coefficient
        b = 0.1;
        

       
    end
    properties
        %axis to draw on
        axis
        
        %length of one side of the flight arena
        spaceDim 
        
        %limits of flight arena
        spaceLimits 
        
        %drone position
        pos
        
        %drone rotation matrix
        R
        
        %Simulation time
        time
        
        %parameter to start drone in random position
        pos_offset
        
        %number of drones
        num_drones 

        %%%instances that we define to make the simulation run %%%%%
        i 
        T
        angles
        anglesdot
        xdot
        a
        rot_mat
        prev_xdot
        omegadot 
        omega
        tau
        thetadot

        

        



    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone(axis, spaceDim, num_drones)
            if nargin > 1
                obj.axis = axis;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                                
                obj.pos_offset = [0;0;5];

                obj.pos = obj.pos_offset ;

                
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;
                
                obj.omega = [0;0;0];
                                  
                %parameters I added
                obj.angles = [ 0 ; 0 ; 0];
                obj.prev_xdot = [0;0;0];
                obj.xdot = [0;0;0];
                obj.pos_offset = [0; 0; 5];
                obj.i = [0 ,0 ,0, 0];
                obj.time = obj.time + obj.time_interval;
                obj.omega = [0;0;0];
                obj.anglesdot = [0;0;0];
                
            else
                error('Drone not initialised correctly')
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %how big should the moving window be
            cL = obj.axis_size;
            
            %set to false if you want to see world view
            %if(obj.drone_follow)
            %    axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            %end
            
            %create middle sphere
            [X Y Z] = sphere(8);
            %[X Y Z] = (obj.body(1)/5.).*[X Y Z];
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end
        
        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %demo (not useful) code to show varying position and rotation
        %replace with your own functions!
%          function obj = change_pos_and_orientation(obj)
%             t = obj.time;
% 
%             %vary position
%             pos_mat =  [2.6*sin(t/2), 2.1*cos(t/5),(2.4*sin(t/3)*sin(t/3))] + obj.pos_offset;
% 
%             %vary orientation
%             pitch = 0.3*sin(t*15.2);
%             roll = 0.1*cos(t*33.1 + 0.5);
%             yaw = 2.*pi*sin(t);
%             rot_mat = eul2rotm([yaw, roll, pitch]);
% 
%             %update position and rotation matrix of drone
%             obj.pos = pos_mat;
%             obj.R = rot_mat;
% 
%         end


        %%%%PHYSISCS PARAMETERS%%%%%%%%%%%%%%%%%%%
        function T = thrust(obj, inputs , k)
            T = [0;0;k*sum(inputs)];
        end 
        
        function tau = torques(obj , inputs , L , b , k)
            tau = [
                L * k * (inputs(1) - inputs(3));
                L * k * (inputs(2) - inputs(4));
                b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
            ];

        end 
        
        function a = acceleration( obj , inputs , angles , xdot , m , g, k , kd)
            gravity = [0;0;-g];
            R = obj.rotation(angles);

            obj.T = R * obj.thrust(inputs , k);

            Fd = -kd * xdot;
            a = gravity +  1/m * obj.T + 1/m * Fd;
        end 
        
        function rot_mat = rotation(obj , angles)
            phi = angles(1);
            theta = angles(2);
            psi = angles(3);

            Rx = [1 0 0; 0 cos(phi) -sin(phi);  0 sin(phi) cos(phi)];
            Ry = [ cos(theta) 0 sin(theta);0 1  0; -sin(theta) 0 cos(theta)];
            Rz = [cos(psi) -sin(psi) 0;  sin(psi)  cos(psi) 0;  0  0 1];
            rot_mat = Rz*Ry*Rx;
        end 


        %entract angles, write down rotation matrix 


        function omegadot = angular_acceleration(obj , inputs , omega , I , L , b , k)
            tau = obj.torques(inputs , L , b , k);
            omegadot = inv(I) * (tau - cross(omega , I * omega));
        end 
        
       
        function omega = thetadot2omega(obj , anglesdot , angles)
            phi = angles(1);
            theta = angles(2);
            psi = angles(3);
            omega = [ 1,         0,         -sin(theta); 
                      0,        cos(phi),  cos(theta)*sin(phi);
                      0,       -sin(phi),  cos(theta)*cos(phi) ] * anglesdot;
        end 

        function anglesdot = omega2thetadot(obj , omega , angles)
            phi = angles(1);
            theta = angles(2);
            psi = angles(3);
            Rot = [ 1,         0,         -sin(theta); 
                   0,        cos(phi),  cos(theta)*sin(phi);
                   0,       -sin(phi),  cos(theta)*cos(phi) ];
            anglesdot = inv(Rot) * omega;
        end 

        




        function update(obj)
            %update simulation time
            
            obj.time = obj.time + obj.time_interval;
            x= obj.m*obj.g/4;
            if obj.time > 0 && obj.time < 2

                %obj.i = [obj.m*obj.g/4 , obj.m*obj.g/4 , obj.m*obj.g/4 ,obj.m*obj.g/4 ];
                %linear acceleratio
                obj.i(:) = x;

                obj.a = acceleration(obj, obj.i ,obj.angles, obj.xdot , obj.m, obj.g, obj.k, obj.k_d );

                obj.xdot = obj.a * obj.time_interval + obj.xdot;
             
                obj.pos = obj.xdot * obj.time_interval + obj.pos;

                %angular acceleration
                obj.omegadot = angular_acceleration(obj , obj.i , obj.omega , obj.I , obj.L , obj.b , obj.k);


                obj.omega = obj.omega + obj.time_interval* obj.omegadot;

                
                obj.anglesdot = obj.omega2thetadot( obj.omega , obj.angles);

                obj.angles = obj.angles + obj.anglesdot * obj.time_interval;

                obj.rot_mat = rotation(obj , obj.angles);
                
                obj.R = obj.rot_mat;

                

            end 

            if obj.time > 2 && obj.time < 4

                obj.i(:) = 1.15*x;
                

                obj.omega = thetadot2omega(obj, obj.anglesdot , obj.angles)
                

                obj.a = acceleration(obj , obj.i ,obj.angles, obj.xdot , obj.m, obj.g, obj.k, obj.k_d );

                %angular acceleration
                obj.omegadot = angular_acceleration(obj , obj.i , obj.omega , obj.I , obj.L , obj.b , obj.k);


                obj.omega = obj.omega + obj.time_interval* obj.omegadot;

                
                obj.anglesdot = obj.omega2thetadot( obj.omega , obj.angles);

                obj.angles = obj.angles + obj.anglesdot * obj.time_interval;

                obj.xdot = obj.a * obj.time_interval + obj.xdot;
             
                obj.pos = obj.xdot * obj.time_interval + obj.pos;

                obj.rot_mat = rotation(obj , obj.angles);
                
                obj.R = obj.rot_mat;


            end 
            if obj.time > 4 && obj.time <= 8
                
                obj.i(3)=0;
                
                obj.a = acceleration(obj , obj.i ,obj.angles, obj.xdot , obj.m, obj.g, obj.k, obj.k_d );

                obj.xdot = obj.a * obj.time_interval + obj.xdot;
             
                obj.pos = obj.xdot * obj.time_interval + obj.pos;

                %angular acceleration
                obj.omegadot = angular_acceleration(obj , obj.i , obj.omega , obj.I , obj.L , obj.b , obj.k);


                obj.omega = obj.omega + obj.time_interval* obj.omegadot;
                
                obj.anglesdot = obj.omega2thetadot( obj.omega , obj.angles);

                obj.angles = obj.angles + obj.anglesdot * obj.time_interval;

                obj.rot_mat = rotation(obj , obj.angles);
                
                obj.R = obj.rot_mat;

            end 
   
            %draw drone on figure
            draw(obj);
        end


    end
end
