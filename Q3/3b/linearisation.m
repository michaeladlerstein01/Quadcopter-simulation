function[Ad , Bd ]  = linearisation(obj)
%     I = [1 0 0; 0 1 0; 0 0 0.5];
%     m = 0.2;
%     g = 9.8;
%     kd = 0.1;
%     k = 1; 
%     L = 0.2; 
%     b = 0.1;
%     spaceDim = 15;
%     spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];
%     %do you want to draw a ground image on the figure?
%     draw_ground = false;
%     if(draw_ground)
%         ground_img = imread('ground.png');
%     end
%     %figure to display drone simulation
%     ax1 = gca;
%     view(ax1, 3);
%     axis equal;
%     axis(spaceLimits)
%     grid ON
%     grid MINOR
%     caxis(ax1, [0 spaceDim]);
%     hold(ax1,'on')
%     axis vis3d
%     num_drones = 1 ;
%    
%     %instantiate a drone object, input the axis and arena limits
%     drones = [];
%     for i = 1:num_drones
%         drones = [drones Drone(ax1, spaceDim, num_drones)];
%     end
%     syms x [12 ,1] 
%     syms xd [12,1] 
%     syms u [4,1] 
%     syms omegas [3,1]
%     phi = x(7);
%     theta = x(8);
%     psi = x(9);
%     %define the equilibrium points of states 
%     equilibrium = [0;0;0;0;0;0;0;0;0;0;0;0].' ;
%     %define equilibrium points of inputs
%     u_equilibrium = ones(4,1);
%     u_equilibrium(:) = m*g/4;
%     %compute state derivatives
%     xd(1:3)  = x(4:6);
%     xd(4:6) = drones.acceleration(u , [phi ; theta ; psi], [x4 ; x5 ;x6] , m , g , k , kd );
%     xd(7:9) = drones.omega2thetadot([x10 ; x11 ; x12] , [phi ; theta ; psi]) ;
%     xd(10:12) = drones.angular_acceleration(u , [x10 ; x11 ; x12] , I , L , b , k);
%     %find jacobian of the A and B matrix 
%     J_A = jacobian(xd , x);
%     J_B = jacobian(xd , u);
%    
%     % substitutes the values in the vector equilibrium for the elements of x in the symbolic matrix J_A.
%     A =subs(J_A , x(1:12).' , equilibrium);
%     % the values in the vector u_equilibrium are substituted for the elements of u in the matrix A. This means that the matrix A 
%     A = double(subs(A , u , u_equilibrium));
%     B = double(subs(J_B , x(1:12).' , equilibrium));
%     C = eye(size(A));
%     D = zeros(size(B));
%     %find state space model 
%     cont_sys = ss(A,B,C,D);
%     %discretize state space model with zero order hold
%     disc = c2d(cont_sys , drones.time_interval , 'zoh');
%     A_d = disc.A;
%     B_d = disc.B;
%     
% end 
% 
% function [Ad, Bd] = disc_linearisation(obj)
    syms x [12,1]
    syms xd [12,1]
    syms u [4,1]
    m = obj.m;
    g = obj.g;
    gravity=[0; 0; -obj.g];
    I = obj.I;
    equilibrium = [0;0;0;0;0;0;0;0;0;0;0;0].' ;
    u_equilibrium = ones(4,1);
    u_equilibrium(:) = m*g/4;
    phi = x(7);
    theta = x(8);
    psi = x(9);
    
    Rx = [1 0 0;
    0 cos(phi) -sin(phi); ... 
    0 sin(phi) cos(phi)];
    
    Ry = [ cos(theta) 0 sin(theta);
    0 1 0; ...
    -sin(theta) 0 cos(theta)];
    
    Rz = [cos(psi) -sin(psi) 0; ... 
        sin(psi) cos(psi) 0; ... 
        0 0 1];

    R = Rz*Ry*Rx;

    Fd = -obj.kd * [x(4); x(5); x(6)] ;
    
    tau = [
    obj.L * obj.k * (u(1) - u(3))
    obj.L * obj.k * (u(2) - u(4))
    obj.b * (u(1) - u(2) + u(3) - u(4))];

    Tb = [0; 0; obj.k * sum(u)];
    
    %% state space
    xd(1:3) = x(4:6);
    xd(4:6) = gravity +(1/obj.m) * R *(Tb) + (1/obj.m) *Fd;
    xd(7:9) = [1, 0, -sin(x(8));
        0, cos(x(7)), cos(x(8)) * sin(x(7));
        0, -sin(x(7)), cos(x(8)) * cos(x(7))] \ x(10:12);

    xd(10:12) = I\tau - [...
        (I(2,2)-I(3,3))/I(1,1)*xd(8)*xd(9);...
        (I(3,3)-I(1,1))/I(2,2)*xd(7)*xd(9);...
        (I(1,1)-I(2,2))/I(3,3)*xd(7)*xd(8)];
    
    Aj = jacobian(xd,x);
    Bj = jacobian(xd,u);
    
    A =  subs(Aj,...
        [x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12],...
        equilibrium);
    A = double (subs(A, u, u_equilibrium));
    B = double (subs(Bj,...
        [x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12],...
        equilibrium));

    C = eye(size(A));
    D = zeros(size(B));
    
    cont_sys = ss(A,B,C,D);
    disc_sys = c2d(cont_sys,obj.time_interval,'zoh');
    obj.cont_sys = cont_sys;
    Ad = disc_sys.A;
    Bd = disc_sys.B;
    
end