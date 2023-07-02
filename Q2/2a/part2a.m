
I = [1 0 0; 0 1 0; 0 0 0.5];
m = 0.2;
g = 9.8;
kd = 0.1;
k = 1; 
L = 0.2; 
b = 0.1;
spaceDim = 15;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];
%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');
end
%figure to display drone simulation
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
syms x [12 ,1] 
syms xd [12,1] 
syms u [4,1] 
syms omegas [3,1]
phi = x(7);
theta = x(8);
psi = x(9);
%define the equilibrium points of states 
equilibrium = [0;0;0;0;0;0;0;0;0;0;0;0].' ;
%define equilibrium points of inputs
u_equilibrium = ones(4,1);
u_equilibrium(:) = m*g/4;
%compute state derivatives
xd(1:3)  = x(4:6);
xd(4:6) = drones.acceleration(u , [phi ; theta ; psi], [x4 ; x5 ;x6] , m , g , k , kd );
xd(7:9) = drones.omega2thetadot([x10 ; x11 ; x12] , [phi ; theta ; psi]) ;
xd(10:12) = drones.angular_acceleration(u , [x10 ; x11 ; x12] , I , L , b , k);
%find jacobian of the A and B matrix 
J_A = jacobian(xd , x);
J_B = jacobian(xd , u);
% substitutes the values in the vector equilibrium for the elements of x in the symbolic matrix J_A.
A =subs(J_A , x(1:12).' , equilibrium);
% the values in the vector u_equilibrium are substituted for the elements of u in the matrix A. This means that the matrix A 
A = double(subs(A , u , u_equilibrium));
B = double(subs(J_B , x(1:12).' , equilibrium));
C = eye(size(A));
D = zeros(size(B));
%find state space model 
cont_sys = ss(A,B,C,D);
%discretize state space model with zero order hold
disc = c2d(cont_sys , drones.time_interval , 'zoh');
A_d = disc.A
B_d = disc.B


