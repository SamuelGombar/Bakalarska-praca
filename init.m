%% Environment & road creation

%% Radars
sensorLength = 20;
sensorFov = pi/2;

%% Car
v1 = vehicle(scenario, 'ClassID',1', 'Position',[0 0 0], 'Velocity',[0.1 -0.5 0], 'Yaw',0);
v1.RearOverhang = 2.35;

L = 4.7;
m = 1000;
d = 0;
v = 0;

turningMax = 0.65;
maxSpeed = 0.5;

dF = 50;

%% Simulation
programStep = 0;
theta = 0;
incrementd = 0;

isCollision = false;

V = [];
Incrementd = [];
[sizeBoundary, ~] = size(outerBoundary);
calpha1 = 0;
calpha2 = 0;
calpha3 = 0;
calpha4 = 0;
calpha5 = 0;
calpha6 = 0;
calpha7 = 0;
calpha8 = 0;
calpha9 = 0;
alpharray1 = [];
alpharray2 = [];
alpharray3 = [];
alpharray4 = [];
alpharray5 = [];
alpharray6 = [];
alpharray7 = [];
alpharray8 = [];
alpharray9 = [];
