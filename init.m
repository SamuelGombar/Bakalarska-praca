%% Environment & road creation
scenario = drivingScenario;
Map2;

%% Car
v1 = vehicle(scenario, 'ClassID',1', 'Position',[0 0 0], 'Velocity',[0.1 -0.5 0], 'Yaw',0);
v1.RearOverhang = 2.35;
L = 4.7;
turningMax = 0.33;
maxSpeed = 0.45;
incrementd = 0;
theta = 0;
d = 0;
v = 0;

%% Radars
sensorLength = 15;
sensorFov = 5*pi/6; %5*pi/6;

%% Simulation
programStep = 0;
isCollision = false;
counter = 0;
[sizeBoundary, ~] = size(outerBoundary);
